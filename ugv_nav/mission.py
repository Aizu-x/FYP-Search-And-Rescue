"""Autonomous survivor search (runs on the edge laptop, on top of the Navigator).

Flow after the operator presses AUTHORIZE with "Search for survivors at goal" ticked::

    TRANSIT (A* to the operator's goal)
       └─▶ SCAN   turn on the spot in steps, holding still at each step so YOLO can look
             ├─ green object confirmed ─▶ SURVIVOR pinged (list + map), then
             │                          APPROACH (A* to a point ``approach_gap_m`` in front of it)
             │                          ─▶ FACE it and stand there ``secure_hold_s`` seconds
             │                          ─▶ status "secured"
             │                          ─▶ another green object known? approach that one, otherwise
             │                             MISSION COMPLETE (the mission ends by itself)
             └─ nothing new ─▶ EXPLORE (A* to the free spot with the most floor the camera has not
                               seen yet) ─▶ SCAN ...   whole area seen ─▶ start a new pass

A green object confirmed at ANY time (also on the way to the goal) interrupts the current drive: the
robot goes to the first green object it sees. Until one is secured the mission keeps searching.

Safety
------
* Motion only starts after an operator authorization (AUTHORIZE / Resume mission).
* Every motion goes through the Navigator: same A*, LiDAR stop zone, UGV watchdog, speed limits.
* Operator STOP, link loss, localization loss and unknown faults PAUSE the mission; it only
  continues after the operator presses AUTHORIZE (Resume) again.
* Obstacles / failed routes are handled automatically (replan, try another spot), but after
  ``max_failures`` failures in a row the mission pauses for the operator.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from ugv_planner.collision import CollisionChecker
from ugv_planner.inflation import inflated_mask

from .navigator import RUNNING


@dataclass
class MissionConfig:
    survivor_class: str = 'green'       # YOLO class name prefix that marks a survivor
    approach_gap_m: float = 0.25        # stop this far in front of a survivor (robot front -> object)
    secure_hold_s: float = 3.0          # stand in front of it this long, then it counts as secured
    end_when_secured: bool = True       # no other green object known after securing: mission complete
    same_survivor_m: float = 0.30       # a green object this close to a known survivor is the same one
    object_size_m: float = 0.05
    scan_step_deg: float = 45.0         # look-around: turn this much ...
    scan_dwell_s: float = 1.5           # ... then hold still this long (YOLO confirms in 4 frames)
    scan_turn_rate: float = 0.30        # rad/s (vision pauses map updates above 0.35 rad/s)
    view_range_m: float = 1.2           # floor counted as "searched" up to this distance ...
    fov_deg: float = 60.0               # ... inside this camera field of view
    viewpoint_spacing_m: float = 0.25   # exploration candidates on this lattice
    min_unseen_cells: int = 30          # a viewpoint must reveal at least this many unseen cells
    max_failures: int = 6               # consecutive failed actions before pausing for the operator
    settle_s: float = 0.6               # robot must stand still this long between actions
    spot_clearance_m: float = 0.06      # prefer stopping spots with this much room beyond the planning radius


@dataclass
class Survivor:
    id: int
    x: float
    y: float
    found_s: float                      # seconds since mission start
    found_clock: str                    # wall-clock time (for the report)
    status: str = 'found'               # found | secured | unreachable
    track_ids: set = field(default_factory=set)
    main_track: object = None           # tracker id whose (averaged) position is shown
    attempts: int = 0

    def as_dict(self):
        return dict(id=self.id, x=self.x, y=self.y, found_s=self.found_s, found_clock=self.found_clock,
                    status=self.status)


PHASE_TEXT = dict(transit='Driving to goal', scan='Looking around', approach='Approaching survivor',
                  face='Securing survivor', explore='Exploring new area')


class SearchMission:
    def __init__(self, nav, config: MissionConfig = None, camera_active=lambda: True, clock=None):
        self.nav = nav
        self.cfg = config or MissionConfig()
        self.camera_active = camera_active
        self.clock = clock or nav.clock
        self.active = False
        self.paused = False
        self.complete = False             # ended by itself after securing the survivor(s)
        self.pause_reason = ''
        self.survivors = []
        self.events = []
        self.started_at = None
        self.passes = 1
        self.seen = None
        self.action = None                # dict(kind, seq, target)
        self.need_scan = False
        self.failures = 0
        self.blacklist = set()
        self._face = None
        self._still_since = None
        self._last_cover = 0.0
        self._pending_new = False
        self._final_elapsed = 0.0
        self._reach_cache = None
        self._reach_cell = None
        self._region_cache = None
        self._last_snapshot = None

    # ------------------------------------------------------------ operator actions
    def start(self, transit=True):
        """Begin the mission. ``transit``: the navigator was just authorized to drive to the goal."""
        with self.nav._lock:
            self.active, self.paused, self.pause_reason = True, False, ''
            self.complete = False
            self.nav.safety.estop = False        # start = explicit operator authorization (confirmed in the console)
            self.nav.auto_replans = 0
            self.started_at = self.clock()
            self.seen = np.zeros(self.nav.grid.shape, bool)
            self.passes, self.failures = 1, 0
            self.blacklist.clear()
            self._face = None
            self.need_scan = not transit
            self.action = dict(kind='transit', seq=self.nav.halt_seq, target=None) if transit else None
            self._event('Search mission started%s.' % (': driving to the goal first' if transit else ' here'))

    def stop(self):
        """End the mission (survivor list is kept). Latches STOP like the STOP button."""
        with self.nav._lock:
            was = self.active
            self.active = self.paused = False
            self.action = None
            self._final_elapsed = self.elapsed()
            if was:
                self._event('Search mission stopped by operator. %d survivor(s) found.' % len(self.survivors))
            self.nav.emergency_stop('Search mission stopped')

    def _complete(self):
        """Every green object seen is secured (or unreachable): the mission ends by itself. The robot is
        already standing still; nothing is latched, so the operator can plan a new route straight away."""
        self.active = self.paused = False
        self.complete = True
        self.action = None
        self._final_elapsed = self.elapsed()
        done = sum(s.status == 'secured' for s in self.survivors)
        self._event('MISSION COMPLETE: %d survivor(s) secured, no other green object in view.' % done)

    def pause(self, reason):
        if not self.active or self.paused:
            return
        self.paused = True
        self.pause_reason = reason
        self.action = None
        self._event('Mission PAUSED: %s. Fix it, then press AUTHORIZE to resume.' % reason)

    def resume(self):
        """Operator authorization to continue a paused mission."""
        with self.nav._lock:
            if not self.active or not self.paused:
                return False, 'No paused mission.'
            self.nav.safety.estop = False        # explicit operator authorization (confirmed in the console)
            self.paused, self.pause_reason = False, ''
            self.failures, self.action, self._still_since = 0, None, None
            self.nav.auto_replans = 0
            self._event('Mission resumed by operator.')
            return True, 'Mission resumed.'

    def clear_survivors(self):
        with self.nav._lock:
            self.survivors = []

    # ------------------------------------------------------------ helpers
    def _event(self, text):
        t = 0.0 if self.started_at is None else self.clock() - self.started_at
        self.events.append((t, text))
        del self.events[:-200]
        self.nav._note(text)

    def elapsed(self):
        return 0.0 if self.started_at is None else self.clock() - self.started_at

    def _pending_survivor(self):
        pose = self.nav.estimator.pose()
        todo = [s for s in self.survivors if s.status == 'found']
        if not todo:
            return None
        if pose is None:
            return todo[0]
        return min(todo, key=lambda s: math.dist((s.x, s.y), pose[:2]))

    def _register_survivors(self):
        live = {o.get('id') for o in self.nav.vision_obstacles}
        for o in self.nav.vision_obstacles:
            if not str(o.get('cls', '')).startswith(self.cfg.survivor_class):
                continue
            tid = o.get('id')
            match = next((s for s in self.survivors if tid in s.track_ids), None)
            if match is None:
                near = [s for s in self.survivors if math.dist((s.x, s.y), (o['x'], o['y'])) <= self.cfg.same_survivor_m]
                match = min(near, key=lambda s: math.dist((s.x, s.y), (o['x'], o['y']))) if near else None
            if match is not None:
                match.track_ids.add(tid)
                if match.main_track not in live:          # its first track was dropped: follow this one
                    match.main_track = tid
                if tid == match.main_track:
                    match.x, match.y = o['x'], o['y']     # tracker average gets better with every frame
                continue
            s = Survivor(len(self.survivors) + 1, o['x'], o['y'], round(self.elapsed(), 1),
                         time.strftime('%H:%M:%S'), track_ids={tid}, main_track=tid)
            self.survivors.append(s)
            self._pending_new = True
            self._event('SURVIVOR S%d found at (%.2f, %.2f) m.' % (s.id, s.x, s.y))

    def _update_coverage(self, pose, now):
        """Mark floor inside the camera view (up to the first wall) as searched."""
        if pose is None or self.seen is None or now - self._last_cover < 0.25 or not self.camera_active():
            return
        tel = self.nav.client.snapshot()['telemetry'] or {}
        if abs((tel.get('odom') or {}).get('wz', 0.0)) > 0.35:
            return                                  # vision ignores frames while turning fast
        self._last_cover = now
        grid = self.nav.grid
        if self.seen.shape != grid.shape:
            self.seen = np.zeros(grid.shape, bool)
        half = math.radians(self.cfg.fov_deg) / 2
        angles = pose[2] + np.linspace(-half, half, 41)
        steps = np.arange(0.10, self.cfg.view_range_m, grid.resolution / 2)
        px = pose[0] + np.outer(np.cos(angles), steps)
        py = pose[1] + np.outer(np.sin(angles), steps)
        cols = np.floor((px - grid.origin[0]) / grid.resolution).astype(int)
        rows = np.floor((py - grid.origin[1]) / grid.resolution).astype(int)
        h, w = grid.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        blocked = ~inside
        blocked[inside] = grid.cells[rows[inside], cols[inside]].astype(bool)
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), len(steps))
        ok = np.arange(len(steps))[None, :] < first[:, None]
        self.seen[rows[ok], cols[ok]] = True

    def _reachable(self):
        """Free floor connected to the robot (closed-off rooms cannot be searched, so they do not count)."""
        grid = self.nav.grid
        free = ~grid.cells.astype(bool)
        key = (id(grid), self._reach_cell)
        if self._reach_cache is not None and self._reach_cache[0] == key:
            return self._reach_cache[1]
        pose = self.nav.estimator.pose()
        if pose is None:
            return free
        r, c = grid.world_to_cell(pose[:2])
        h, w = grid.shape
        if not (0 <= r < h and 0 <= c < w) or not free[r, c]:
            return free
        region = np.zeros_like(free)
        region[r, c] = True
        while True:
            grown = region.copy()
            grown[1:, :] |= region[:-1, :]; grown[:-1, :] |= region[1:, :]
            grown[:, 1:] |= region[:, :-1]; grown[:, :-1] |= region[:, 1:]
            grown &= free
            if (grown == region).all():
                break
            region = grown
        self._reach_cell = (r, c)
        self._reach_cache = ((id(grid), (r, c)), region)
        return region

    def _robot_region(self):
        """Cells the robot CENTRE can reach (outside the planning-radius inflation, connected to the robot).
        Used to skip targets A* could never reach (each failed A* search costs ~1 s)."""
        grid, radius = self.nav.grid, self.nav.radius
        pose = self.nav.estimator.pose()
        if pose is None:
            return None
        key = (id(grid), round(radius, 4))
        if self._region_cache is not None and self._region_cache[0] == key and \
                self._region_cache[1][grid.world_to_cell(pose[:2])]:
            return self._region_cache[1]
        open_ = ~inflated_mask(grid, radius)
        r, c = grid.world_to_cell(pose[:2])
        h, w = grid.shape
        seeds = [(r + dr, c + dc) for dr in range(-4, 5) for dc in range(-4, 5)
                 if 0 <= r + dr < h and 0 <= c + dc < w and open_[r + dr, c + dc]]
        if not seeds:
            return None
        region = np.zeros_like(open_)
        for rr, cc in seeds:
            region[rr, cc] = True
        while True:
            grown = region.copy()
            grown[1:, :] |= region[:-1, :]; grown[:-1, :] |= region[1:, :]
            grown[:, 1:] |= region[:, :-1]; grown[:, :-1] |= region[:, 1:]
            grown &= open_
            if (grown == region).all():
                break
            region = grown
        self._region_cache = (key, region)
        return region

    def _reachable_spot(self, region, point):
        if region is None:
            return True
        r, c = self.nav.grid.world_to_cell(point)
        h, w = region.shape
        return 0 <= r < h and 0 <= c < w and bool(region[r, c])

    def coverage(self):
        if self.seen is None:
            return 0.0
        area = self._reachable()
        return float((self.seen & area).sum()) / max(1, int(area.sum()))

    # ------------------------------------------------------------ main loop (navigator thread, under its lock)
    def step(self):
        if not self.active:
            return
        nav = self.nav
        now = self.clock()
        pose = nav.estimator.pose() if nav.estimator.status(now)[0] == 'OK' else None
        self._register_survivors()
        self._update_coverage(pose, now)
        if self.paused:
            return
        if nav.state == RUNNING:
            self._still_since = None
            kind = (self.action or {}).get('kind')
            if self._pending_new and kind in ('transit', 'scan', 'explore') and self._pending_survivor() is not None:
                nav.halt_motion('Survivor spotted: stopping to approach it.')
            return
        if nav._auto_resume_at is not None:
            return                                  # navigator continues by itself (obstacle replan)
        if self.action is not None:
            self._finish_action()
            return
        # Next decision once the robot has stood still for a moment (camera gets a sharp look).
        if not nav.robot_still():
            self._still_since = None
            return
        if self._still_since is None:
            self._still_since = now
            return
        if now - self._still_since < self.cfg.settle_s:
            return
        self._still_since = None
        if not self.camera_active():
            self.pause('Camera + YOLO is off (turn it on in the Camera tab)')
            return
        loc = nav.estimator.status(now)
        if loc[0] != 'OK':
            self.pause('localization %s (%s): re-confirm the robot pose' % loc)
            return
        self._decide()

    def _finish_action(self):
        nav, action = self.nav, self.action
        self.action = None
        if nav.halt_seq == action['seq']:           # never started moving: count as a failure
            self._failed(action, 'motion did not start')
            return
        kind = nav.stop_kind
        if kind in ('operator', 'link', 'localization', 'fault'):
            self.pause(nav.message)
            return
        if kind == 'arrived':
            self.failures = 0
            self._done(action)
        elif kind == 'mission':
            self._pending_new = False               # interrupted on purpose (survivor spotted)
            if action['kind'] == 'transit':
                self.need_scan = True               # goal given up for the survivor: look around afterwards
                self._event('Green object seen on the way: going to it instead of the goal.')
        else:                                       # obstacle / replan / tracking: route did not work out
            self._failed(action, nav.message)

    def _done(self, action):
        kind = action['kind']
        if kind == 'transit':
            self.need_scan = True
            self._event('Reached the goal: starting the survivor search.')
        elif kind == 'scan':
            self.need_scan = False
            self._event('Look-around complete (%.0f %% of the floor searched).' % (100 * self.coverage()))
        elif kind == 'approach':
            self._face = action['target']
        elif kind == 'face':
            s = self._survivor(action['target'])
            if s is not None:
                s.status = 'secured'
                self._event('Survivor S%d SECURED at (%.2f, %.2f) after standing %.0f s in front of it.'
                            % (s.id, s.x, s.y, self.cfg.secure_hold_s))
            self.need_scan = True
        elif kind == 'explore':
            self.need_scan = True
            self.blacklist.add(action['target'])      # visited: do not pick the same viewpoint again this pass

    def _failed(self, action, why):
        self.failures += 1
        kind = action['kind']
        if kind == 'transit':
            self.need_scan = True                   # search from wherever the robot got to
            self._event('Goal not reached (%s): searching from here.' % why)
        elif kind == 'scan':
            self.need_scan = False                  # cannot turn here: look elsewhere
            self._event('Look-around stopped (%s): moving on.' % why)
        elif kind in ('approach', 'face'):
            s = self._survivor(action['target'])
            if s is not None:
                s.attempts += 1
                if kind == 'face' or s.attempts >= 2:
                    s.status = 'secured' if kind == 'face' else 'unreachable'
                    self._event('Survivor S%d marked %s (%s).' % (s.id, s.status, why))
        elif kind == 'explore' and action.get('target') is not None:
            self.blacklist.add(action['target'])
        if self.failures >= self.cfg.max_failures:
            self.pause('%d attempts in a row did not work (last: %s)' % (self.failures, why))

    def _survivor(self, sid):
        return next((s for s in self.survivors if s.id == sid), None)

    # ------------------------------------------------------------ decisions
    def _decide(self):
        self._pending_new = False
        pose = self.nav.estimator.pose()
        if self._face is not None:
            sid, self._face = self._face, None
            s = self._survivor(sid)
            if s is not None:
                heading = math.atan2(s.y - pose[1], s.x - pose[0])
                self._event('In position at survivor S%d: holding %.0f s.' % (sid, self.cfg.secure_hold_s))
                self._spin('face', [heading], max(0.5, self.cfg.secure_hold_s), sid,
                           'Securing survivor S%d (holding %.0f s).' % (sid, self.cfg.secure_hold_s))
                return
        target = self._pending_survivor()
        if target is not None:
            self._approach(target, pose)
        elif self.cfg.end_when_secured and any(s.status == 'secured' for s in self.survivors):
            self._complete()
        elif self.need_scan:
            n = max(1, int(round(360.0 / self.cfg.scan_step_deg)))
            step = math.radians(360.0 / n)
            self._spin('scan', [pose[2] + k * step for k in range(n)], self.cfg.scan_dwell_s, None,
                       'Looking around for survivors (%d views).' % n)
        else:
            self._explore(pose)

    def _spin(self, kind, headings, dwell, target, note):
        seq = self.nav.halt_seq
        ok, msg = self.nav.start_spin(headings, dwell, self.cfg.scan_turn_rate, 'MISSION: ' + note)
        if ok:
            self.action = dict(kind=kind, seq=seq, target=target)
        else:
            self._refused(msg)

    def _refused(self, msg):
        if 'STOP' in msg or 'link' in msg or 'localization' in msg or 'arm' in msg.lower():
            self.pause(msg)
        else:
            self.failures += 1
            self._event('Mission action refused: %s' % msg)
            if self.failures >= self.cfg.max_failures:
                self.pause(msg)

    def _goto(self, point, kind, target, note):
        """Plan with the existing A* and drive. Returns True if the robot started moving."""
        nav = self.nav
        cell = nav.grid.world_to_cell(point)
        goal = nav.grid.cell_to_world(cell)
        nav.set_goal(goal)
        if nav.state != 'PLANNED':
            return False
        nav.auto_replans = 0
        seq = nav.halt_seq
        ok, msg = nav._arm_and_run('MISSION: ' + note)
        if not ok:
            self._refused(msg)
            return None                              # refused (not a routing problem)
        self.action = dict(kind=kind, seq=seq, target=target)
        return True

    def _approach(self, s, pose):
        sc = self.nav.safety.cfg
        ring = self.cfg.approach_gap_m + (sc.half_length_m or 0.11) + self.cfg.object_size_m / 2
        if math.dist(pose[:2], (s.x, s.y)) <= ring + 0.08:
            self._face = s.id                         # already close enough: just face it
            self._decide()
            return
        checker = CollisionChecker(self.nav.grid)
        spots = []
        for k in range(16):
            a = 2 * math.pi * k / 16
            p = (s.x + ring * math.cos(a), s.y + ring * math.sin(a))
            if self.nav.grid.contains(self.nav.grid.world_to_cell(p)) and checker.is_clear(p, p, self.nav.radius):
                roomy = checker.is_clear(p, p, self.nav.radius + self.cfg.spot_clearance_m)
                spots.append((not roomy, math.dist(p, pose[:2]), p))   # roomy spots first, then nearest
        region = self._robot_region()
        spots = [p for _, _, p in sorted(spots) if self._reachable_spot(region, p)]
        for p in spots[:4]:
            started = self._goto(p, 'approach', s.id, 'approaching survivor S%d.' % s.id)
            if started or started is None:
                return
        s.status = 'unreachable'
        self.failures += 1
        self._event('Survivor S%d cannot be reached by a safe route: marked unreachable.' % s.id)

    def _explore(self, pose):
        self._reach_cell = None                     # recompute the reachable area from here
        unseen = self._reachable() & ~self.seen
        if int(unseen.sum()) < self.cfg.min_unseen_cells or not self._try_viewpoints(pose, unseen):
            self.passes += 1
            self.seen[:] = False
            self.blacklist.clear()
            self.need_scan = True
            self._event('Whole reachable area searched: starting search pass %d.' % self.passes)

    def _try_viewpoints(self, pose, unseen):
        grid = self.nav.grid
        res = grid.resolution
        r = max(1, int(round(0.8 * self.cfg.view_range_m / res)))     # count unseen floor around a spot
        integral = np.pad(unseen.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
        h, w = grid.shape
        checker = CollisionChecker(grid)
        x0, x1, y0, y1 = grid.extent
        region = self._robot_region()
        sp = self.cfg.viewpoint_spacing_m
        candidates = []
        for x in np.arange(x0 + sp / 2, x1, sp):
            for y in np.arange(y0 + sp / 2, y1, sp):
                key = (round(float(x), 3), round(float(y), 3))
                if key in self.blacklist or math.dist(key, pose[:2]) < 0.3:
                    continue
                row, col = grid.world_to_cell(key)
                if not (0 <= row < h and 0 <= col < w) or not self._reachable_spot(region, key) or \
                        not checker.is_clear(key, key, self.nav.radius + self.cfg.spot_clearance_m):
                    continue                     # viewpoints need room to turn around and look
                r0, r1, c0, c1 = max(0, row - r), min(h, row + r + 1), max(0, col - r), min(w, col + r + 1)
                count = int(integral[r1, c1] - integral[r0, c1] - integral[r1, c0] + integral[r0, c0])
                if count >= self.cfg.min_unseen_cells:
                    candidates.append((count / (1.0 + 0.7 * math.dist(key, pose[:2])), key))
        candidates.sort(reverse=True)
        for _, key in candidates[:4]:
            started = self._goto(key, 'explore', key, 'exploring towards (%.2f, %.2f).' % key)
            if started or started is None:
                return True
            self.blacklist.add(key)
        return False

    # ------------------------------------------------------------ display
    def snapshot(self, blocking=True):
        if not self.nav._lock.acquire(blocking=blocking):
            if self._last_snapshot is not None:
                return self._last_snapshot
            self.nav._lock.acquire()
        try:
            self._last_snapshot = self._snapshot()
            return self._last_snapshot
        finally:
            self.nav._lock.release()

    def _snapshot(self):
        if True:
            kind = (self.action or {}).get('kind')
            if not self.active:
                phase = 'COMPLETE' if self.complete else 'OFF'
            elif self.paused:
                phase = 'PAUSED'
            elif kind == 'approach' or kind == 'face':
                phase = '%s S%s' % (PHASE_TEXT[kind], self.action['target'])
            else:
                phase = PHASE_TEXT.get(kind, 'Deciding next step')
            return dict(active=self.active, paused=self.paused, complete=self.complete, pause_reason=self.pause_reason, phase=phase,
                        survivors=[s.as_dict() for s in self.survivors], events=list(self.events[-60:]),
                        elapsed=self.elapsed() if self.active else self._final_elapsed,
                        passes=self.passes, coverage=self.coverage(),
                        seen=None if self.seen is None else self.seen.copy())
