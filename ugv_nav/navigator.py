"""Edge navigation orchestrator: map + pose + goal -> existing A* -> path tracker -> UGV.

States::

    IDLE ──set_goal──▶ PLANNED ──authorize()──▶ RUNNING ──▶ ARRIVED
                          ▲                        │
                          └── replan (needs new ───┤ any fault / STOP
                              authorization)       ▼
                                                STOPPED

Movement only ever starts from an operator ``authorize()``. Every fault,
STOP, reconnect, map change or replan leaves the robot stopped and disarmed
until the operator authorizes again -- with ONE optional exception:
``auto_continue_after_replan``. When an unmapped obstacle blocks the route and
the edge finds a new A* route, the navigator may re-arm and drive it by itself
after the robot has stood still for ``auto_continue_delay_s``, at most
``max_auto_replans`` times per operator authorization, and only if all safety
checks pass. Operator STOP, link loss, localization loss and every other fault
never auto-continue.

Planning uses ``fast_planner.RoutePlanner``: the same A* search as the unmodified ``ugv_planner``
(identical routes, verified by tests) with caching, followed by optional objective-preserving
smoothing (``smoothing.smooth_route``). ``planner_engine = 'original'`` calls ``ugv_planner`` directly.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field, replace

import numpy as np

from ugv_planner.config import PlannerConfig
from ugv_planner.grid import Grid

from .controller import Command, ControllerConfig, PathTracker
from .fast_planner import RoutePlanner
from .localization import LocalizationConfig, PoseEstimator, wrap
from .map_update import mark_scan_obstacles
from .path_processing import path_length, remove_collinear, shortcut, validate_path
from .room_map import RoomMap, RoomTracker
from .safety_monitor import SafetyConfig, SafetyMonitor, remaining_path_clear
from ugv_bridge.safety import stopping_distance
from .scan_match import ScanMatcher
from .smoothing import smooth_route

IDLE, PLANNED, RUNNING, ARRIVED, STOPPED = 'IDLE', 'PLANNED', 'RUNNING', 'ARRIVED', 'STOPPED'

# When may the robot be placed by the LiDAR search WITHOUT asking the operator? Only with a strong fit on
# surroundings that were mostly seen before and no second place that fits nearly as well. Measured in the
# simulated room: with these limits no wrong place was accepted (220 of 220 right in a fully drawn room; in
# a half-drawn room the doubtful ones are shown to the operator instead).
SURE = dict(min_score=0.80, min_known=0.60, margin=0.08)


@dataclass
class NavigatorConfig:
    control_rate_hz: float = 10.0
    command_ttl_s: float = 0.3
    use_shortcut: bool = False
    smooth_path: bool = True              # straighten the grid zig-zag (never worse for the planner's objective)
    lidar_correction: bool = False        # keep correcting the pose from the LiDAR (see lidar_reference)
    lidar_correction_period_s: float = 1.0
    lidar_reference: str = 'map'          # what the LiDAR can see: 'map' = the arena walls of the operator map,
                                          # 'room' = only things taller than the arena (ugv_nav.room_map)
    auto_find: bool = True                # room: find the robot by itself when no pose is set / it was carried
    planner_engine: str = 'fast'          # 'fast' = same A* results, cached + ~50x quicker; 'original' = ugv_planner
    replan_on_obstacle: bool = True       # update edge map + replan on the edge when an unmapped obstacle blocks
    auto_continue_after_replan: bool = False  # drive the new route without waiting for the operator
    max_auto_replans: int = 5             # per operator authorization (prevents endless stop/replan loops)
    auto_continue_delay_s: float = 1.0    # robot must stand still this long before continuing
    allow_tight_radius: bool = False      # operator accepts a planning radius below the turning circle
    obstacle_map_range_m: float = 1.0
    max_log: int = 200
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)


class Navigator:
    def __init__(self, client, grid, planner_config: PlannerConfig, config: NavigatorConfig = NavigatorConfig(),
                 clock=time.monotonic):
        self.client = client
        # Map layers: base (operator map) | LiDAR-detected cells | camera (YOLO) obstacles.
        # self.grid is the composed grid A* plans on; self.display_grid = base | LiDAR (what the GUI draws).
        self.grid = grid
        self.base_grid = grid
        self.display_grid = grid
        self.lidar_mask = np.zeros(grid.shape, bool)
        self.vision_mask = np.zeros(grid.shape, bool)
        self.vision_obstacles = []
        self.map_rev = 0                      # increments whenever any map layer changes (displays re-fetch)
        self._no_route = False                # last A* attempt found no route to the goal
        self._blocked_while_running = False   # ... and that happened while the robot was driving
        self.planner_config = planner_config
        self.cfg = config
        self.clock = clock
        self.estimator = PoseEstimator(config.localization)
        self.safety = SafetyMonitor(config.safety)
        self.state = IDLE
        self.message = 'Connect, set the initial pose, then choose a goal.'
        self.goal = None
        self.plan_result = None
        self.path = []
        self.tracker = None
        self.trajectory = []
        self.last_command = None
        self.last_reasons = []
        self.log = []
        self.log_count = 0
        self._lock = threading.RLock()
        self._last_tick = None
        self._armed_at_server = 0.0
        self.base_controller = config.controller   # 100 % speed = these max_v / max_w
        self.speed_fraction = 1.0
        self.auto_replans = 0             # automatic continuations since the last operator authorization
        self._auto_resume_at = None       # pending automatic continuation time (None = none pending)
        self._last_telemetry_time = None
        self._thread = None
        self._stop_thread = threading.Event()
        self.motion = None                # 'path' | 'spin' while RUNNING
        self.stop_kind = None             # why the robot last stopped (see _halt)
        self.halt_seq = 0                 # increments on every stop (lets the mission see each event once)
        self.mission = None               # optional SearchMission (ugv_nav.mission)
        self._last_snapshot = None
        self._motion_ref = None           # (odom distance, odom rotation, LiDAR view) for the stall check
        self.planner = RoutePlanner(config.planner_engine)
        self.plan_info = {}               # last plan: timing, raw/smoothed length, waypoint counts
        self._path_checked_grid = None    # map object the remaining route was last validated against
        self._shorter_worker = None       # background "is there a shorter route now?" check
        self._fast = None                 # last cheap display sample (see fast_state)
        self._scan_matcher = None         # (base grid, ScanMatcher)
        self._next_lidar_fix = 0.0
        self.lidar_fix = None             # last automatic LiDAR correction attempt (diagnostics)
        self._match_quality = None
        self._loop = dict(last=None, periods=[], durations=[])   # control-loop timing (diagnostics)
        self.room = None                  # RoomTracker (lidar_reference == 'room')
        self.lidar_mount_ok = False       # the LiDAR mounting was verified by the robot's own motion
        self.found = None                 # last "find robot" result that needs the operator: dict(reason, candidates)
        self.found_seq = 0                # counts poses adopted from the LiDAR search (displays follow them)
        self._search = None               # background search for the robot: dict(thread, result, gen)
        self._next_search = 0.0
        self._search_misses = 0           # automatic searches in a row that found nothing (they get rarer)
        self._pose_gen = 0                # changes whenever the pose is set / the room drawing is replaced
        self._spots = None                # (base grid, places the robot can stand)

    # ------------------------------------------------------------ helpers
    def _note(self, text):
        self.message = text
        self.log.append((round(self.clock(), 2), text))
        self.log_count += 1               # total notes ever written (displays fetch only the new ones)
        del self.log[:-self.cfg.max_log]

    def _halt(self, state, reason, kind='fault'):
        """Stop the robot (best effort; the UGV watchdog stops it anyway) and change state.

        ``kind`` tells the search mission why: arrived | obstacle | replan | tracking | mission
        (recoverable by the mission) or operator | link | localization | fault (mission pauses).
        """
        try:
            self.client.stop()
        except Exception:
            pass
        self.state = state
        self.tracker = None
        self.motion = None
        self.stop_kind = kind
        self.halt_seq += 1
        self._note(reason)

    def halt_motion(self, reason, kind='mission'):
        """Stop the current motion without latching STOP (used by the search mission)."""
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                self._halt(STOPPED, reason, kind)

    def auto_enabled(self):
        m = self.mission
        return self.cfg.auto_continue_after_replan or (m is not None and m.active and not m.paused)

    def robot_still(self, snap=None):
        """True when odometry and the UGV's motor output show the robot standing still."""
        snap = snap or self.client.snapshot()
        tel = snap['telemetry'] or {}
        odom = tel.get('odom') or {}
        return not (abs(odom.get('vx', 0.0)) > 0.02 or abs(odom.get('vy', 0.0)) > 0.02
                    or abs(odom.get('wz', 0.0)) > 0.05 or any(tel.get('output', [0.0, 0.0, 0.0])))

    def _ingest(self):
        snap = self.client.snapshot()
        tel = snap['telemetry']
        if tel is not None and snap['telemetry_age'] is not None:
            received = self.clock() - snap['telemetry_age']
            if received != self._last_telemetry_time:
                self._last_telemetry_time = received
                self.estimator.update(tel, received)
        return snap

    def _cancel_auto(self):
        self._auto_resume_at = None

    @property
    def radius(self):
        return self.planner_config.planning_radius

    # ------------------------------------------------------------ operator actions
    def set_map(self, grid, keep_lidar=False):
        """New operator map. ``keep_lidar`` keeps the LiDAR-detected cells as their own layer (map edits);
        without it they are dropped (new environment)."""
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                self._halt(STOPPED, 'Map changed while running: stopped.', 'operator')
            self.base_grid = grid
            if keep_lidar and self.lidar_mask.shape == grid.shape:
                self.lidar_mask = self.lidar_mask & ~grid.cells.astype(bool)
            else:
                self.lidar_mask = np.zeros(grid.shape, bool)
            if self.vision_mask.shape != grid.shape:
                self.vision_mask = np.zeros(grid.shape, bool); self.vision_obstacles = []
            if self.room is not None and tuple(self.room.extent) != tuple(grid.extent):
                self.room = None                    # another arena size: the room drawing no longer fits
                self._pose_gen += 1
            self._compose()
            self.path, self.plan_result = [], None
            if self.state == PLANNED:
                self.state = IDLE

    # ------------------------------------------------------------ map layers
    def _compose(self):
        b = self.base_grid
        self.display_grid = (Grid((b.cells | self.lidar_mask).astype(np.uint8), b.resolution, b.origin)
                             if self.lidar_mask.any() else b)
        extra = self.lidar_mask | self.vision_mask
        if not extra.any():
            self.grid = self.base_grid
        else:
            self.grid = Grid((b.cells | extra).astype(np.uint8), b.resolution, b.origin)
        self.map_rev += 1

    def clear_lidar_obstacles(self):
        """Forget the obstacles the LiDAR added to the edge map (operator action). Replans if a goal is set."""
        with self._lock:
            if self.state == RUNNING:
                raise ValueError('Stop the robot first.')
            removed = int(self.lidar_mask.sum())
            self.lidar_mask = np.zeros(self.base_grid.shape, bool)
            self._compose()
            if removed and self.goal is not None and self.state in (PLANNED, IDLE, STOPPED, ARRIVED) \
                    and self.estimator.pose() is not None:
                self._cancel_auto()
                self._plan()
            return removed

    def _block_mask(self, obstacles):
        b = self.base_grid
        mask = np.zeros(b.shape, bool)
        res = b.resolution
        h, w = b.shape
        for o in obstacles:
            half = o.get('size', 0.05) / 2.0
            c0 = int(np.floor((o['x'] - half - b.origin[0]) / res)); c1 = int(np.floor((o['x'] + half - b.origin[0]) / res))
            r0 = int(np.floor((o['y'] - half - b.origin[1]) / res)); r1 = int(np.floor((o['y'] + half - b.origin[1]) / res))
            r0, r1, c0, c1 = max(r0, 0), min(r1, h - 1), max(c0, 0), min(c1, w - 1)
            if r0 <= r1 and c0 <= c1:
                mask[r0:r1 + 1, c0:c1 + 1] = True
        return mask

    def set_vision_obstacles(self, obstacles, reason='camera'):
        """Replace the camera (YOLO) obstacle layer: [dict(x, y, size, cls, id)] in map metres.

        New obstacle on the route while driving -> stop and replan (auto-continue if enabled).
        Obstacle removed while driving -> replan if the new route is clearly shorter.
        """
        with self._lock:
            new_mask = self._block_mask(obstacles)
            old_ids = {o.get('id') for o in self.vision_obstacles}
            gone = old_ids - {o.get('id') for o in obstacles}     # blocks that really disappeared
            self.vision_obstacles = list(obstacles)
            if np.array_equal(new_mask, self.vision_mask):
                return
            added = int((new_mask & ~self.vision_mask).sum())
            removed = int((self.vision_mask & ~new_mask).sum())
            self.vision_mask = new_mask
            self._compose()
            if self.state == RUNNING and self.tracker is not None and self.motion == 'path':
                if not remaining_path_clear(self.grid, self.path, self.tracker.segment, self.radius):
                    self._halt(STOPPED, 'Stopped: %s obstacle on the route.' % reason, 'obstacle')
                    self._after_replan(max(added, 1), 'Camera obstacle added to edge map')
                elif gone and self.goal is not None:      # a block was removed (not just re-estimated)
                    self._replan_if_shorter(reason)
            elif self.state == PLANNED and self.goal is not None:
                self._plan()
                self._note('Map updated by %s (%d cells added, %d removed); route replanned.' % (reason, added, removed))
            elif self.state in (IDLE, STOPPED) and self.goal is not None and gone and self._no_route:
                # The route was blocked; a block disappeared: try again. Drive on by itself only if the
                # robot was driving when it got blocked (auto-continue rules apply); else wait for AUTHORIZE.
                if self._blocked_while_running and not self.safety.estop:
                    self._after_replan(removed, 'Obstacle removed from edge map; route found again')
                else:
                    result = self._plan()
                    if result is not None and result.found:
                        self._note('Obstacle removed: route to the goal found again. Authorize to drive.')

    def _replan_if_shorter(self, reason):
        """An obstacle disappeared while driving: is there a clearly shorter route now?

        The A* search runs in a helper thread WITHOUT the navigator lock, so the control loop keeps
        sending commands at its normal rate while the laptop plans."""
        pose = self.estimator.pose()
        if pose is None or (self._shorter_worker is not None and self._shorter_worker.is_alive()):
            return
        snap = (self.grid, pose, self.goal, self.planner_config, self.path, self.tracker)
        self._shorter_worker = threading.Thread(target=self._shorter_route_check, args=(snap, reason),
                                                name='shorter-route', daemon=True)
        self._shorter_worker.start()

    def _shorter_route_check(self, snap, reason):
        grid, pose, goal, config, path, tracker = snap
        try:
            result = self.planner.replan(grid, pose[:2], goal, config)
        except Exception:
            return
        if not result.found:
            return
        candidate = remove_collinear(result.waypoints)
        if self.cfg.smooth_path:
            candidate, _ = smooth_route(candidate, grid, config, self.planner.cache)
        with self._lock:
            if (self.state != RUNNING or self.path is not path or self.tracker is not tracker
                    or self.grid is not grid or self.motion != 'path'):
                return                                   # the situation changed while planning
            now_pose = self.estimator.pose()
            if now_pose is None:
                return
            k = tracker.segment
            remaining = (math.dist(now_pose[:2], path[min(k + 1, len(path) - 1)]) + path_length(path[k + 1:]))
            if path_length(candidate) < remaining - 0.20:
                self._halt(STOPPED, 'Stopped: obstacle removed (%s); a shorter route is available.' % reason, 'replan')
                self._after_replan(1, 'Obstacle removed from edge map')

    def set_planner_config(self, config: PlannerConfig):
        with self._lock:
            if self.state == RUNNING:
                self._halt(STOPPED, 'Planner settings changed while running: stopped.', 'operator')
            self.planner_config = config

    def set_speed_fraction(self, fraction):
        """Robot speed as a fraction (0.1-1.0) of the configured max_v / max_w. Safe to change while driving."""
        fraction = float(fraction)
        if not math.isfinite(fraction) or not 0.1 <= fraction <= 1.0:
            raise ValueError('Speed must be between 10 % and 100 %.')
        base = self.base_controller
        max_v, max_w = base.max_v * fraction, base.max_w * fraction
        scaled = replace(base, max_v=max_v, max_w=max_w, min_v=min(base.min_v, max_v),
                         min_w=min(base.min_w, max_w))   # keep the motor dead-band minimums where possible
        with self._lock:
            self.speed_fraction = fraction
            self.cfg.controller = scaled
            if self.tracker is not None:
                self.tracker.cfg = scaled
        return max_v, max_w

    def set_scale(self, scale_x, scale_y):
        """Apply X/Y map-scale corrections (not while driving)."""
        with self._lock:
            if self.state == RUNNING:
                raise ValueError('Stop the robot before changing the scale calibration.')
            self.estimator.set_scale(scale_x, scale_y)
            self._note('Scale calibration applied: X %.3f, Y %.3f.' % (scale_x, scale_y))

    def set_initial_pose(self, map_pose, source='manual'):
        """Calibration: the robot is physically at ``map_pose`` (x, y, yaw) right now. ``source``: 'manual'
        (pin / typed), 'dock' (a corner start: a physical reference, so the room drawing made from it is
        worth keeping)."""
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                raise ValueError('Stop the robot before recalibrating.')
            self._ingest()
            x, y, _ = map_pose
            cell = self.grid.world_to_cell((x, y))
            if not self.grid.contains(cell):
                raise ValueError('Initial pose is outside the map.')
            ok, msg = validate_path([(x, y)], self.grid, self.radius)
            if not ok:
                raise ValueError('Initial pose is not clear of occupancy for the configured footprint: ' + msg)
            self.estimator.calibrate(map_pose)
            self.trajectory = []
            self.path, self.plan_result = [], None
            self.found = None
            self._pose_gen += 1
            self._search_misses = 0
            if self.state in (PLANNED, ARRIVED):
                self.state = IDLE
            if self.cfg.lidar_reference == 'room':
                self._room().restart(tuple(map_pose), trusted=(source == 'dock'))
                self._room_step(self.client.snapshot(), self.clock())     # start drawing the room right away
            self._note('Initial pose set to (%.3f, %.3f, %.1f°).' % (x, y, math.degrees(map_pose[2])))

    def forget_pose(self, why='The map changed'):
        """The confirmed pose is no longer valid (e.g. the map was rescaled): the operator must set it again."""
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                self._halt(STOPPED, '%s while running: stopped.' % why, 'operator')
            est = self.estimator
            est.map_T_odom = est.anchor = est.imu_offset = None
            est.distance = est.rotation = 0.0
            self._pose_gen += 1
            self.trajectory, self.path, self.plan_result, self.goal, self.found = [], [], None, None, None
            if self.state in (PLANNED, ARRIVED):
                self.state = IDLE
            if self.room is not None:
                self.room.pose = None
            self._note('%s: place the robot and confirm its pose again.' % why)

    def set_goal(self, goal_xy):
        """Plan with the existing A* from the current localized pose. Does not move the robot."""
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                self._halt(STOPPED, 'New goal while running: stopped before replanning.', 'operator')
            self.goal = tuple(goal_xy)
            self._blocked_while_running = False
            return self._plan()

    def _plan(self):
        self._ingest()
        status = self.estimator.status(self.clock())
        pose = self.estimator.pose()
        if pose is None or status[0] not in ('OK',):
            self.state = IDLE
            self._note('Cannot plan: localization %s (%s).' % status)
            return None
        result = self.planner.replan(self.grid, pose[:2], self.goal, self.planner_config)
        self.plan_result = result
        self.plan_info = dict(found=result.found, astar_ms=round(result.computation_ms, 1),
                              expanded=result.expanded_nodes, status=result.status)
        self._path_checked_grid = None
        if not result.found:
            self.path = []
            self.state = IDLE
            self._no_route = True
            self._note('A* %s: %s' % (result.status, result.message))
            return result
        self._no_route = False
        path = remove_collinear(result.waypoints)
        raw_length, raw_points = path_length(path), len(path)
        smoothed = False
        if self.cfg.smooth_path:
            path, info = smooth_route(path, self.grid, self.planner_config, self.planner.cache)
            smoothed = info['smoothed']
        elif self.cfg.use_shortcut:
            path = shortcut(path, self.grid, self.radius)
        ok, msg = validate_path(path, self.grid, self.radius)
        if not ok:
            self.path = []
            self.state = IDLE
            self._note('Planned path rejected: ' + msg)
            return result
        self.path = path
        self.state = PLANNED
        self.plan_info.update(length_m=round(path_length(path), 3), points=len(path), smoothed=smoothed,
                              raw_length_m=round(raw_length, 3), raw_points=raw_points,
                              total_ms=round(self.planner.last_ms, 1))
        self._note('Path planned: %.2f m, %d waypoints%s. Awaiting operator authorization.' % (
            path_length(path), len(path), ' (smoothed from %d)' % raw_points if smoothed else ''))
        return result

    def authorize(self):
        """Operator authorization: the ONLY way motion begins. Returns (ok, message)."""
        with self._lock:
            self._cancel_auto()
            ok, msg = self._arm_and_run('Navigation AUTHORIZED by operator.', operator=True)
            if ok:
                self.auto_replans = 0  # a fresh operator authorization starts a new budget
                self._blocked_while_running = False
            return ok, msg

    def _arm_and_run(self, note, operator=False):
        with self._lock:
            if self.state != PLANNED or not self.path:
                return False, 'No valid planned path to execute.'
            return self._arm(note, 'path', operator=operator)

    def start_spin(self, headings, dwell_s=1.0, max_rate=None, note='Turning on the spot.'):
        """Turn in place through absolute map headings, pausing ``dwell_s`` at each (camera look).

        Only called by the search mission, which itself runs only after an operator authorization.
        """
        with self._lock:
            self._cancel_auto()
            if self.state == RUNNING:
                return False, 'Robot is already moving.'
            self._ingest()
            pose = self.estimator.pose()
            if pose is None:
                return False, 'No localized pose.'
            self.path = [tuple(pose[:2])]
            return self._arm(note, 'spin', SpinTracker(headings, dwell_s, self.cfg.controller, max_rate))

    def _arm(self, note, motion, spin_tracker=None, operator=False):
        """Arm the UGV and start ``motion``. Only an operator authorization may clear a latched STOP."""
        with self._lock:
            if self.safety.estop and not operator:
                return False, 'Operator STOP is latched: only the operator can continue.'
            if self.planner_config.footprint.radius_m is None:
                return False, 'Robot radius is unknown: enter the measured radius before physical navigation.'
            sc = self.safety.cfg
            if self.safety.dimensions_known():
                turn = math.hypot(sc.half_length_m, sc.half_width_m) + sc.rotate_clearance_m
                if self.radius + 1e-9 < turn:
                    if not self.cfg.allow_tight_radius:
                        return False, ('Planning radius %.3f m is smaller than the turning envelope %.3f m '
                                       '(hypot(half_length, half_width) + rotate_clearance): the robot could be routed '
                                       'where it may not turn. Increase radius/margin, or tick "Allow radius below '
                                       'turning circle" to accept this.' % (self.radius, turn))
                    note = ('%s [tight radius %.3f m < turning circle %.3f m accepted by operator; the UGV refuses '
                            'turns with a wall inside its turning circle]' % (note, self.radius, turn))
            self.safety.estop = False
            snap = self._ingest()
            now = self.clock()
            pose = self.estimator.pose()
            reasons = self.safety.check(snap['link_ok'], snap['telemetry'], self.estimator.status(now))
            if reasons:
                return False, 'Refused: ' + '; '.join(reasons)
            if motion == 'path' and math.dist(pose[:2], self.path[0]) > self.cfg.controller.max_cross_track_m:
                return False, 'Robot moved since planning; replan first.'
            ok, why = self.client.arm()
            if not ok:
                return False, 'UGV refused to arm: ' + why
            self._armed_at_server = self.client.server_now() or 0.0
            self.tracker = spin_tracker or PathTracker(self.path, self.cfg.controller)
            self._motion_ref = None
            self.motion = motion
            self.state = RUNNING
            self._last_tick = None
            self._note(note)
            return True, 'Running.'

    def emergency_stop(self, reason='Operator STOP'):
        # Act at once, even if the control thread is busy planning: latch STOP and tell the UGV first.
        self.safety.estop = True
        try:
            self.client.stop()
        except Exception:
            pass
        with self._lock:
            self._cancel_auto()
            self._blocked_while_running = False
            self.safety.estop = True
            self._halt(STOPPED, reason + ' (latched; re-authorize to continue).', 'operator')
            if self.mission is not None:
                self.mission.pause(reason)

    # ------------------------------------------------------------ control loop
    def tick(self):
        """One control cycle. Safe to call at any time; only acts when RUNNING."""
        with self._lock:
            snap = self._ingest()
            now = self.clock()
            pose = self.estimator.pose()
            if pose is not None and (not self.trajectory or math.dist(self.trajectory[-1][:2], pose[:2]) > 0.005):
                self.trajectory.append(pose)
                del self.trajectory[:-20000]
            self._lidar_correct(snap, now)
            if self.state == PLANNED and self._auto_resume_at is not None:
                self._try_auto_continue(now, snap)
                return None
            if self.state != RUNNING:
                return None
            dt = 0.0 if self._last_tick is None else now - self._last_tick
            self._last_tick = now
            loc = self.estimator.status(now)
            tel = snap['telemetry']
            # Connection loss / reconnect -> UGV is disarmed; never resume automatically.
            disarmed = (tel is not None and not tel.get('armed')
                        and tel.get('server_time', 0.0) > self._armed_at_server + 0.05)
            if not snap['link_ok'] or disarmed:
                self._halt(STOPPED, 'Stopped: UGV link lost or UGV disarmed (%s). Re-authorize after checking.'
                           % (snap['error'] or 'disarmed'), 'link')
                return None
            if loc[0] != 'OK':
                self._halt(STOPPED, 'Stopped: localization %s (%s).' % loc, 'localization')
                return None
            stalled = self._not_really_moving(tel)
            if stalled:
                self._halt(STOPPED, 'Stopped: ' + stalled, 'fault')
                return None
            spinning = self.motion == 'spin'
            cmd = self.tracker.update(pose, now, dt)
            if cmd.status == 'arrived':
                self._halt(ARRIVED, ('Turn complete (%s).' if spinning else 'Goal reached (%s).') % cmd.message, 'arrived')
                return cmd
            if cmd.status == 'failed':
                self._halt(STOPPED, 'Stopped: %s failed (%s).' % ('turn' if spinning else 'path following', cmd.message),
                           'tracking')
                return cmd
            if not spinning and self.grid is not self._path_checked_grid:
                # Only when the map object changed: the route was validated on the map it was planned on.
                if not remaining_path_clear(self.grid, self.path, self.tracker.segment, self.radius):
                    self._halt(STOPPED, 'Stopped: remaining route is no longer valid on the current map.', 'obstacle')
                    return cmd
                self._path_checked_grid = self.grid
            reasons = self.safety.check(True, tel, loc, (cmd.vx, cmd.vy, cmd.wz), pose, self.grid, self.estimator.sigma)
            self.last_reasons = reasons
            if reasons:
                obstacle = any(r.startswith('LiDAR:') or 'obstacle' in r for r in reasons)
                self._halt(STOPPED, 'Stopped: ' + '; '.join(reasons), 'obstacle' if obstacle else 'fault')
                if obstacle and self.cfg.replan_on_obstacle and not spinning:
                    self._replan_around_obstacle(pose, tel)
                return cmd
            if not self.client.send_velocity(cmd.vx, cmd.vy, cmd.wz, self.cfg.command_ttl_s):
                self._halt(STOPPED, 'Stopped: command could not be sent.', 'link')
                return cmd
            self.last_command = cmd
            return cmd

    # ------------------------------------------------------------ LiDAR <-> map
    def _matcher(self):
        if self._scan_matcher is None or self._scan_matcher[0] is not self.base_grid:
            self._scan_matcher = (self.base_grid, ScanMatcher(self.base_grid, self.planner.cache))
        return self._scan_matcher[1]

    def _room(self):
        if self.room is None:
            self.room = RoomTracker(self.base_grid.extent, mount=self.safety.lidar_mount,
                                    calibrated=self.lidar_mount_ok)
        return self.room

    def set_lidar_mount(self, mount, verified):
        """Extra LiDAR mounting (x, y, yaw) on top of the robot's own setting, e.g. remembered from last time."""
        with self._lock:
            self.safety.lidar_mount = tuple(float(v) for v in mount)
            self.lidar_mount_ok = bool(verified)
            self.room = None
            self._pose_gen += 1

    def _reference(self):
        """(matcher, room tracker or None) for what the LiDAR can see; matcher None = nothing to compare with."""
        if self.cfg.lidar_reference == 'room':
            room = self._room()
            return (None if room.map.empty else room.map.matcher()), room
        return self._matcher(), None

    def map_match(self):
        """How well the LiDAR scan fits the reference (arena map or room drawing) at the estimated pose:
        dict(matched 0..1, score, points) or None. Display only; never waits for the control thread."""
        if not self._lock.acquire(blocking=False):
            return self._match_quality
        try:
            pose = self.estimator.pose()
            tel = self.client.snapshot()['telemetry'] or {}
            matcher, _ = self._reference()
            if pose is None or not tel.get('scan') or matcher is None:
                self._match_quality = None
            else:
                self._match_quality = matcher.evaluate(pose, self.safety.lidar_points(tel))
            return self._match_quality
        finally:
            self._lock.release()

    def relocalize_with_lidar(self, window_xy=0.30, window_yaw=math.radians(20.0)):
        """Operator action: correct the pose by fitting the LiDAR scan to the reference. Returns (ok, message).
        Only when the robot stands still; refused when the fit is poor or ambiguous."""
        with self._lock:
            if self.state == RUNNING:
                return False, 'Stop the robot first.'
            snap = self._ingest()
            now = self.clock()
            status = self.estimator.status(now)
            pose = self.estimator.pose()
            if pose is None or status[0] not in ('OK', 'LOW_CONFIDENCE'):
                return False, 'Confirm the robot pose by hand first (localization %s: %s).' % status
            tel = snap['telemetry'] or {}
            scan = tel.get('scan')
            if not scan or scan.get('age', 9.0) > self.safety.cfg.scan_stale_s:
                return False, 'No fresh LiDAR scan.'
            if not self.robot_still(snap):
                return False, 'Wait until the robot stands still.'
            matcher, room = self._reference()
            if matcher is None:
                return False, 'The LiDAR has not drawn the room yet: stand still for a moment after Confirm pose.'
            result = matcher.match(pose, self.safety.lidar_points(tel), window_xy, window_yaw, precise=True)
            if not result.ok:
                return False, 'LiDAR re-localization not possible: %s.' % result.reason
            self._cancel_auto()
            self.estimator.correct(result.pose)
            if room is not None:
                room.adopt(result.pose)
            self._note('Pose corrected with LiDAR: moved %.1f cm, turned %.1f deg; %.0f %% of the scan now lies on '
                       '%s.' % (result.shift_m * 100, math.degrees(result.turn_rad), result.matched * 100,
                                'the room outline' if room is not None else 'mapped walls'))
            message = self.message
            if self.goal is not None and self.state in (PLANNED, IDLE, STOPPED) and not self.safety.estop:
                self._plan()
            return True, message

    def _lidar_correct(self, snap, now):
        """Keep the pose in step with what the LiDAR sees (arena walls, or the room when it looks over them)."""
        if self.cfg.lidar_reference == 'room':
            return self._room_step(snap, now)
        if not self.cfg.lidar_correction or now < self._next_lidar_fix:
            return
        self._next_lidar_fix = now + self.cfg.lidar_correction_period_s
        est = self.estimator
        tel = snap['telemetry'] or {}
        scan, odom = tel.get('scan'), tel.get('odom') or {}
        if est.map_T_odom is None or est.fault or not scan or scan.get('age', 9.0) > 0.3 \
                or abs(odom.get('wz', 0.0)) > 0.3:
            return
        pose = est.pose()
        result = self._matcher().match(pose, self.safety.lidar_points(tel), window_xy=0.09,
                                       window_yaw=math.radians(6.0))
        self.lidar_fix = dict(ok=result.ok, reason=result.reason, matched=round(result.matched, 2),
                              shift_cm=round(result.shift_m * 100, 1), turn_deg=round(math.degrees(result.turn_rad), 1))
        if not result.ok:
            return
        target = result.pose
        if result.shift_m > 0.04 or result.turn_rad > math.radians(3.0):       # large step: go half way (smooth)
            target = (pose[0] + 0.5 * (target[0] - pose[0]), pose[1] + 0.5 * (target[1] - pose[1]),
                      wrap(pose[2] + 0.5 * wrap(target[2] - pose[2])))
        est.correct(target)

    # ------------------------------------------------------------ LiDAR looks over the arena: the room
    def _room_step(self, snap, now):
        self._collect_search()
        if not self.cfg.lidar_correction:
            return
        room, est = self._room(), self.estimator
        tel = snap['telemetry'] or {}
        scan, odom = tel.get('scan'), tel.get('odom') or {}
        if est.fault or est.odom is None or not scan:
            return
        still = self.robot_still(snap)
        if est.map_T_odom is None:              # nobody said where the robot is: look for it in a known room
            if self.cfg.auto_find and not room.map.empty and still and self.state != RUNNING:
                self._start_search(self.safety.lidar_points(tel), now)
            return
        age = max(0.0, min(0.5, float(scan.get('age', 0.0)) - float(odom.get('age', 0.0))))
        step = room.step(now, est.pose(), self.safety.lidar_points(tel), est.odom, float(odom.get('vx', 0.0)),
                         float(odom.get('wz', 0.0)), age, still)
        for note in step.notes:
            self._note(note)
        if step.mount is not None:
            self.safety.lidar_mount = tuple(step.mount)
        self.lidar_mount_ok = room.calibrated
        if step.correct is not None:
            try:
                est.correct(step.correct)
            except ValueError:
                pass
        room.estimator_is(est.pose())
        if step.search and self.cfg.auto_find and self.state != RUNNING:
            self._start_search(self.safety.lidar_points(tel), now)

    def _standing_spots(self):
        """Places in the arena where the robot's centre can be at all, on a 6 cm lattice (for searching)."""
        if self._spots is None or self._spots[0] is not self.base_grid:
            grid = self.base_grid
            sc = self.safety.cfg
            need = max(0.03, min(sc.half_length_m or 0.11, sc.half_width_m or 0.09) - 0.03)
            clear = self.planner.cache.distance(grid, 0.30)
            rows, cols = np.nonzero(clear[::2, ::2] >= need)
            spots = np.column_stack((grid.origin[0] + (2 * cols + 0.5) * grid.resolution,
                                     grid.origin[1] + (2 * rows + 0.5) * grid.resolution))

            def possible(x, y, clear=clear, grid=grid, need=need - 0.02):
                r, c = grid.world_to_cell((x, y))
                return grid.contains((r, c)) and clear[r, c] >= need
            self._spots = (grid, spots, possible)
        return self._spots[1], self._spots[2]

    def _start_search(self, points, now):
        """Search the whole arena for the robot in the background (the control loop keeps running)."""
        if self._search is not None or now < self._next_search:
            return
        matcher, _ = self._reference()
        if matcher is None:
            return
        spots, possible = self._standing_spots()
        job = dict(result=None, done=False, gen=self._pose_gen)

        def work():
            try:
                job['result'] = matcher.locate(points, spots, possible=possible, **SURE)
            finally:
                job['done'] = True
        job['thread'] = threading.Thread(target=work, name='find-robot', daemon=True)
        self._search = job
        self._next_search = now + min(4.0 * 2 ** self._search_misses, 60.0)     # fruitless searches get rarer
        job['thread'].start()

    def _collect_search(self):
        job = self._search
        if job is None or not job['done']:
            return
        self._search = None
        result = job['result']
        if job['gen'] != self._pose_gen:                     # the operator set the pose meanwhile: his word counts
            return
        if result is None or not result.ok:
            self._search_misses = min(self._search_misses + 1, 6)
        if result is not None and not result.ok and self.room is not None and self.room.state == 'lost':
            self.room.reason = result.reason                 # why the robot is not found (shown to the operator)
        if result is None or not result.ok or self.state == RUNNING or not self.robot_still():
            return
        had_pose = self.estimator.map_T_odom is not None
        self._apply_found(result.pose, 'Robot found by itself' if not had_pose else 'The robot was moved: found again',
                          result)

    def adjust_pose(self, map_pose):
        """Operator action: the robot stands still, but the estimate should be a little different (nudge).
        No footprint check: the robot is where it is."""
        with self._lock:
            if self.state == RUNNING:
                raise ValueError('Stop the robot first.')
            self._ingest()
            self._apply_found(map_pose, 'Position set by hand', hint=False)
            if self.room is not None and self.cfg.lidar_reference == 'room':
                self.room.restart(tuple(float(v) for v in map_pose), trusted=False)
                self._room_step(self.client.snapshot(), self.clock())

    def _apply_found(self, pose, how, result=None, hint=True):
        """Adopt a pose that came from the LiDAR search (no pin). Not a footprint check: the robot may stand
        closer to a wall than A* would ever drive (then planning reports that the start is not clear)."""
        x, y, yaw = (float(v) for v in pose)
        if not self.base_grid.contains(self.base_grid.world_to_cell((x, y))):
            raise ValueError('That place is outside the map.')
        if self.estimator.odom is None:
            raise ValueError('No odometry received yet.')
        self._cancel_auto()
        fault = self.estimator.fault
        self.estimator.calibrate((x, y, yaw))
        self.estimator.fault = fault
        self.trajectory = []
        self.path, self.plan_result = [], None
        self.found = None
        self._pose_gen += 1
        self._search_misses = 0
        unfinished = self.state in (PLANNED, STOPPED) and self.goal is not None      # a route was waiting / interrupted
        if self.state in (PLANNED, ARRIVED):
            self.state = IDLE
        if self.room is not None:
            self.room.adopt((x, y, yaw))
        self.found_seq += 1
        fit = '' if result is None else ' (%.0f %% of the LiDAR view fits)' % (100 * result.matched)
        self._note('%s at (%.2f, %.2f) m, heading %.0f deg%s.%s'
                   % (how, x, y, math.degrees(yaw), fit,
                      ' Check the picture: if the robot is somewhere else, set Start by hand.' if hint else ''))
        text = self.message
        if unfinished and not self.safety.estop:
            self._plan()                            # the route to the same goal, from where the robot really is
            self.message = '%s %s' % (text, self.message)
        return text

    def find_robot(self, choose=None):
        """Operator action "Find robot": where does the LiDAR view fit? Returns (ok, message).
        ``choose`` = index into the candidates of the previous, ambiguous answer."""
        with self._lock:
            if self.state == RUNNING:
                return False, 'Stop the robot first.'
            if choose is not None:
                options = (self.found or {}).get('candidates') or []
                if not 0 <= int(choose) < len(options):
                    return False, 'That choice is no longer available: press Find robot again.'
                try:
                    self._apply_found(options[int(choose)]['pose'], 'Robot placed by your choice')
                except ValueError as exc:
                    return False, str(exc)
                return True, self.message
            snap = self._ingest()
            tel = snap['telemetry'] or {}
            scan = tel.get('scan')
            if not scan or scan.get('age', 9.0) > self.safety.cfg.scan_stale_s:
                return False, 'No fresh LiDAR scan.'
            if not self.robot_still(snap):
                return False, 'Wait until the robot stands still.'
            matcher, room = self._reference()
            if matcher is None:
                return False, ('The LiDAR has not drawn this room yet. Do one corner start first (Corner start, then '
                               'Confirm pose); after that the robot is found by itself.')
            points = self.safety.lidar_points(tel)
            spots, possible = self._standing_spots()
            generation = self._pose_gen
        result = matcher.locate(points, spots, possible=possible)          # about a second; without the lock
        with self._lock:
            if self.state == RUNNING or not self.robot_still() or generation != self._pose_gen:
                return False, 'The robot moved during the search: try again.'
            top = result.candidates[0] if result.candidates else None
            if result.ok and top['score'] >= SURE['min_score'] and top['known'] >= SURE['min_known']:
                try:
                    self._apply_found(result.pose, 'Robot found', result)
                except ValueError as exc:
                    return False, str(exc)
                return True, self.message
            if top is None:
                self.found = None
                return False, 'Robot not found: %s.' % result.reason
            # a possible place, but not sure enough to decide alone: the operator looks at the picture and chooses
            self.found = dict(reason=result.reason, candidates=[
                dict(pose=tuple(float(v) for v in c['pose']), score=c['score'], matched=c['matched'])
                for c in result.candidates[:4]])
            if len(result.candidates) > 1:
                return False, 'Robot not found for sure: %s.' % result.reason
            return False, ('The LiDAR is not sure (it has only drawn part of this room so far). Is the robot at the '
                           'marked place? Click it in the list to accept, or set Start by hand.')

    def room_file(self):
        """The room drawing as bytes if it is worth keeping (started from a corner start, mounting verified)."""
        with self._lock:
            room = self.room
            if room is None or not room.trusted or not room.calibrated or room.map.scans < 5:
                return None
            return room.map.to_bytes(dict(mount=list(self.safety.lidar_mount), saved=time.strftime('%Y-%m-%d %H:%M')))

    def load_room(self, data):
        """Use a room drawing from an earlier session. Returns a short description; ValueError if it does not fit."""
        with self._lock:
            room_map, meta = RoomMap.from_bytes(data)
            if any(abs(a - b) > 1e-6 for a, b in zip(room_map.extent, self.base_grid.extent)):
                raise ValueError('it was made for another field size')
            if any(abs(a - b) > 1e-4 for a, b in zip(meta.get('mount', (0, 0, 0)), self.safety.lidar_mount)):
                raise ValueError('it was made with another LiDAR mounting')
            self._room().use_map(room_map, trusted=True)
            self._pose_gen += 1
            self._search_misses = 0
            return 'room outline from %s (%d wall cells)' % (meta.get('saved', 'an earlier session'), room_map.cells())

    def _not_really_moving(self, telemetry):
        """Odometry says the robot moved, but does it really? (MentorPi odometry can be computed from the
        COMMANDED speed, so it keeps "moving" even when the motors have no power or the wheels are stuck.)
        Compares the LiDAR view now with the view when odometry was 0.3 m / 45 deg earlier."""
        pts = ((telemetry or {}).get('scan') or {}).get('points') or []
        if len(pts) < 30:
            return None
        bins = _scan_bins(pts)
        dist, rot = self.estimator.odometer_m, self.estimator.odometer_rad
        ref = self._motion_ref
        if ref is None:
            self._motion_ref = (dist, rot, bins)
            return None
        moved, turned = dist - ref[0], rot - ref[1]
        if moved < 0.30 and turned < 0.8:
            return None
        self._motion_ref = (dist, rot, bins)
        change = _scan_change(ref[2], bins)
        if change is not None and change < 0.04:
            return ('the robot is NOT really moving: odometry says it moved %.2f m / turned %.0f deg, but the '
                    'LiDAR view did not change. Check the motor power switch / battery and that the wheels touch '
                    'the floor, then confirm the pose again.' % (moved, math.degrees(turned)))
        return None

    def _replan_around_obstacle(self, pose, telemetry):
        points = self.safety.unknown_points(self.safety.lidar_points(telemetry) or [], pose, self.grid,
                                            self.estimator.sigma)
        grid, added = mark_scan_obstacles(self.grid, pose, points, self.cfg.obstacle_map_range_m,
                                          protect_radius_m=self.radius)
        if added:
            self.lidar_mask |= grid.cells & ~self.grid.cells
            self._compose()
        self._after_replan(added, 'Obstacle added to edge map')

    def _after_replan(self, added, what):
        """Plan again from the current pose; continue automatically only if enabled and allowed."""
        if self.goal is None:
            return
        result = self._plan()
        if not (result is not None and result.found and self.state == PLANNED):
            self._blocked_while_running = True
            self._note('%s (%d cells); no alternative route to the goal. Robot STOPPED (it will retry when a '
                       'camera obstacle disappears).' % (what, added))
            return
        if not self.auto_enabled():
            self._note('%s (%d cells); new A* route planned on the edge. '
                       'Robot is STOPPED: authorize to continue.' % (what, added))
        elif added == 0:
            # Nothing new was mapped (e.g. only the UGV's own stop zone fired): the same route would
            # hit the same stop again, so leave the decision to the operator.
            self._note('Stopped by the UGV safety zone but no new obstacle could be mapped; '
                       'route unchanged. Authorize to retry.')
        elif self.auto_replans >= self.cfg.max_auto_replans:
            self._note('%s (%d cells); new route planned, but the automatic '
                       'continue limit (%d) is reached. Authorize to continue.' % (what, added, self.cfg.max_auto_replans))
        else:
            self._auto_resume_at = self.clock() + self.cfg.auto_continue_delay_s
            self._note('%s (%d cells); new A* route planned on the edge. '
                       'Auto-continue %d/%d in %.1f s (press STOP to cancel).'
                       % (what, added, self.auto_replans + 1, self.cfg.max_auto_replans, self.cfg.auto_continue_delay_s))

    def _try_auto_continue(self, now, snap):
        if now < self._auto_resume_at:
            return
        if self.safety.estop:  # never override an operator STOP
            self._auto_resume_at = None
            return
        if not self.robot_still(snap):
            self._auto_resume_at = now + 0.2  # still coasting: wait until the robot is really still
            return
        self._auto_resume_at = None
        pose = self.estimator.pose()
        if pose is None or not self.path or math.dist(pose[:2], self.path[0]) > 0.03:
            self._plan()  # robot settled slightly away from the planned start: replan from where it is
            if self.state != PLANNED:
                return
        self.auto_replans += 1
        ok, msg = self._arm_and_run('AUTO-CONTINUE %d/%d: driving the replanned route.'
                                    % (self.auto_replans, self.cfg.max_auto_replans))
        if not ok:
            self._note('Auto-continue not possible (%s). Robot STOPPED: authorize to continue.' % msg)

    # ------------------------------------------------------------ background loop (headless use)
    def fast_state(self):
        """Cheap, never-blocking sample for displays (pose, state, command). While the control thread
        holds the lock (e.g. planning) the previous sample is returned."""
        if self._lock.acquire(blocking=self._fast is None):      # the very first call waits; later ones never
            try:
                self._ingest()
                cmd = self.last_command if self.state == RUNNING else None
                self._fast = dict(t=self.clock(), pose=self.estimator.pose(), state=self.state,
                                  cmd=None if cmd is None else (cmd.vx, cmd.wz), sigma=self.estimator.sigma)
            finally:
                self._lock.release()
        return self._fast

    def loop_stats(self):
        """Control-loop health over the last ~5 s: achieved rate, worst gap between cycles, worst cycle time."""
        periods, durations = list(self._loop['periods']), list(self._loop['durations'])
        if not periods:
            return dict(hz=0.0, max_gap_ms=0.0, max_work_ms=0.0, target_hz=self.cfg.control_rate_hz)
        return dict(hz=round(len(periods) / max(sum(periods), 1e-9), 1), max_gap_ms=round(max(periods) * 1000, 1),
                    max_work_ms=round(max(durations) * 1000, 1), target_hz=self.cfg.control_rate_hz)

    def run_in_background(self):
        def loop():
            period = 1.0 / self.cfg.control_rate_hz
            keep = max(10, int(5 * self.cfg.control_rate_hz))
            while not self._stop_thread.is_set():
                started = self.clock()
                stats = self._loop
                if stats['last'] is not None:
                    stats['periods'].append(started - stats['last']); del stats['periods'][:-keep]
                stats['last'] = started
                try:
                    self.tick()
                    if self.mission is not None:
                        with self._lock:
                            self.mission.step()
                except Exception as exc:  # any bug -> stop the robot
                    with self._lock:
                        self._halt(STOPPED, 'Stopped: internal error %r' % exc)
                        if self.mission is not None:
                            self.mission.pause('internal error %r' % exc)
                stats['durations'].append(self.clock() - started); del stats['durations'][:-keep]
                self._stop_thread.wait(max(0.0, period - (self.clock() - started)))
        self._stop_thread.clear()
        self._thread = threading.Thread(target=loop, name='navigator', daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop_thread.set()
        if self._thread:
            self._thread.join(timeout=2)
        with self._lock:
            if self.state == RUNNING:
                self._halt(STOPPED, 'Navigator shut down.', 'operator')

    def snapshot(self, blocking=True):
        """State for displays. ``blocking=False`` (GUI): if the control thread is busy (e.g. A* planning),
        return the previous snapshot instead of freezing the window."""
        if not self._lock.acquire(blocking=blocking):
            if self._last_snapshot is not None:
                return self._last_snapshot
            self._lock.acquire()
        try:
            self._last_snapshot = self._snapshot()
            return self._last_snapshot
        finally:
            self._lock.release()

    def _snapshot(self):
        now = self.clock()
        cmd = self.last_command if self.state == RUNNING else None
        sc = self.safety.cfg
        zone = None
        if cmd is not None and self.motion == 'path':
            zone = stopping_distance(abs(cmd.vx), sc.stop_distance_m, sc.reaction_time_s, sc.brake_decel)
        return dict(state=self.state, message=self.message, pose=self.estimator.pose(),
                    localization=self.estimator.status(now), sigma=self.estimator.sigma,
                    max_sigma=self.estimator.cfg.max_sigma_m,
                    path=list(self.path), goal=self.goal, trajectory=list(self.trajectory),
                    command=self.last_command, reasons=list(self.last_reasons), estop=self.safety.estop,
                    grid=self.display_grid, base_grid=self.base_grid, lidar_mask=self.lidar_mask,
                    map_rev=self.map_rev, vision_obstacles=list(self.vision_obstacles),
                    log=list(self.log[-8:]), raw_pose=self.estimator.raw_pose(), anchor=self.estimator.anchor,
                    speed_fraction=self.speed_fraction, scale=(self.estimator.scale_x, self.estimator.scale_y),
                    auto_replans=self.auto_replans, auto_pending=self._auto_resume_at is not None,
                    motion=self.motion, stop_kind=self.stop_kind, plan_info=dict(self.plan_info),
                    stop_zone_m=zone, segment=None if self.tracker is None else self.tracker.segment,
                    room=None if self.room is None or self.cfg.lidar_reference != 'room' else self.room.status(),
                    found=self.found, searching=self._search is not None)


def _scan_bins(points, n=72):
    """Closest LiDAR range in each of ``n`` bearing bins (NaN = no return)."""
    out = np.full(n, np.nan)
    for x, y in points:
        k = int((math.atan2(y, x) + math.pi) / (2 * math.pi) * n) % n
        r = math.hypot(x, y)
        if not r >= out[k]:          # also true for NaN
            out[k] = r
    return out


def _scan_change(a, b):
    """90th percentile of the range change per bearing (metres), or None if too few common bins."""
    both = ~np.isnan(a) & ~np.isnan(b)
    if both.sum() < 12:
        return None
    return float(np.percentile(np.abs(a[both] - b[both]), 90))


class SpinTracker:
    """Turn in place through absolute headings, holding still ``dwell_s`` at each one.

    Same interface as PathTracker (``update`` -> Command, ``segment``). Used by the search
    mission to look around with the camera. Fails if the heading stops improving.
    """

    def __init__(self, headings, dwell_s, config: ControllerConfig, max_rate=None, tolerance_deg=4.0):
        if not headings:
            raise ValueError('No headings to turn to.')
        self.targets = list(headings)
        self.dwell_s = dwell_s
        self.cfg = config
        self.max_rate = max_rate
        self.tol = math.radians(tolerance_deg)
        self.segment = 0
        self.index = 0
        self.dwell_start = None
        self.best = float('inf')
        self.last_progress = None

    def update(self, pose, now, dt):
        c = self.cfg
        if self.dwell_start is not None:
            if now - self.dwell_start < self.dwell_s:
                return Command(0.0, 0.0, 0.0, 'rotating', 'looking')
            self.dwell_start = None
            self.index += 1
            self.best, self.last_progress = float('inf'), None
            if self.index >= len(self.targets):
                return Command(status='arrived', message='%d heading(s) checked' % len(self.targets))
        error = wrap(self.targets[self.index] - pose[2])
        if abs(error) <= self.tol:
            self.dwell_start = now
            return Command(0.0, 0.0, 0.0, 'rotating', 'looking')
        if self.last_progress is None or abs(error) < self.best - math.radians(c.progress_min_deg):
            self.best, self.last_progress = abs(error), now
        elif now - self.last_progress > c.progress_timeout_s:
            return Command(status='failed', message='turn made no progress for %.1f s' % (now - self.last_progress))
        limit = c.max_w if self.max_rate is None else min(c.max_w, self.max_rate)
        w = max(-limit, min(limit, c.kp_w * error))
        if abs(w) < c.min_w:
            w = math.copysign(c.min_w, error)
        return Command(0.0, 0.0, w, 'rotating', 'turning to look around', heading_error=error)
