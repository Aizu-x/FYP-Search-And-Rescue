"""Position from the LiDAR when it looks OVER the arena.

On the MentorPi the LiDAR sits higher than the arena walls and the blocks, so it never sees the arena.
It does see the ROOM around it: walls, furniture, a door. The room does not move, which makes it a
landmark that is independent of the wheels:

* ``RoomMap``      what the LiDAR has seen so far, drawn as a grid in ARENA coordinates (hit / free
                   evidence per cell). It starts when the operator confirms the robot pose and grows
                   while the robot moves. People walking by fade out again.
* ``RoomTracker``  matches every new scan to that drawing. The result is the robot's TRUE movement:
                   it does not care about wheel slip, about odometry that is computed from commanded
                   speed, or about the robot being carried by hand.

Two things are learnt from the robot's own motion before the tracker is allowed to correct the position
("observe first"), because they cannot be read from a data sheet:

* the direction of the sensor (yaw): while the robot drives straight, the tracked position must move along
  the heading. A constant sideways share is the yaw error of the mounting.
* the place of the sensor on the robot (x, y): while the robot turns on the spot, a sensor that is not at
  the centre of rotation travels on a circle. Its radius and phase are the offset.

Nothing here commands the robot. The navigator decides what to do with the result.
"""
from __future__ import annotations

import io
import math
from dataclasses import dataclass, field

import numpy as np

from .localization import compose, inverse, wrap
from .scan_match import ScanMatcher

HIT, MISS, LOW, HIGH, OCCUPIED = 0.9, 0.25, -2.0, 3.6, 0.5      # log-odds evidence per scan and limits


def capped_distance(occupied, cap_cells):
    """Exact distance (in cells) from every cell to the nearest occupied cell, clipped at ``cap_cells``."""
    occupied = np.asarray(occupied, bool)
    h, w = occupied.shape
    cols = np.arange(w)[None, :]
    far = 10 ** 6
    left = np.maximum.accumulate(np.where(occupied, cols, -far), axis=1)
    right = np.minimum.accumulate(np.where(occupied, cols, far)[:, ::-1], axis=1)[:, ::-1]
    row = np.minimum(np.minimum(cols - left, right - cols), cap_cells + 1).astype(np.float32)
    row *= row
    best = np.full((h, w), float((cap_cells + 1) ** 2), np.float32)
    for dy in range(-cap_cells, cap_cells + 1):
        src = slice(max(0, dy), min(h, h + dy))
        dst = slice(max(0, -dy), min(h, h - dy))
        np.minimum(best[dst], row[src] + float(dy * dy), out=best[dst])
    return np.minimum(np.sqrt(best), float(cap_cells))


def _dilate(mask, n):
    out = mask.copy()
    for k in range(1, n + 1):
        out[:, k:] |= mask[:, :-k]; out[:, :-k] |= mask[:, k:]
    wide = out.copy()
    for k in range(1, n + 1):
        out[k:, :] |= wide[:-k, :]; out[:-k, :] |= wide[k:, :]
    return out


class RoomMap:
    def __init__(self, extent, resolution=0.03, margin=4.5):
        x0, x1, y0, y1 = (float(v) for v in extent)
        self.extent = (x0, x1, y0, y1)
        self.res = float(resolution)
        self.origin = (x0 - margin, y0 - margin)
        self.shape = (int(math.ceil((y1 - y0 + 2 * margin) / resolution)),
                      int(math.ceil((x1 - x0 + 2 * margin) / resolution)))
        self.logodds = np.zeros(self.shape, np.float32)
        self.seen = np.zeros(self.shape, bool)
        self.scans = 0
        self.rev = 0
        self._matcher = None

    # ------------------------------------------------------------------ content
    @property
    def empty(self):
        return self.scans == 0

    def occupied(self):
        return self.logodds >= OCCUPIED

    def cells(self):
        return int(self.occupied().sum())

    def _index(self, wx, wy):
        c = np.floor((wx - self.origin[0]) / self.res).astype(int)
        r = np.floor((wy - self.origin[1]) / self.res).astype(int)
        return r, c, (r >= 0) & (r < self.shape[0]) & (c >= 0) & (c < self.shape[1])

    def integrate(self, pose, points, sensor=(0.0, 0.0), min_range=0.15):
        """Add one scan taken at robot ``pose``. ``points``: robot-frame (x, y) hits; ``sensor``: where the
        LiDAR sits on the robot (rays start there). Cells the rays crossed count as free, the ends as walls."""
        pts = np.asarray(points if points is not None else [], float).reshape(-1, 2)
        if not len(pts):
            return 0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        ox = pose[0] + c * sensor[0] - s * sensor[1]
        oy = pose[1] + s * sensor[0] + c * sensor[1]
        wx = pose[0] + c * pts[:, 0] - s * pts[:, 1]
        wy = pose[1] + s * pts[:, 0] + c * pts[:, 1]
        dx, dy = wx - ox, wy - oy
        length = np.hypot(dx, dy)
        keep = length >= min_range
        wx, wy, dx, dy, length = wx[keep], wy[keep], dx[keep], dy[keep], length[keep]
        if not len(wx):
            return 0
        hit = np.zeros(self.shape, bool)
        r, col, inside = self._index(wx, wy)
        hit[r[inside], col[inside]] = True
        steps = np.arange(0.5, float(length.max()) / self.res, 0.7) * self.res          # along every ray
        along = steps[None, :]
        free_part = along < (length[:, None] - 1.5 * self.res)
        fx = ox + (dx / length)[:, None] * along
        fy = oy + (dy / length)[:, None] * along
        r, col, inside = self._index(fx, fy)
        ok = inside & free_part
        free = np.zeros(self.shape, bool)
        free[r[ok], col[ok]] = True
        free &= ~hit
        self.logodds[free] -= MISS
        self.logodds[hit] += HIT
        np.clip(self.logodds, LOW, HIGH, out=self.logodds)
        self.seen |= free | hit
        self.scans += 1
        self.rev += 1
        self._matcher = None
        return int(hit.sum())

    def matcher(self):
        """ScanMatcher on the current drawing (rebuilt only when the drawing changed)."""
        if self._matcher is None or self._matcher[0] != self.rev:
            distance = capped_distance(self.occupied(), 10) * self.res
            self._matcher = (self.rev, ScanMatcher(distance=distance, origin=self.origin, resolution=self.res,
                                                   known=_dilate(self.seen, 2), edge_is_wall=False))
        return self._matcher[1]

    def wall_points(self, limit=2500):
        """Centres of the wall cells as [[x, y], ...] for the display."""
        rows, cols = np.nonzero(self.occupied())
        if len(rows) > limit:
            pick = np.linspace(0, len(rows) - 1, limit).astype(int)
            rows, cols = rows[pick], cols[pick]
        return [[round(self.origin[0] + (c + 0.5) * self.res, 3), round(self.origin[1] + (r + 0.5) * self.res, 3)]
                for r, c in zip(rows.tolist(), cols.tolist())]

    # ------------------------------------------------------------------ keep for the next session
    def to_bytes(self, meta=None):
        buffer = io.BytesIO()
        np.savez_compressed(buffer, logodds=self.logodds.astype(np.float16), seen=np.packbits(self.seen),
                            shape=np.array(self.shape), origin=np.array(self.origin), res=np.array(self.res),
                            extent=np.array(self.extent), scans=np.array(self.scans),
                            meta=np.array(repr(dict(meta or {}))))
        return buffer.getvalue()

    @classmethod
    def from_bytes(cls, data):
        """(RoomMap, meta dict). Raises ValueError for anything that is not a room file."""
        import ast
        try:
            z = np.load(io.BytesIO(data), allow_pickle=False)
            shape = tuple(int(v) for v in z['shape'])
            room = cls.__new__(cls)
            room.extent = tuple(float(v) for v in z['extent'])
            room.res = float(z['res'])
            room.origin = tuple(float(v) for v in z['origin'])
            room.shape = shape
            room.logodds = z['logodds'].astype(np.float32)
            room.seen = np.unpackbits(z['seen'])[:shape[0] * shape[1]].reshape(shape).astype(bool)
            room.scans = int(z['scans'])
            meta = ast.literal_eval(str(z['meta']))
        except Exception as exc:
            raise ValueError('not a room file (%s)' % exc)
        if room.logodds.shape != shape or len(shape) != 2 or max(shape) > 2000 or not isinstance(meta, dict):
            raise ValueError('room file is damaged')
        room.rev, room._matcher = 1, None
        return room, meta


# --------------------------------------------------------------------------------------------- tracker
@dataclass
class RoomConfig:
    period_moving_s: float = 0.5        # one match about every half second while driving
    period_still_s: float = 0.5
    window_xy: float = 0.09             # search this far around the odometry prediction while driving
    window_yaw: float = math.radians(6.0)
    hand_window_xy: float = 0.21        # standing still by odometry but the view changed: somebody moved the robot
    hand_window_yaw: float = math.radians(16.0)
    min_points: int = 40                # LiDAR points needed for a match
    max_turn_rate: float = 0.30         # rad/s: scans taken while turning faster are smeared
    max_scan_age_s: float = 0.35
    warmup_scans: int = 3               # scans drawn from the starting spot before matching begins
    integrate_step_m: float = 0.06      # draw a new scan after moving this far ...
    integrate_step_rad: float = math.radians(6.0)   # ... or turning this far
    still_fix_m: float = 0.012          # standing still: only correct when the fit moved by more than this
    still_fix_rad: float = math.radians(0.8)
    lost_after: int = 3                 # failed matches in a row (standing still) before searching everywhere
    # learning how the LiDAR is mounted
    cal_straight_m: float = 0.5         # straight driving needed for the direction (yaw)
    cal_turn_info: float = 2.0          # turning needed for the position (one 90 degree turn gives 2.0)
    cal_min_chord_rad: float = math.radians(20.0)
    cal_done_yaw: float = math.radians(1.0)     # below this the mounting counts as right
    cal_done_xy: float = 0.012
    cal_max_xy: float = 0.20            # an offset larger than this is not believable: ignore the data
    cal_max_residual: float = 0.02      # turns must agree with one offset this well (RMS, metres)


@dataclass
class Step:
    correct: tuple = None               # pose the navigator should adopt now (None = leave the estimate alone)
    mount: tuple = None                 # new LiDAR mounting (x, y, yaw) to use from now on
    search: bool = False                # lost while standing still: please search the whole map
    notes: list = field(default_factory=list)


class RoomTracker:
    """See the module text. ``mount`` = (x, y, yaw) the caller already applies to the raw LiDAR points;
    ``calibrated`` = that mounting was verified by this robot's motion (then corrections are allowed)."""

    def __init__(self, extent, config: RoomConfig = None, mount=(0.0, 0.0, 0.0), calibrated=False):
        self.cfg = config or RoomConfig()
        self.extent = tuple(extent)
        self.map = RoomMap(self.extent)
        self.mount = tuple(float(v) for v in mount)
        self.calibrated = bool(calibrated)
        self.trusted = False            # this drawing started from a corner start (worth keeping)
        self.origin = None              # robot pose when this drawing was started
        self.pose = None                # the tracker's own (LiDAR) pose
        self._last_est = None
        self.state = 'waiting'          # waiting | learning | locked | searching | lost
        self.reason = ''
        self.quality = None
        self.fails = 0
        self.moved_by_hand = 0.0
        self._next = 0.0
        self._drawn_at = None
        self._prev = None               # last good match: (tracker pose, raw odometry pose, time)
        self._turn_ref = None           # ... the one where the current turn began
        self._verifying = False         # a mounting was just measured: the next round only has to confirm it
        self._reset_calibration()

    # ------------------------------------------------------------------ operator / navigator events
    def restart(self, pose, trusted=False, estimate=None):
        """The robot is at ``pose``: forget the drawing and start a new one there. ``estimate``: the
        navigator's own estimate right now if it differs from ``pose`` (default: it is ``pose``)."""
        self.map = RoomMap(self.extent)
        self.trusted = bool(trusted)
        self.origin = None
        self.pose = tuple(pose)
        self._last_est = tuple(pose if estimate is None else estimate)
        self.state, self.reason, self.quality, self.fails = 'learning', '', None, 0
        self._drawn_at = self._prev = self._turn_ref = None
        self._next = 0.0

    def adopt(self, pose):
        """The pose was found IN this drawing (Find robot): keep the drawing, continue from there."""
        self.pose = tuple(pose)
        self._last_est = tuple(pose)
        self.fails = 0
        self._prev = self._turn_ref = None

    def estimator_is(self, pose):
        """The navigator changed its estimate (applied a correction): do not count that as motion."""
        self._last_est = None if pose is None else tuple(pose)

    def use_map(self, room, trusted=True):
        self.map = room
        self.trusted = trusted
        self.origin = None
        self.pose = self._last_est = None
        self.state, self.reason, self.quality, self.fails = 'waiting', '', None, 0
        self._drawn_at = self._prev = self._turn_ref = None

    @property
    def active(self):
        return self.calibrated

    def status(self):
        q = self.quality or {}
        cal = self._cal
        return dict(state=self.state, reason=self.reason, fit=q.get('matched'), score=q.get('score'),
                    known=q.get('known'), cells=self.map.cells() if not self.map.empty else 0,
                    scans=self.map.scans, trusted=self.trusted, calibrated=self.calibrated,
                    mount=[round(self.mount[0], 4), round(self.mount[1], 4), round(math.degrees(self.mount[2]), 2)],
                    learn=dict(straight=round(min(1.0, math.hypot(cal['forward'], cal['lateral'])
                                                  / self.cfg.cal_straight_m), 2),
                               turn=round(min(1.0, cal['info'] / self.cfg.cal_turn_info), 2)))

    # ------------------------------------------------------------------ one control cycle
    def step(self, now, est_pose, points, odom, vx=0.0, wz=0.0, scan_age=0.0, still=True):
        """``est_pose``: the navigator's estimate; ``points``: robot-frame LiDAR points (mounting applied);
        ``odom``: raw odometry pose (only differences are used). Returns a ``Step``."""
        out = Step()
        cfg = self.cfg
        if est_pose is None:
            self.pose = self._last_est = None
            self.state = 'waiting'
            return out
        if self.pose is None or self._last_est is None:
            self.pose = tuple(est_pose)
        else:                                   # carry the LiDAR pose along with the odometry step
            self.pose = compose(self.pose, compose(inverse(self._last_est), est_pose))
        self._last_est = tuple(est_pose)
        pts = np.asarray(points if points is not None else [], float).reshape(-1, 2)
        if len(pts) < cfg.min_points:
            self.state, self.reason = 'searching', 'the LiDAR sees too little (%d points)' % len(pts)
            return out
        if abs(wz) > cfg.max_turn_rate or scan_age > cfg.max_scan_age_s or now < self._next:
            return out
        self._next = now + (cfg.period_still_s if still else cfg.period_moving_s)
        back = (-vx * scan_age, 0.0, -wz * scan_age)            # where the robot was when the scan was taken
        then = compose(self.pose, back)
        sensor = self.mount[:2]
        room = self.map
        if room.empty:                          # the first scan after the pose was confirmed starts the drawing
            self.origin = tuple(then)
            room.integrate(then, pts, sensor)
            self._drawn_at = then
            self.state, self.reason = 'learning', 'drawing the room from here'
            return out
        if room.scans < cfg.warmup_scans and still and self.origin is not None \
                and math.dist(then[:2], self.origin[:2]) < 0.01 and abs(wrap(then[2] - self.origin[2])) < 0.02:
            room.integrate(then, pts, sensor)   # a few more scans from the very same spot (less noise)
            return out
        matcher = room.matcher()
        result = matcher.match(then, pts, cfg.window_xy, cfg.window_yaw, precise=True)
        if not result.ok and (still or not self.calibrated):
            # standing still by odometry, yet the view changed: moved by hand? Or the mounting is still unknown
            # and may be far off (then nothing is corrected anyway, the tracker only watches).
            result = matcher.match(then, pts, cfg.hand_window_xy, cfg.hand_window_yaw, precise=True)
            if result.ok and matcher.crossing(result.pose, pts, sensor) > 0.12:
                result.ok, result.reason = False, 'the view only looks similar there (rays would pass through walls)'
        if not result.ok:
            self.fails += 1
            self.quality = dict(score=result.start_score, matched=result.matched, known=None)
            self.state, self.reason = ('lost' if self.fails >= cfg.lost_after else 'searching'), result.reason
            if still and self.fails >= cfg.lost_after and self.fails % cfg.lost_after == 0:
                out.search = True
            return out
        self.fails = 0
        found = tuple(result.pose)
        if still and result.shift_m > 0.06:
            self.moved_by_hand += result.shift_m
        self.pose = compose(found, (vx * scan_age, 0.0, wz * scan_age))
        # standing still and the estimate is (nearly) where the LiDAR says: leave it alone (no jitter)
        steady = (still and math.dist(self.pose[:2], est_pose[:2]) < cfg.still_fix_m
                  and abs(wrap(self.pose[2] - est_pose[2])) < cfg.still_fix_rad)
        self.quality = matcher.evaluate(found, pts)
        self.state, self.reason = 'locked', ''
        if self._drawn_at is None or math.dist(found[:2], self._drawn_at[:2]) > cfg.integrate_step_m \
                or abs(wrap(found[2] - self._drawn_at[2])) > cfg.integrate_step_rad:
            room.integrate(found, pts, sensor)
            self._drawn_at = found
        self._learn_mounting(found, odom, now, out)
        if out.mount is None and self.active and not steady:    # (standing still and it fits: do not jitter)
            out.correct = self.pose
        return out

    # ------------------------------------------------------------------ how is the LiDAR mounted?
    def _reset_calibration(self):
        self._cal = dict(forward=0.0, lateral=0.0, odometry=0.0, info=0.0, turns=[])

    def _learn_mounting(self, found, odom, now, out):
        """Collect evidence from two consecutive good matches: a straight piece (direction of the sensor) or a
        turn with little driving (place of the sensor). Solve when there is enough of both."""
        cfg, cal = self.cfg, self._cal
        prev, self._prev = self._prev, (found, tuple(odom), now)
        if prev is None:
            return
        p0, o0, t0 = prev
        moved = math.dist(odom[:2], o0[:2])
        turned = abs(wrap(odom[2] - o0[2]))
        dx, dy = found[0] - p0[0], found[1] - p0[1]
        if moved > 0.03 and turned < math.radians(2.0) and now - t0 < 3.0:      # a straight piece
            c, s = math.cos(p0[2]), math.sin(p0[2])
            forward, lateral = c * dx + s * dy, -s * dx + c * dy
            if 0.5 * moved < math.hypot(forward, lateral) < 1.6 * moved:        # (not while being carried)
                cal['forward'] += forward; cal['lateral'] += lateral; cal['odometry'] += moved
            self._turn_ref = self._prev                                         # a turn can only start after it
        # a turn with little driving, however slowly it happens: compare with the match where the turn began
        ref = self._turn_ref
        if ref is None or math.dist(odom[:2], ref[1][:2]) > 0.12 or now - ref[2] > 10.0:
            ref = self._turn_ref = prev
        p0, o0, t0 = ref
        turned = abs(wrap(odom[2] - o0[2]))
        if turned >= cfg.cal_min_chord_rad and math.dist(odom[:2], o0[:2]) <= 0.12:
            dx, dy = found[0] - p0[0], found[1] - p0[1]
            angle = abs(wrap(found[2] - p0[2]))
            if abs(angle - turned) < math.radians(6.0) and math.hypot(dx, dy) <= 2 * cfg.cal_max_xy + 0.12:
                c, s = math.cos(o0[2]), math.sin(o0[2])                         # odometry step in the robot frame
                ox, oy = odom[0] - o0[0], odom[1] - o0[1]
                cal['turns'].append((p0[2], found[2], dx, dy, c * ox + s * oy, -s * ox + c * oy))
                cal['info'] += 4 * math.sin(angle / 2) ** 2
            self._turn_ref = self._prev
        need = 0.5 if self._verifying else 1.0                                  # a check needs half the evidence
        driven = math.hypot(cal['forward'], cal['lateral'])
        if driven < need * cfg.cal_straight_m or cal['info'] < need * cfg.cal_turn_info or len(cal['turns']) < 2:
            return
        yaw = -math.atan2(cal['lateral'], cal['forward'])
        scale = driven / max(cal['odometry'], 1e-6)                             # real distance per odometry metre
        a, b, chords = np.zeros((2, 2)), np.zeros(2), []
        for th0, th1, tx, ty, lx, ly in cal['turns']:
            m = _rot(th1) - _rot(th0)
            d = np.array([tx, ty]) - scale * (_rot(th0 - yaw) @ np.array([lx, ly]))   # minus the little driving
            a += m.T @ m; b += m.T @ d
            chords.append((m, d))
        offset = np.linalg.solve(a, b)
        residual = math.sqrt(sum(float(((m @ offset - d) ** 2).sum()) for m, d in chords) / len(chords))
        self._reset_calibration()
        if residual > cfg.cal_max_residual or float(np.hypot(*offset)) > cfg.cal_max_xy:
            out.notes.append('LiDAR mounting check: the turns did not agree with each other (%.1f cm): '
                             'measuring again.' % (100 * residual))
            return
        if abs(yaw) < cfg.cal_done_yaw and float(np.hypot(*offset)) < cfg.cal_done_xy:
            self._verifying = False
            if not self.calibrated:
                self.calibrated = True
                out.notes.append('LiDAR mounting verified by the robot\'s own motion (%.1f cm ahead, %.1f cm left, '
                                 'turned %.1f deg). Live LiDAR tracking is now correcting the position. Odometry '
                                 'reads %.0f %% of the real distance.'
                                 % (100 * self.mount[0], 100 * self.mount[1], math.degrees(self.mount[2]),
                                    100 / max(scale, 1e-6)))
            return
        shift = _rot(yaw) @ offset                                # residual transform D = (shift, yaw)
        d = (float(shift[0]), float(shift[1]), float(yaw))
        self.mount = compose(d, self.mount)
        self.calibrated = False                                   # verified by the next round of driving
        self._verifying = True
        out.mount = self.mount
        out.notes.append('LiDAR mounting measured from the robot\'s motion: %.1f cm ahead of the centre, %.1f cm '
                         'to the left, turned %.1f deg. Drawing the room again with it; it is checked once more '
                         'before the position is corrected.'
                         % (100 * self.mount[0], 100 * self.mount[1], math.degrees(self.mount[2])))
        if abs(wrap(self.mount[2])) > math.radians(20.0):
            out.notes.append('WARNING: the LiDAR is turned by about %.0f deg compared with what the robot itself '
                             'assumes. The console now allows for it, but the robot\'s own LiDAR stop zone looks '
                             'the wrong way until "lidar_mount" is set in bridge_config.json on the robot.'
                             % math.degrees(wrap(self.mount[2])))
        if self.origin is not None:                               # what the pose really was all along
            true = compose(compose(self.origin, d), compose(compose(inverse(self.origin), self.pose), inverse(d)))
        else:
            true = self.pose
        trusted = self.trusted and self.origin is not None
        self.restart(true, trusted, estimate=self._last_est)
        if self.active:
            out.correct = true


def _rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def apply_mount(points, mount):
    """Raw LiDAR points -> robot frame for a sensor mounted at ``mount`` = (x, y, yaw) on the robot."""
    mx, my, yaw = mount
    if not points or (mx == 0.0 and my == 0.0 and yaw == 0.0):
        return points
    c, s = math.cos(yaw), math.sin(yaw)
    return [(mx + c * p[0] - s * p[1], my + s * p[0] + c * p[1]) for p in points]

