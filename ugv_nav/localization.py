"""SE(2) helpers, odometry-to-map alignment and pose confidence.

Frames
------
map  : the A* grid frame from ``ugv_planner.grid`` -- origin at the lower-left
       map corner, x right (columns), y up (rows after the in-memory flip),
       yaw counter-clockwise from +x. Units meters/radians.
odom : the MentorPi odometry frame (wherever the robot was when odometry
       started). Raw odometry is NOT a map position.

``map_T_odom`` is established by an explicit initial-pose calibration: the
operator places the robot at a known, physically marked map pose and confirms
it. Afterwards ``map_pose = map_T_odom * odom_pose``.

Scale calibration: ``scale_x`` / ``scale_y`` correct a systematic mismatch
between how far the robot believes it moved (odometry) and how far it moved on
the map, per map axis. The correction is applied to the displacement since the
last initial-pose calibration (the "anchor"):
``x = anchor_x + scale_x * (raw_x - anchor_x)`` and likewise for y. 1.0 = no
correction. The map itself and A* are not changed.

Confidence: wheel odometry drifts without bound. A conservative position
uncertainty grows linearly with distance travelled and rotation since the last
calibration. Navigation is refused/stopped once it exceeds the limit, when
feedback is stale, or when odometry jumps (reset/glitch).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    ax, ay, at = a; bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(at + bt))


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, wrap(-t))


def transform_point(pose, point):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return (x + c * point[0] - s * point[1], y + s * point[0] + c * point[1])


@dataclass
class LocalizationConfig:
    stale_after_s: float = 0.5        # odometry older than this -> localization lost
    sigma0_m: float = 0.02            # placement uncertainty at calibration
    drift_per_m: float = 0.05         # +5 cm uncertainty per metre travelled (conservative wheel-odom guess -- MEASURE in Test D)
    drift_per_rad_m: float = 0.01     # +1 cm per radian turned
    max_sigma_m: float = 0.15         # stop navigation above this
    max_jump_m: float = 0.15          # odometry step between samples larger than this -> lost
    heading_source: str = 'odom'      # 'odom' or 'imu'
    scale_x: float = 1.0              # map-axis scale correction (from the X calibration run)
    scale_y: float = 1.0              # map-axis scale correction (from the Y calibration run)


SCALE_MIN, SCALE_MAX = 0.7, 1.3       # outside this a calibration run almost certainly went wrong
MIN_CAL_DISTANCE_M = 0.3              # a calibration run must move at least this far along its axis


def validate_scale(value, name='scale'):
    value = float(value)
    if not math.isfinite(value) or not SCALE_MIN <= value <= SCALE_MAX:
        raise ValueError('%s %.3f is outside %.2f-%.2f; check the calibration run.' % (name, value, SCALE_MIN, SCALE_MAX))
    return value


def compute_axis_scale(axis, raw_start, raw_end, true_start, true_end):
    """Scale for one map axis from a calibration run.

    raw_*: the robot's own UNSCALED estimate (map frame) at the start/end of the run.
    true_*: where the robot really was, as marked by the operator on the map.
    Returns (scale, info dict). Raises ValueError for an unusable run.
    """
    i = 0 if axis == 'x' else 1
    raw = raw_end[i] - raw_start[i]
    true = true_end[i] - true_start[i]
    if abs(raw) < MIN_CAL_DISTANCE_M or abs(true) < MIN_CAL_DISTANCE_M:
        raise ValueError('The run moved only %.2f m along %s; drive at least %.1f m along that axis.'
                         % (abs(true), axis.upper(), MIN_CAL_DISTANCE_M))
    if (raw > 0) != (true > 0):
        raise ValueError('Robot and marked position moved in opposite directions along %s; '
                         'check the initial heading.' % axis.upper())
    scale = validate_scale(true / raw, '%s scale' % axis.upper())
    other = 1 - i
    sideways_error = (true_end[other] - true_start[other]) - (raw_end[other] - raw_start[other])
    return scale, dict(robot_measured_m=abs(raw), marked_m=abs(true), sideways_error_m=sideways_error)


class PoseEstimator:
    def __init__(self, config: LocalizationConfig = LocalizationConfig()):
        self.cfg = config
        self.map_T_odom = None
        self.imu_offset = None
        self.odom = None
        self.odom_time = None
        self.imu_yaw = None
        self.distance = 0.0               # travelled since the last (re)calibration -> drift estimate
        self.rotation = 0.0
        self.odometer_m = 0.0             # total travelled / turned, never reset (stall check, statistics)
        self.odometer_rad = 0.0
        self.fault = None
        self.anchor = None
        self.scale_x = validate_scale(config.scale_x, 'X scale')
        self.scale_y = validate_scale(config.scale_y, 'Y scale')

    def set_scale(self, scale_x, scale_y):
        self.scale_x = validate_scale(scale_x, 'X scale')
        self.scale_y = validate_scale(scale_y, 'Y scale')

    # ------------------------------------------------------------ inputs
    def update(self, telemetry, received_at):
        """Feed one telemetry message. ``received_at`` is the local monotonic receive time."""
        odom = telemetry.get('odom') if telemetry else None
        if not odom:
            return
        t = received_at - float(odom.get('age', 0.0))
        pose = (float(odom['x']), float(odom['y']), float(odom['yaw']))
        if not all(math.isfinite(v) for v in pose):
            self.fault = 'non-finite odometry'
            return
        if self.odom is not None and self.odom_time is not None and t >= self.odom_time:
            step = math.dist(pose[:2], self.odom[:2])
            if step > self.cfg.max_jump_m:
                self.fault = 'odometry jumped %.2f m (reset or glitch); recalibrate' % step
            turn = abs(wrap(pose[2] - self.odom[2]))
            self.distance += step
            self.rotation += turn
            self.odometer_m += step
            self.odometer_rad += turn
        self.odom, self.odom_time = pose, t
        imu = telemetry.get('imu')
        self.imu_yaw = float(imu['yaw']) if imu and math.isfinite(imu.get('yaw', float('nan'))) else None

    def calibrate(self, map_pose):
        """Declare that the robot currently sits at ``map_pose`` (x, y, yaw) in the map frame."""
        if self.odom is None:
            raise ValueError('No odometry received yet; cannot calibrate.')
        self.map_T_odom = compose(map_pose, inverse(self.odom))
        self.anchor = (float(map_pose[0]), float(map_pose[1]))
        self.imu_offset = None if self.imu_yaw is None else wrap(map_pose[2] - self.imu_yaw)
        self.distance = self.rotation = 0.0
        self.fault = None

    def correct(self, map_pose):
        """The robot is at ``map_pose`` according to an external fix (LiDAR scan matching): same effect as
        ``calibrate`` (drift estimate restarts) but never clears an odometry fault."""
        if self.fault:
            raise ValueError('Odometry fault (%s): confirm the pose by hand.' % self.fault)
        fault = self.fault
        self.calibrate(map_pose)
        self.fault = fault

    # ------------------------------------------------------------ outputs
    @property
    def sigma(self):
        return self.cfg.sigma0_m + self.cfg.drift_per_m * self.distance + self.cfg.drift_per_rad_m * self.rotation

    def raw_pose(self):
        """Map-frame pose WITHOUT the scale correction (used for calibration runs)."""
        if self.map_T_odom is None or self.odom is None:
            return None
        return compose(self.map_T_odom, self.odom)

    def pose(self):
        """Map-frame (x, y, yaw) with the scale correction, or None if uncalibrated."""
        raw = self.raw_pose()
        if raw is None:
            return None
        ax, ay = self.anchor
        x = ax + self.scale_x * (raw[0] - ax)
        y = ay + self.scale_y * (raw[1] - ay)
        yaw = raw[2]
        if self.cfg.heading_source == 'imu' and self.imu_offset is not None and self.imu_yaw is not None:
            yaw = wrap(self.imu_yaw + self.imu_offset)
        return (x, y, yaw)

    def status(self, now):
        """('OK' | 'UNCALIBRATED' | 'STALE' | 'LOST' | 'LOW_CONFIDENCE', message)."""
        if self.odom is None:
            return 'STALE', 'no odometry received'
        if now - self.odom_time > self.cfg.stale_after_s:
            return 'STALE', 'odometry %.2f s old' % (now - self.odom_time)
        if self.fault:
            return 'LOST', self.fault
        if self.map_T_odom is None:
            return 'UNCALIBRATED', 'initial pose not set'
        if self.sigma > self.cfg.max_sigma_m:
            return 'LOW_CONFIDENCE', 'estimated drift %.2f m > %.2f m; recalibrate' % (self.sigma, self.cfg.max_sigma_m)
        return 'OK', 'drift estimate %.3f m' % self.sigma
