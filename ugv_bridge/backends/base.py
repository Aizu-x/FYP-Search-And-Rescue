"""Hardware abstraction used by the bridge. Times are this process's time.monotonic()."""
from __future__ import annotations


class SensorSnapshot:
    """Latest sensor values; any field may be None if never received."""

    def __init__(self, odom=None, odom_time=None, imu=None, imu_time=None, scan=None, scan_time=None):
        # odom: dict(x, y, yaw, vx, vy, wz) in the robot's odometry frame (meters, radians)
        self.odom = odom
        self.odom_time = odom_time
        # imu: dict(yaw, wz)
        self.imu = imu
        self.imu_time = imu_time
        # scan: dict(angle_min, angle_increment, ranges(list), range_min, range_max) in the LiDAR frame
        self.scan = scan
        self.scan_time = scan_time


class Backend:
    """Interface the bridge requires. Implementations must be thread-safe."""

    name = 'abstract'

    def start(self):
        pass

    def shutdown(self):
        pass

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        raise NotImplementedError

    def latest(self) -> SensorSnapshot:
        raise NotImplementedError

    def latest_image(self):
        """Optional camera: (encoded_bytes, format_str, monotonic_time) or None."""
        return None

    def camera_info(self):
        """Optional camera calibration dict(width, height, k(9), d(list), frame_id) or None."""
        return None
