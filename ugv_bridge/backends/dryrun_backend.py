"""Hardware-free backend for communication-only testing (Test B) on the Pi or laptop.

It never touches motors. It reports a static odometry pose and, optionally, a
synthetic clear LiDAR scan so the full arm/command/watchdog path can be
exercised. Every velocity it receives is recorded for inspection.
"""
from __future__ import annotations

import math
import threading
import time

from .base import Backend, SensorSnapshot


class DryRunBackend(Backend):
    name = 'dryrun'

    def __init__(self, provide_scan=True, clear_range=2.0):
        self._lock = threading.Lock()
        self.provide_scan = provide_scan
        self.clear_range = clear_range
        self.commands = []  # (monotonic time, vx, vy, wz)

    def send_velocity(self, vx, vy, wz):
        with self._lock:
            self.commands.append((time.monotonic(), vx, vy, wz))
            del self.commands[:-2000]

    def last_command(self):
        with self._lock:
            return self.commands[-1] if self.commands else None

    image = None          # tests may set (bytes, format, time)
    info = None

    def latest_image(self):
        return self.image

    def camera_info(self):
        return self.info

    def latest(self):
        now = time.monotonic()
        scan = None
        if self.provide_scan:
            n = 360
            scan = dict(angle_min=-math.pi, angle_increment=2 * math.pi / n,
                        ranges=[self.clear_range] * n, range_min=0.05, range_max=8.0)
        return SensorSnapshot(odom=dict(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0), odom_time=now,
                              imu=dict(yaw=0.0, wz=0.0), imu_time=now,
                              scan=scan, scan_time=now if scan else None)
