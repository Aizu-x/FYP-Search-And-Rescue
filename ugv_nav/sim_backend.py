"""Simulated MentorPi for closed-loop software testing (no hardware).

Implements the bridge ``Backend`` interface, so the real bridge server,
protocol, watchdog and edge client are all exercised exactly as on the robot.

* Kinematics: body-frame (vx, vy, wz) with first-order velocity lag, 100 Hz.
* Odometry: starts at (0, 0, 0) wherever the robot is placed (like the real
  robot), with configurable scale error and yaw drift so map alignment and
  drift handling are tested honestly.
* LiDAR: 360 rays ray-marched through a "world" grid (e.g. the GT map), plus
  optional circular obstacles injected at run time. With ``lidar_world`` the LiDAR
  looks OVER the arena walls (like a MentorPi whose LiDAR sits higher than the
  arena) and only sees that other grid, e.g. the room around the arena
  (``simulated_room``). ``lidar_mount`` puts the sensor off-centre / turned on the
  robot, ``scan_noise_m`` adds range noise.
* Ground-truth collision detection against the world grid for reporting only.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from ugv_bridge.backends.base import Backend, SensorSnapshot
from ugv_planner.collision import CollisionChecker

from .localization import compose, inverse


class SimBackend(Backend):
    name = 'simulation'

    def __init__(self, world_grid, start_pose=(0.5, 0.5, 0.0), body_radius=0.1, tau=0.1,
                 odom_scale_error=0.0, yaw_drift_rad_per_s=0.0, rays=360, max_range=3.0, rate_hz=100.0,
                 clock=time.monotonic, realtime=True, lidar_world=None, lidar_mount=(0.0, 0.0, 0.0),
                 scan_noise_m=0.0, seed=7):
        self.world = world_grid
        self.lidar_world = lidar_world        # None: the LiDAR sees the arena walls of ``world_grid``
        self.lidar_mount = tuple(float(v) for v in lidar_mount)   # true sensor pose on the robot (x, y, yaw)
        self.scan_noise = float(scan_noise_m)
        self._rng = np.random.default_rng(seed)
        self.checker = CollisionChecker(world_grid)
        self.body_radius = body_radius
        self.tau, self.scale_err, self.yaw_drift = tau, odom_scale_error, yaw_drift_rad_per_s
        self.rays, self.max_range, self.dt = rays, max_range, 1.0 / rate_hz
        self.clock = clock
        self.realtime = realtime
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.obstacles = []  # (x, y, r) -- seen by the simulated LiDAR
        self.blocks = None   # vision_sim.SimBlocks: low WRO blocks, below the LiDAR plane (camera only)
        self.camera_source = None
        self.cmd = (0.0, 0.0, 0.0)
        self.collided = False
        self.truth_history = []
        self.place(start_pose)

    # ---------------------------------------------------------- scenario control
    def place(self, pose):
        """Physically put the robot somewhere. Odometry keeps running from its current value."""
        with self._lock:
            self.truth = tuple(pose)
            self.vel = (0.0, 0.0, 0.0)
            if not hasattr(self, 'odom'):
                self.odom = (0.0, 0.0, 0.0)
            # odom_origin_in_map such that odom_origin * odom == truth
            self.odom_origin = compose(self.truth, inverse(self.odom))
            self.truth_history = [self.truth]

    def add_obstacle(self, x, y, r):
        with self._lock:
            self.obstacles.append((x, y, r))

    # ---------------------------------------------------------- Backend
    def start(self):
        if self.realtime:
            self._thread = threading.Thread(target=self._loop, name='sim-physics', daemon=True)
            self._thread.start()

    def shutdown(self):
        self._stop.set()

    def send_velocity(self, vx, vy, wz):
        with self._lock:
            self.cmd = (float(vx), float(vy), float(wz))

    def latest_image(self):
        return self.camera_source.latest_image() if self.camera_source is not None else None

    def camera_info(self):
        return self.camera_source.camera_info() if self.camera_source is not None else None

    def latest(self):
        now = self.clock()
        with self._lock:
            ox, oy, oyaw = self.odom
            vx, vy, wz = self.vel
            scan = self._scan()
        return SensorSnapshot(odom=dict(x=ox, y=oy, yaw=oyaw, vx=vx, vy=vy, wz=wz), odom_time=now,
                              imu=dict(yaw=oyaw, wz=wz), imu_time=now, scan=scan, scan_time=now)

    # ---------------------------------------------------------- physics
    def _loop(self):
        while not self._stop.is_set():
            time.sleep(self.dt)
            self.advance(self.dt)

    def advance(self, dt):
        with self._lock:
            a = 1.0 - math.exp(-dt / self.tau)
            self.vel = tuple(v + a * (c - v) for v, c in zip(self.vel, self.cmd))
            vx, vy, wz = self.vel
            x, y, yaw = self.truth
            step = (vx * dt, vy * dt, wz * dt)
            new = compose(self.truth, step)
            if not self.checker.is_clear((x, y), new[:2], self.body_radius) or self._hits_obstacle(new):
                self.collided = True
                self.vel = (0.0, 0.0, 0.0)
                return
            self.truth = new
            if math.dist(self.truth_history[-1][:2], new[:2]) > 0.005 or abs(wz) > 0:
                self.truth_history.append(new)
                del self.truth_history[:-20000]
            # odometry: scaled translation and drifting yaw
            ostep = (step[0] * (1 + self.scale_err), step[1] * (1 + self.scale_err), step[2] + self.yaw_drift * dt)
            self.odom = compose(self.odom, ostep)

    def _hits_obstacle(self, pose):
        if any(math.hypot(pose[0] - ox, pose[1] - oy) <= r + self.body_radius for ox, oy, r in self.obstacles):
            return True
        if self.blocks is not None:
            return any(math.hypot(pose[0] - bx, pose[1] - by) <= 0.035 + self.body_radius
                       for _, (bx, by, _c) in self.blocks.items())
        return False

    def _scan(self):
        x, y, yaw = self.truth
        mx, my, myaw = self.lidar_mount
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, yaw = x + c * mx - s * my, y + s * mx + c * my, yaw + myaw      # where the sensor really is
        world = self.lidar_world if self.lidar_world is not None else self.world
        angles = -math.pi + np.arange(self.rays) * (2 * math.pi / self.rays)
        res = world.resolution
        steps = np.arange(0.06, self.max_range, res / 2)
        dirs = yaw + angles
        px = x + np.outer(np.cos(dirs), steps)
        py = y + np.outer(np.sin(dirs), steps)
        cols = np.floor((px - world.origin[0]) / res).astype(int)
        rows = np.floor((py - world.origin[1]) / res).astype(int)
        h, w = world.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        # arena: beyond the map is wall. Room: beyond the room grid there is nothing to see.
        hit = ~inside if self.lidar_world is None else np.zeros(inside.shape, bool)
        hit[inside] = world.cells[rows[inside], cols[inside]]
        for ox, oy, r in self.obstacles:
            hit |= (px - ox) ** 2 + (py - oy) ** 2 <= r * r
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, steps[np.maximum(first, 0)], float('inf'))
        if self.scan_noise:
            ranges = ranges + self._rng.normal(0.0, self.scan_noise, len(ranges))
        return dict(angle_min=-math.pi, angle_increment=2 * math.pi / self.rays,
                    ranges=[float(r) for r in ranges], range_min=0.05, range_max=self.max_range)

    # ---------------------------------------------------------- reporting
    def truth_pose(self):
        with self._lock:
            return self.truth


def simulated_room(arena_grid, left=0.60, right=0.95, bottom=0.45, top=0.75, furniture=True):
    """A room around the arena for a LiDAR that looks over the arena walls: four room walls (with an open
    door) and a few pieces of furniture OUTSIDE the arena. Returns a Grid with its own origin; pass it as
    ``lidar_world``. The arena itself is empty in it (its walls and blocks are lower than the LiDAR)."""
    from ugv_planner.grid import Grid
    res = arena_grid.resolution
    ax0, ax1, ay0, ay1 = arena_grid.extent
    n = lambda metres: int(round(metres / res))        # noqa: E731
    cl, cr, cb, ct = n(left), n(right), n(bottom), n(top)
    h, w = arena_grid.shape
    cells = np.zeros((h + cb + ct, w + cl + cr), np.uint8)
    cells[:2, :] = cells[-2:, :] = 1
    cells[:, :2] = cells[:, -2:] = 1
    door = slice(cl + n(1.9), cl + n(2.7))
    cells[:2, door] = 0                                # open door in the bottom wall
    if furniture:
        H, W = cells.shape
        cells[H - n(0.55):H - 2, 2:n(1.5)] = 1                         # bed along the top wall, left
        cells[cb + n(0.9):cb + n(2.2), W - n(0.5):W - 2] = 1           # wardrobe on the right wall
        cells[2:n(0.3), cl + n(0.3):cl + n(0.9)] = 1                   # shelf on the bottom wall
        for r0, c0 in ((cb + n(0.5), n(0.15)), (cb + n(1.1), n(0.15)), (cb + n(0.5), n(0.4)), (cb + n(1.1), n(0.4))):
            cells[r0:r0 + 2, c0:c0 + 2] = 1                            # desk legs on the left
        cells[H - n(0.35):H - n(0.2), W - n(0.7):W - n(0.55)] = 1      # a bin in the top-right corner
    return Grid(cells, res, (ax0 - cl * res, ay0 - cb * res))
