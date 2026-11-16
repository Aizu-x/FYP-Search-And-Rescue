"""Edge-side safety checks evaluated every control cycle before any command is sent.

The UGV bridge has its own independent watchdog and LiDAR stop; these checks
are a second layer that also covers things only the edge knows (map validity,
localization confidence, operator STOP).
"""
from __future__ import annotations

from dataclasses import dataclass

import math

from ugv_bridge.safety import obstacle_blocking, stopping_distance

from ugv_planner.collision import CollisionChecker


@dataclass
class SafetyConfig:
    half_length_m: float | None = None    # MEASURE (same values as the bridge config)
    half_width_m: float | None = None
    stop_distance_m: float = 0.12         # zone margin; zone = margin + v*reaction + v^2/(2*decel)
    reaction_time_s: float = 0.4          # edge reacts later than the bridge (network + 10 Hz loop)
    brake_decel: float = 0.3
    rotate_clearance_m: float = 0.02      # must satisfy hypot(half_l, half_w) + this <= planning radius
    ignore_known_obstacles: bool = True   # walls already in the map are handled by A* clearance;
    known_tolerance_cells: float = 2.0    # a return within sigma + this many cells of a mapped wall is "known"
    scan_stale_s: float = 0.6
    require_lidar: bool = True


class SafetyMonitor:
    def __init__(self, config: SafetyConfig):
        self.cfg = config
        self.estop = False
        self.lidar_mount = (0.0, 0.0, 0.0)    # extra LiDAR mounting (x, y, yaw) learnt on the edge (room_map)

    def dimensions_known(self):
        return bool(self.cfg.half_length_m and self.cfg.half_width_m)

    def lidar_points(self, telemetry):
        scan = telemetry.get('scan') if telemetry else None
        if scan is None:
            return None
        mx, my, yaw = self.lidar_mount
        if mx == 0.0 and my == 0.0 and yaw == 0.0:
            return [tuple(p) for p in scan.get('points', [])]
        c, s = math.cos(yaw), math.sin(yaw)
        return [(mx + c * p[0] - s * p[1], my + s * p[0] + c * p[1]) for p in scan.get('points', [])]

    def unknown_points(self, points, pose, grid, sigma):
        """Robot-frame LiDAR points that do NOT correspond to an occupied cell of the map.

        The UGV bridge never applies this filter: its own stop zone sees every return.
        """
        if not self.cfg.ignore_known_obstacles or pose is None or grid is None:
            return points
        c, s = math.cos(pose[2]), math.sin(pose[2])
        n = int(math.ceil(sigma / grid.resolution + self.cfg.known_tolerance_cells))
        h, w = grid.shape
        out = []
        for px, py in points:
            r, col = grid.world_to_cell((pose[0] + c * px - s * py, pose[1] + s * px + c * py))
            if not (0 <= r < h and 0 <= col < w):
                continue  # beyond the mapped arena (outer boundary is blocked anyway)
            if not grid.cells[max(0, r - n):r + n + 1, max(0, col - n):col + n + 1].any():
                out.append((px, py))
        return out

    def check(self, link, telemetry, localization_status, intended=None, pose=None, grid=None, sigma=0.0):
        """Return a list of blocking reasons (empty list = safe to command ``intended``).

        ``intended`` is the (vx, vy, wz) the controller wants to send. With
        ``pose``/``grid`` the LiDAR check only reacts to obstacles that are not in the map.
        """
        reasons = []
        if self.estop:
            reasons.append('operator STOP latched')
        if not link:
            reasons.append('UGV link lost or stale')
        if localization_status[0] != 'OK':
            reasons.append('localization %s: %s' % localization_status)
        if not self.dimensions_known():
            reasons.append('robot dimensions not configured')
        if telemetry:
            if telemetry.get('safety_stop'):
                reasons.append('UGV safety stop: %s' % telemetry['safety_stop'])
            scan = telemetry.get('scan')
            if scan is None or scan.get('age', 1e9) > self.cfg.scan_stale_s:
                if self.cfg.require_lidar:
                    reasons.append('LiDAR stale or missing')
            elif intended is not None and self.dimensions_known():
                points = self.unknown_points(self.lidar_points(telemetry), pose, grid, sigma)
                zone = stopping_distance(math.hypot(intended[0], intended[1]), self.cfg.stop_distance_m,
                                         self.cfg.reaction_time_s, self.cfg.brake_decel)
                reason = obstacle_blocking(points, *intended, self.cfg.half_length_m,
                                           self.cfg.half_width_m, zone, self.cfg.rotate_clearance_m)
                if reason:
                    reasons.append('LiDAR: ' + reason)
        return reasons


def remaining_path_clear(grid, path, from_segment, radius):
    """True if every remaining path segment still clears the (possibly updated) map."""
    checker = CollisionChecker(grid)
    for a, b in zip(path[from_segment:], path[from_segment + 1:]):
        if not checker.is_clear(a, b, radius):
            return False
    return True
