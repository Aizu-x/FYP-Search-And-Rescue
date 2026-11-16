"""Feedback path follower (runs on the edge).

Simple, predictable "rotate-then-drive" mode for a mecanum base driven like a
differential robot (vy = 0, no lateral motion yet):

1. Project the measured map pose onto the A* polyline (monotonic progress).
2. Aim at a carrot point ``lookahead_m`` further along the path.
3. Heading error > ``rotate_threshold``: turn in place (PD on heading).
   Otherwise drive forward with speed reduced by heading error, cross-track
   error and proximity to the goal, while a PD loop corrects heading.
4. Arrived when within ``goal_tolerance_m`` of the final waypoint.
5. Fails (caller must stop) on excessive cross-track error or lack of progress.

All commands are recomputed from the latest measured pose every cycle; there
is no time-based open-loop motion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .localization import wrap


@dataclass
class ControllerConfig:
    max_v: float = 0.12               # m/s
    min_v: float = 0.03               # m/s, below this the motors may stall
    max_w: float = 0.6                # rad/s
    min_w: float = 0.15               # rad/s while turning in place (deadband)
    kp_w: float = 1.5
    kd_w: float = 0.1
    lookahead_m: float = 0.15
    rotate_threshold_deg: float = 35.0
    align_first_deg: float = 35.0     # = rotate_threshold_deg: original behaviour (drive and steer together)
    goal_tolerance_m: float = 0.05
    slow_radius_m: float = 0.25
    max_cross_track_m: float = 0.10
    progress_timeout_s: float = 8.0
    progress_min_m: float = 0.02
    progress_min_deg: float = 5.0


@dataclass
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    status: str = 'tracking'          # tracking | rotating | arrived | failed
    message: str = ''
    cross_track: float = 0.0
    heading_error: float = 0.0
    remaining: float = 0.0


def _project(path, cum, point, start_seg, limit_s):
    """Closest point on segments from start_seg whose start lies before arc length ``limit_s``.
    The window stops the projection jumping to a later part of a looping path."""
    best = (float('inf'), start_seg, 0.0)
    for k in range(start_seg, len(path) - 1):
        if k > start_seg and cum[k] > limit_s:
            break
        a, b = path[k], path[k + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        t = 0.0 if den == 0 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / den))
        d = math.hypot(a[0] + t * dx - point[0], a[1] + t * dy - point[1])
        if d < best[0] - 1e-12:
            best = (d, k, t)
    d, k, t = best
    return d, k, cum[k] + t * (cum[k + 1] - cum[k])


def _point_at(path, cum, s):
    s = max(0.0, min(cum[-1], s))
    for k in range(len(path) - 1):
        if cum[k + 1] >= s:
            seg = cum[k + 1] - cum[k]
            t = 0.0 if seg == 0 else (s - cum[k]) / seg
            a, b = path[k], path[k + 1]
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
    return path[-1]


class PathTracker:
    def __init__(self, path, config: ControllerConfig = ControllerConfig()):
        if not path:
            raise ValueError('Empty path.')
        self.cfg = config
        self.path = [tuple(p) for p in path]
        if len(self.path) == 1:
            self.path.append(self.path[0])
        self.cum = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.segment = 0
        self.s = 0.0
        self.prev_error = None
        self.best_remaining = float('inf')
        self.best_heading = float('inf')
        self.last_progress_time = None
        self.was_rotating = False
        self.aligned = False              # start of route: face the route before driving (no swerving into walls)

    def update(self, pose, now, dt):
        c = self.cfg
        x, y, yaw = pose
        goal = self.path[-1]
        to_goal = math.hypot(goal[0] - x, goal[1] - y)
        cross, self.segment, s = _project(self.path, self.cum, (x, y), self.segment,
                                          self.s + c.lookahead_m + 2 * c.max_cross_track_m + 0.1)
        self.s = max(self.s, s)
        remaining = (self.cum[-1] - self.s) + cross
        if to_goal <= c.goal_tolerance_m:
            return Command(status='arrived', message='within %.3f m of goal' % to_goal, remaining=to_goal)
        if cross > c.max_cross_track_m:
            return Command(status='failed', message='cross-track error %.3f m > %.3f m' % (cross, c.max_cross_track_m),
                           cross_track=cross, remaining=remaining)
        carrot = _point_at(self.path, self.cum, self.s + c.lookahead_m)
        if math.dist(carrot, (x, y)) < 1e-3:
            carrot = goal
        error = wrap(math.atan2(carrot[1] - y, carrot[0] - x) - yaw)
        derivative = 0.0 if self.prev_error is None or dt <= 0 else wrap(error - self.prev_error) / dt
        self.prev_error = error
        w = max(-c.max_w, min(c.max_w, c.kp_w * error + c.kd_w * derivative))

        # Progress monitor: remaining path length must shrink, or, while turning
        # in place, the heading error must shrink.
        threshold = c.rotate_threshold_deg if self.aligned else min(c.align_first_deg, c.rotate_threshold_deg)
        rotating = abs(error) > math.radians(threshold)
        if not rotating:
            self.aligned = True
        if self.last_progress_time is None:
            self.last_progress_time = now
        improved = False
        if remaining < self.best_remaining - c.progress_min_m:
            self.best_remaining = remaining; improved = True
        if rotating:
            if not self.was_rotating:
                self.best_heading = abs(error)
            elif abs(error) < self.best_heading - math.radians(c.progress_min_deg):
                self.best_heading = abs(error); improved = True
        self.was_rotating = rotating
        if improved:
            self.last_progress_time = now
        elif now - self.last_progress_time > c.progress_timeout_s:
            return Command(status='failed', message='no progress for %.1f s' % (now - self.last_progress_time),
                           cross_track=cross, heading_error=error, remaining=remaining)

        if rotating:
            if abs(w) < c.min_w:
                w = math.copysign(c.min_w, error)
            return Command(0.0, 0.0, w, 'rotating', 'aligning heading', cross, error, remaining)
        v = c.max_v * math.cos(error) * max(0.3, 1.0 - cross / c.max_cross_track_m)
        v = min(v, c.max_v * max(to_goal / c.slow_radius_m, 0.0))
        v = max(c.min_v, min(c.max_v, v))
        return Command(v, 0.0, w, 'tracking', '', cross, error, remaining)
