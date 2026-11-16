"""Compare the LiDAR scan with the occupancy map: "does the map agree with where we think we are?"

Two uses, both on the edge laptop:

* ``evaluate``  -> a live quality number for the operator (share of LiDAR points that land on mapped
  walls at the estimated pose). Odometry on the MentorPi drifts; this shows it before it matters.
* ``match``     -> search a small window of poses around the estimate for the one where the scan fits
  the map best (correlative scan matching on a likelihood field). The result is only marked ``ok``
  when the fit is good, clearly better than its surroundings (not ambiguous, e.g. not a featureless
  corridor) and not on the edge of the search window. The caller decides what to do with it:
  the operator's "Re-localize with LiDAR" button, or the optional continuous correction.

The map boundary counts as a wall (the planner treats it the same way). Objects that are not on the
map simply do not contribute.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .fast_planner import DEFAULT_CACHE
from .localization import wrap


@dataclass
class MatchResult:
    ok: bool
    reason: str
    pose: tuple                 # best pose found (x, y, yaw)
    score: float                # mean likelihood at the best pose (0..1)
    start_score: float          # ... at the pose we started from
    matched: float              # share of points within ``near_m`` of a wall at the best pose
    points: int
    shift_m: float
    turn_rad: float


class ScanMatcher:
    def __init__(self, grid, cache=None, sigma_m=0.05, near_m=0.06, min_points=40, min_range=0.15):
        self.grid = grid
        self.sigma, self.near, self.min_points, self.min_range = sigma_m, near_m, min_points, min_range
        cap = 6 * sigma_m
        self.distance = (cache or DEFAULT_CACHE).distance(grid, cap)
        self.field = np.exp(-0.5 * (self.distance / sigma_m) ** 2)

    # ------------------------------------------------------------------ helpers
    def usable(self, points):
        pts = np.asarray(points if points is not None else [], float).reshape(-1, 2)
        if len(pts):
            pts = pts[np.hypot(pts[:, 0], pts[:, 1]) >= self.min_range]
        return pts

    def _lookup(self, table, wx, wy):
        """Bilinear lookup of a per-cell table at world points; beyond the map edge the wall is the edge."""
        g = self.grid
        h, w = g.shape
        u = (wx - g.origin[0]) / g.resolution - 0.5
        v = (wy - g.origin[1]) / g.resolution - 0.5
        outside = np.maximum(np.maximum(-0.5 - u, u - (w - 0.5)), 0) + np.maximum(np.maximum(-0.5 - v, v - (h - 0.5)), 0)
        u = np.clip(u, 0, w - 1); v = np.clip(v, 0, h - 1)
        i0 = np.minimum(u.astype(int), w - 2); j0 = np.minimum(v.astype(int), h - 2)
        fu, fv = u - i0, v - j0
        value = (table[j0, i0] * (1 - fu) * (1 - fv) + table[j0, i0 + 1] * fu * (1 - fv)
                 + table[j0 + 1, i0] * (1 - fu) * fv + table[j0 + 1, i0 + 1] * fu * fv)
        return value, outside * g.resolution

    def _likelihood(self, wx, wy):
        value, beyond = self._lookup(self.field, wx, wy)
        return np.where(beyond > 0, np.exp(-0.5 * (beyond / self.sigma) ** 2), value)

    def _world(self, pose, pts):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]

    # ------------------------------------------------------------------ quality at a pose
    def evaluate(self, pose, points):
        """dict(score, matched, points) at ``pose``; None with too few LiDAR points."""
        pts = self.usable(points)
        if len(pts) < self.min_points:
            return None
        wx, wy = self._world(pose, pts)
        dist, beyond = self._lookup(self.distance, wx, wy)
        dist = np.where(beyond > 0, beyond, dist)
        return dict(score=float(self._likelihood(wx, wy).mean()), matched=float((dist <= self.near).mean()),
                    points=int(len(pts)))

    # ------------------------------------------------------------------ search
    def _scores(self, pose, pts, dxy, dyaw):
        """Score for every (yaw offset, xy offset): array [len(dyaw), len(dxy)]."""
        out = np.empty((len(dyaw), len(dxy)))
        for k, d in enumerate(dyaw):
            wx, wy = self._world((pose[0], pose[1], pose[2] + d), pts)
            out[k] = self._likelihood(wx[None, :] + dxy[:, 0:1], wy[None, :] + dxy[:, 1:2]).mean(axis=1)
        return out

    def match(self, pose, points, window_xy=0.15, window_yaw=math.radians(10.0), step_xy=0.03,
              step_yaw=math.radians(2.0), min_score=0.40, ambiguity_tol=0.03, ambiguity_xy=0.09,
              ambiguity_yaw=math.radians(6.0)):
        pts = self.usable(points)
        start = self.evaluate(pose, points)
        fail = lambda why, score=0.0: MatchResult(False, why, tuple(pose), score, (start or {}).get('score', 0.0),  # noqa: E731
                                                 (start or {}).get('matched', 0.0), len(pts), 0.0, 0.0)
        if start is None:
            return fail('too few LiDAR points (%d)' % len(pts))
        n = int(round(window_xy / step_xy))
        axis = np.arange(-n, n + 1) * step_xy
        gx, gy = np.meshgrid(axis, axis)
        dxy = np.column_stack((gx.ravel(), gy.ravel()))
        m = int(round(window_yaw / step_yaw))
        dyaw = np.arange(-m, m + 1) * step_yaw
        coarse = self._scores(pose, pts, dxy, dyaw)
        k, j = np.unravel_index(int(np.argmax(coarse)), coarse.shape)
        best = float(coarse[k, j])
        # ambiguous: clearly different poses fit (almost) as well
        near_best = np.argwhere(coarse >= best - ambiguity_tol)
        spread_xy = float(np.hypot(*(dxy[near_best[:, 1]] - dxy[j]).T).max())
        spread_yaw = float(np.abs(dyaw[near_best[:, 0]] - dyaw[k]).max())
        at_edge = (abs(dxy[j, 0]) > window_xy - 1e-9 or abs(dxy[j, 1]) > window_xy - 1e-9
                   or abs(dyaw[k]) > window_yaw - 1e-9)
        # refine around the coarse best
        centre = (pose[0] + dxy[j, 0], pose[1] + dxy[j, 1], pose[2] + dyaw[k])
        fine_axis = np.arange(-3, 4) * (step_xy / 3.0)
        fx, fy = np.meshgrid(fine_axis, fine_axis)
        fxy = np.column_stack((fx.ravel(), fy.ravel()))
        fyaw = np.arange(-4, 5) * (step_yaw / 4.0)
        fine = self._scores(centre, pts, fxy, fyaw)
        fk, fj = np.unravel_index(int(np.argmax(fine)), fine.shape)
        found = (centre[0] + fxy[fj, 0], centre[1] + fxy[fj, 1], wrap(centre[2] + fyaw[fk]))
        quality = self.evaluate(found, points)
        result = MatchResult(True, 'ok', found, quality['score'], start['score'], quality['matched'], len(pts),
                             math.dist(found[:2], pose[:2]), abs(wrap(found[2] - pose[2])))
        if quality['score'] < min_score:
            result.ok, result.reason = False, ('the scan does not fit the map well enough anywhere nearby '
                                               '(best %.0f %%)' % (100 * quality['matched']))
        elif spread_xy > ambiguity_xy or spread_yaw > ambiguity_yaw:
            result.ok, result.reason = False, ('the position is ambiguous here (walls in view do not pin it down: '
                                               'fits within %.0f cm / %.0f deg are equally good)'
                                               % (100 * spread_xy, math.degrees(spread_yaw)))
        elif at_edge:
            result.ok, result.reason = False, 'the best fit is at the edge of the search area (pose too far off)'
        return result
