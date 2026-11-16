"""Straighten the staircase of a grid A* route without making it worse.

A* on an 8-connected grid can only move in 45 degree steps, so its routes zig-zag. Here a part of
the route is replaced by a straight line a -> b only if

1. the robot's swept disk along a -> b is clear with the full planning radius (the same exact
   ``CollisionChecker`` the planner and the navigator use), and
2. the planner's OWN objective -- length x (1 + clearance_weight x clearance penalty) -- integrated
   along the straight line is not larger than along the part it replaces.

With clearance weight 0 this is classic any-angle shortening. With the clearance preference the
route keeps its distance from walls: a shortcut that hugs a wall costs more and is refused.
The first point (the robot's exact pose) and the last point (the goal) are never moved, and the
result is re-validated segment by segment; on any doubt the original route is returned.
"""
from __future__ import annotations

import math
import time

import numpy as np

from ugv_planner.collision import EPS

from .fast_planner import DEFAULT_CACHE
from .path_processing import path_length, remove_collinear

_HALF_DIAGONAL = math.sqrt(0.5)       # x cell size: a point in a cell is at most this far from its centre


class _Evaluator:
    """Cost and clearance of straight segments on one map (vectorised sampling every ``ds`` metres)."""

    def __init__(self, grid, config, cache, ds=0.01):
        self.grid, self.ds = grid, ds
        self.radius = config.planning_radius
        self.weight = float(config.clearance_weight)
        self.checker = cache.checker(grid)
        fields = cache.fields(grid, config)
        self.penalty = fields.penalty if self.weight else None
        self.slack = _HALF_DIAGONAL * grid.resolution + ds / 2 + 1e-6
        self.distance = cache.distance(grid, self.radius + self.slack + 0.05)
        self.exact_checks = 0

    def _samples(self, a, b):
        length = math.dist(a, b)
        n = max(1, int(math.ceil(length / self.ds)))
        t = np.arange(n + 1) / n
        return length, n, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t

    def _penalty_at(self, xs, ys):
        g = self.grid
        h, w = g.shape
        u = np.clip((xs - g.origin[0]) / g.resolution - 0.5, 0, w - 1)
        v = np.clip((ys - g.origin[1]) / g.resolution - 0.5, 0, h - 1)
        i0 = np.minimum(u.astype(int), w - 2)
        j0 = np.minimum(v.astype(int), h - 2)
        fu, fv = u - i0, v - j0
        p = self.penalty
        return (p[j0, i0] * (1 - fu) * (1 - fv) + p[j0, i0 + 1] * fu * (1 - fv)
                + p[j0 + 1, i0] * (1 - fu) * fv + p[j0 + 1, i0 + 1] * fu * fv)

    def cost(self, a, b):
        length, n, xs, ys = self._samples(a, b)
        if self.penalty is None or length == 0:
            return length
        p = self._penalty_at(xs, ys)
        return length * (1 + self.weight * float(p.sum() - (p[0] + p[-1]) / 2) / n)

    def clear(self, a, b):
        """Exact answer; a cheap conservative test on the cached distance field avoids most exact checks."""
        g = self.grid
        h, w = g.shape
        _, _, xs, ys = self._samples(a, b)
        cols = np.floor((xs - g.origin[0]) / g.resolution).astype(int)
        rows = np.floor((ys - g.origin[1]) / g.resolution).astype(int)
        if cols.min() < 0 or rows.min() < 0 or cols.max() >= w or rows.max() >= h:
            return False
        nearest = float(self.distance[rows, cols].min())
        if nearest - self.slack > self.radius + EPS:
            return True                          # every point of the segment is certainly further than R
        if nearest + self.slack < self.radius:
            return False                         # some sample is certainly closer than R
        self.exact_checks += 1
        return self.checker.is_clear(a, b, self.radius)


def _densify(points, spacing):
    out = [tuple(points[0])]
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / spacing)))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out


def _pull(points, ev, spacing, window, misses):
    dense = _densify(points, spacing)
    n = len(dense)
    prefix = [0.0]
    for a, b in zip(dense, dense[1:]):
        prefix.append(prefix[-1] + ev.cost(a, b))
    out = [dense[0]]
    i = 0
    while i < n - 1:
        best, miss, k = i + 1, 0, i + 2
        while k < n and k - i <= window and miss < misses:
            a, b = dense[i], dense[k]
            if ev.cost(a, b) <= prefix[k] - prefix[i] + 1e-9 and ev.clear(a, b):
                best, miss = k, 0
            else:
                miss += 1
            k += 1
        out.append(dense[best])
        i = best
    return remove_collinear(out, tol=1e-7)


def smooth_route(points, grid, config, cache=None, spacing=0.06, window=90, misses=8, passes=3):
    """Return (smoothed points, info dict). ``points`` is the A* route (metres), first = robot pose."""
    began = time.perf_counter()
    cache = cache or DEFAULT_CACHE
    original = [tuple(float(v) for v in p) for p in points]
    info = dict(smoothed=False, raw_length_m=path_length(original), length_m=path_length(original),
                raw_points=len(original), points=len(original), passes=0, ms=0.0)
    if len(original) < 3:
        return original, info
    ev = _Evaluator(grid, config, cache)
    current = original
    for _ in range(passes):
        pulled = _pull(current, ev, spacing, window, misses)
        info['passes'] += 1
        gained = path_length(current) - path_length(pulled)
        if len(pulled) >= len(current) and gained < 1e-3:
            break
        current = pulled
        if gained < 1e-3:
            break
    # Final guarantee: every segment passes the exact swept-disk check; ends untouched.
    ok = (current[0] == original[0] and current[-1] == original[-1]
          and all(ev.checker.is_clear(a, b, ev.radius) for a, b in zip(current, current[1:])))
    if ok and current != original:
        info.update(smoothed=True, length_m=path_length(current), points=len(current))
    else:
        current = original
    info['ms'] = (time.perf_counter() - began) * 1000
    info['exact_checks'] = ev.exact_checks
    return current, info
