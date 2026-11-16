"""Turn an existing ``ugv_planner`` A* result into a validated navigation path."""
from __future__ import annotations

import math

from ugv_planner.collision import CollisionChecker


def remove_collinear(points, tol=1e-9):
    """Drop intermediate points on straight runs. Geometry is unchanged."""
    if len(points) <= 2:
        return [tuple(p) for p in points]
    out = [tuple(points[0])]
    for prev, cur, nxt in zip(points, points[1:], points[2:]):
        cross = (cur[0] - prev[0]) * (nxt[1] - cur[1]) - (cur[1] - prev[1]) * (nxt[0] - cur[0])
        if abs(cross) > tol:
            out.append(tuple(cur))
    out.append(tuple(points[-1]))
    return out


def shortcut(points, grid, radius):
    """Optional greedy line-of-sight smoothing. Every new segment passes the exact
    swept-disk check against the occupancy grid with ``radius``."""
    if len(points) <= 2:
        return list(points)
    checker = CollisionChecker(grid)
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not checker.is_clear(points[i], points[j], radius):
            j -= 1
        out.append(points[j])
        i = j
    return out


def validate_path(points, grid, radius):
    """Return (ok, message). Every segment must clear occupancy by more than ``radius``."""
    if not points:
        return False, 'empty path'
    if any(not (math.isfinite(p[0]) and math.isfinite(p[1])) for p in points):
        return False, 'non-finite waypoint'
    checker = CollisionChecker(grid)
    segments = list(zip(points, points[1:])) or [(points[0], points[0])]
    for k, (a, b) in enumerate(segments):
        if not checker.is_clear(a, b, radius):
            return False, 'segment %d (%.2f,%.2f)->(%.2f,%.2f) violates %.3f m clearance' % (k, a[0], a[1], b[0], b[1], radius)
    return True, 'path clear of occupancy with %.3f m radius' % radius


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))
