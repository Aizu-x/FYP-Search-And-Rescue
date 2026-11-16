"""Occupancy-only soft preference; this never replaces hard collision checks."""
import math
import numpy as np


def clearance_field(grid, cap):
    """Exact center-to-occupied-square/boundary distance, clipped at cap."""
    h, w = grid.shape; s = grid.resolution
    rows, cols = np.indices(grid.shape)
    distance = np.minimum.reduce([(rows+.5)*s, (h-rows-.5)*s,
                                  (cols+.5)*s, (w-cols-.5)*s,
                                  np.full(grid.shape, cap)])
    n = min(max(h, w), math.ceil(cap/s+.5))
    for dr in range(-n, n+1):
        for dc in range(-n, n+1):
            d = math.hypot(max(abs(dr)-.5, 0)*s, max(abs(dc)-.5, 0)*s)
            if d >= cap: continue
            r0, r1 = max(0, dr), min(h, h+dr)
            c0, c1 = max(0, dc), min(w, w+dc)
            if r1 <= r0 or c1 <= c0: continue
            view = distance[r0:r1, c0:c1]
            occupied = grid.cells[r0-dr:r1-dr, c0-dc:c1-dc]
            np.minimum(view, d, out=view, where=occupied)
    return distance


def penalties(grid, config):
    target = config.planning_radius + config.preferred_clearance_m
    distance = clearance_field(grid, target)
    return np.clip((target-distance)/config.preferred_clearance_m, 0, 1)**2
