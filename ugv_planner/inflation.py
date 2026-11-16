"""Cell-center disk inflation, consistent with continuous collision geometry."""
import math
import numpy as np
from .collision import EPS


def inflated_mask(grid, radius):
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('Inflation radius must be finite and nonnegative.')
    a = grid.cells; h, w = a.shape; s = grid.resolution
    blocked = a.copy()
    n = min(max(h, w), math.ceil(radius/s+0.5))
    for dr in range(-n, n+1):
        for dc in range(-n, n+1):
            if math.hypot(max(abs(dr)-0.5, 0)*s, max(abs(dc)-0.5, 0)*s) > radius+EPS:
                continue
            r0, r1 = max(0, dr), min(h, h+dr)
            c0, c1 = max(0, dc), min(w, w+dc)
            if r1 > r0 and c1 > c0:
                blocked[r0:r1, c0:c1] |= a[r0-dr:r1-dr, c0-dc:c1-dc]
    rows, cols = np.indices(a.shape)
    edge_distance = np.minimum.reduce([(rows+.5)*s, (h-rows-.5)*s,
                                       (cols+.5)*s, (w-cols-.5)*s])
    blocked |= edge_distance <= radius+EPS
    return blocked
