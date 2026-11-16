"""Exact Euclidean segment-to-closed-square geometry, including map boundaries."""
import numpy as np

EPS = 1e-10


class CollisionChecker:
    def __init__(self, grid):
        self.grid = grid
        rows, cols = np.nonzero(grid.cells)
        s = grid.resolution; x, y = grid.origin
        self.lo = np.column_stack((x+cols*s, y+rows*s))
        self.hi = self.lo+s

    def _distances(self, a, b, lo, hi):
        """Distance between segment and each AABB: slab intersection, then
        endpoint-to-box and box-corner-to-segment distances (all exact in 2-D).
        """
        if len(lo) == 0:
            return np.empty(0)
        a = np.asarray(a, float); b = np.asarray(b, float); d = b-a
        tmin = np.zeros(len(lo)); tmax = np.ones(len(lo))
        possible = np.ones(len(lo), bool)
        for k in range(2):
            if abs(d[k]) < 1e-15:
                possible &= (a[k] >= lo[:, k]) & (a[k] <= hi[:, k])
            else:
                t1 = (lo[:, k]-a[k])/d[k]; t2 = (hi[:, k]-a[k])/d[k]
                tmin = np.maximum(tmin, np.minimum(t1, t2))
                tmax = np.minimum(tmax, np.maximum(t1, t2))
        intersects = possible & (tmin <= tmax+EPS)
        da = np.maximum(np.maximum(lo-a, a-hi), 0)
        db = np.maximum(np.maximum(lo-b, b-hi), 0)
        best = np.minimum(np.sum(da*da, axis=1), np.sum(db*db, axis=1))
        den = np.dot(d, d)
        if den > 0:
            for ix, iy in ((0, 0), (0, 1), (1, 0), (1, 1)):
                corners = np.column_stack(((lo, hi)[ix][:, 0], (lo, hi)[iy][:, 1]))
                t = np.clip(((corners-a) @ d)/den, 0, 1)
                v = corners-(a+t[:, None]*d)
                best = np.minimum(best, np.sum(v*v, axis=1))
        best[intersects] = 0
        return np.sqrt(best)

    def boundary_clearance(self, a, b):
        x0, x1, y0, y1 = self.grid.extent
        return float(min(a[0]-x0, b[0]-x0, x1-a[0], x1-b[0],
                         a[1]-y0, b[1]-y0, y1-a[1], y1-b[1]))

    def clearance(self, a, b):
        values = self._distances(a, b, self.lo, self.hi)
        return min(self.boundary_clearance(a, b), float(values.min()) if len(values) else float('inf'))

    def is_clear(self, a, b, radius=0.0):
        if self.boundary_clearance(a, b) <= radius+EPS:
            return False
        low = np.minimum(a, b)-radius-EPS; high = np.maximum(a, b)+radius+EPS
        mask = np.all(self.hi >= low, axis=1) & np.all(self.lo <= high, axis=1)
        values = self._distances(a, b, self.lo[mask], self.hi[mask])
        return not np.any(values <= radius+EPS)
