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

* ``locate``    -> search the WHOLE map (every candidate position, every heading) for the pose where the
  scan fits best: "where is the robot?" without any starting guess. Only ``ok`` when one pose is clearly
  better than every other one; otherwise the best candidates are returned for the operator to choose.

The same matcher also works on a map the LiDAR drew itself (``ugv_nav.room_map``): there the edge of the
map is NOT a wall, and LiDAR points that land where nothing has been observed yet are neutral.
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


@dataclass
class LocateResult:
    ok: bool
    reason: str
    pose: tuple                 # best pose (x, y, yaw) or None
    score: float
    matched: float
    points: int
    candidates: list            # [dict(pose, score, matched)] best first (several when ambiguous)


class ScanMatcher:
    def __init__(self, grid=None, cache=None, sigma_m=0.05, near_m=0.06, min_points=40, min_range=0.15,
                 distance=None, origin=None, resolution=None, known=None, edge_is_wall=True, min_known=0.4):
        """Either ``grid`` (the operator's occupancy map: its edge is a wall) or a ready ``distance`` field
        (metres to the nearest wall per cell, clipped) with ``origin`` / ``resolution`` and optionally
        ``known`` (cells that were observed at all)."""
        self.grid = grid
        self.sigma, self.near, self.min_points, self.min_range = sigma_m, near_m, min_points, min_range
        if distance is None:
            distance = (cache or DEFAULT_CACHE).distance(grid, 6 * sigma_m)
            origin, resolution = grid.origin, grid.resolution
        self.distance = distance
        self.shape, self.origin, self.res = distance.shape, tuple(origin), float(resolution)
        self.field = np.exp(-0.5 * (self.distance / sigma_m) ** 2)
        self.known = None if known is None else np.asarray(known, bool)
        self.edge_is_wall = edge_is_wall
        self.min_known = min_known          # at least this share of the scan must land on observed floor / walls
        self._coarse = {}
        self._walls = None

    # ------------------------------------------------------------------ helpers
    def usable(self, points):
        pts = np.asarray(points if points is not None else [], float).reshape(-1, 2)
        if len(pts):
            pts = pts[np.hypot(pts[:, 0], pts[:, 1]) >= self.min_range]
        return pts

    def _lookup(self, table, wx, wy):
        """Bilinear lookup of a per-cell table at world points; also how far beyond the map edge each point is."""
        h, w = self.shape
        u = (wx - self.origin[0]) / self.res - 0.5
        v = (wy - self.origin[1]) / self.res - 0.5
        outside = np.maximum(np.maximum(-0.5 - u, u - (w - 0.5)), 0) + np.maximum(np.maximum(-0.5 - v, v - (h - 0.5)), 0)
        u = np.clip(u, 0, w - 1); v = np.clip(v, 0, h - 1)
        i0 = np.minimum(u.astype(int), w - 2); j0 = np.minimum(v.astype(int), h - 2)
        fu, fv = u - i0, v - j0
        value = (table[j0, i0] * (1 - fu) * (1 - fv) + table[j0, i0 + 1] * fu * (1 - fv)
                 + table[j0 + 1, i0] * (1 - fu) * fv + table[j0 + 1, i0 + 1] * fu * fv)
        return value, outside * self.res

    def _cells(self, wx, wy):
        """Nearest cell of world points: (rows, columns, inside the map?)."""
        h, w = self.shape
        c = np.floor((wx - self.origin[0]) / self.res).astype(int)
        r = np.floor((wy - self.origin[1]) / self.res).astype(int)
        inside = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        return np.clip(r, 0, h - 1), np.clip(c, 0, w - 1), inside

    def _weights(self, wx, wy):
        """1 for points that count, 0 for points on never-observed ground (None = every point counts)."""
        if self.known is None and self.edge_is_wall:
            return None
        r, c, inside = self._cells(wx, wy)
        if self.known is None:
            return inside.astype(float)
        return (self.known[r, c] & inside).astype(float)

    def _likelihood(self, wx, wy):
        value, beyond = self._lookup(self.field, wx, wy)
        if self.edge_is_wall:               # the operator's map: its edge is the arena wall
            return np.where(beyond > 0, np.exp(-0.5 * (beyond / self.sigma) ** 2), value)
        return np.where(beyond > 0, 0.0, value)

    def _mean(self, values, weights):
        """Mean over the points that count (last axis); too few of them count as a poor fit, not a good one."""
        if weights is None:
            return values.mean(axis=-1)
        floor = max(self.min_known * values.shape[-1], 1.0)
        return (values * weights).sum(axis=-1) / np.maximum(weights.sum(axis=-1), floor)

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
        dist = np.where(beyond > 0, beyond if self.edge_is_wall else 9.0, dist)
        weights = self._weights(wx, wy)
        known = 1.0 if weights is None else float(weights.mean())
        return dict(score=float(self._mean(self._likelihood(wx, wy), weights)),
                    matched=float(self._mean((dist <= self.near).astype(float), weights)),
                    points=int(len(pts)), known=known)

    # ------------------------------------------------------------------ search
    def _scores(self, pose, pts, dxy, dyaw):
        """Score for every (yaw offset, xy offset): array [len(dyaw), len(dxy)]."""
        out = np.empty((len(dyaw), len(dxy)))
        for k, d in enumerate(dyaw):
            wx, wy = self._world((pose[0], pose[1], pose[2] + d), pts)
            wx, wy = wx[None, :] + dxy[:, 0:1], wy[None, :] + dxy[:, 1:2]
            out[k] = self._mean(self._likelihood(wx, wy), self._weights(wx, wy))
        return out

    def match(self, pose, points, window_xy=0.15, window_yaw=math.radians(10.0), step_xy=0.03,
              step_yaw=math.radians(2.0), min_score=0.40, ambiguity_tol=0.03, ambiguity_xy=0.09,
              ambiguity_yaw=math.radians(6.0), precise=False):
        """``precise``: one more, three times finer search step (about 3 mm / 0.17 deg) around the result."""
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
        if len(near_best):
            spread_xy = float(np.hypot(*(dxy[near_best[:, 1]] - dxy[j]).T).max())
            spread_yaw = float(np.abs(dyaw[near_best[:, 0]] - dyaw[k]).max())
        else:                                   # ambiguity test switched off by the caller
            spread_xy = spread_yaw = 0.0
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
        if precise:
            pxy, pyaw = fxy / 3.0, fyaw[2:7] / 3.0
            finer = self._scores(found, pts, pxy, pyaw)
            pk, pj = np.unravel_index(int(np.argmax(finer)), finer.shape)
            found = (found[0] + pxy[pj, 0], found[1] + pxy[pj, 1], wrap(found[2] + pyaw[pk]))
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

    # ------------------------------------------------------------------ search everywhere
    def crossing(self, pose, points, sensor=(0.0, 0.0), clear_m=0.15):
        """Share of the LiDAR rays that would pass THROUGH a wall of the map before they end, if the robot stood
        at ``pose``. Light does not do that: at the true pose this is (nearly) zero, at a wrong place that
        merely looks similar it is large."""
        pts = self.usable(points)
        if not len(pts):
            return 0.0
        if self._walls is None:
            self._walls = self.distance <= 1e-9
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        ox = pose[0] + c * sensor[0] - s_ * sensor[1]
        oy = pose[1] + s_ * sensor[0] + c * sensor[1]
        wx, wy = self._world(pose, pts)
        dx, dy = wx - ox, wy - oy
        length = np.maximum(np.hypot(dx, dy), 1e-6)
        along = np.arange(clear_m, float(length.max()), 0.7 * self.res)[None, :]
        if not along.size:
            return 0.0
        before_end = along < (length[:, None] - clear_m)
        r, col, inside = self._cells(ox + (dx / length)[:, None] * along, oy + (dy / length)[:, None] * along)
        through = self._walls[r, col] & inside & before_end
        return float(through.any(axis=1).mean())

    def _coarse_field(self, sigma):
        if sigma not in self._coarse:
            self._coarse[sigma] = np.exp(-0.5 * (self.distance / sigma) ** 2).astype(np.float32)
        return self._coarse[sigma]

    def locate(self, points, positions, yaw_step=math.radians(3.0), coarse_sigma=0.08, max_points=90, keep=10,
               min_score=0.5, margin=0.06, separate_xy=0.20, separate_yaw=math.radians(25.0), possible=None,
               min_known=0.45, max_crossing=0.12):
        """Where is the robot? ``positions``: candidate (x, y) places (e.g. every free spot of the arena on a
        6 cm lattice). Every heading is tried at each of them with a blurred, cheap version of the map; the
        best few are refined with the exact matcher. ``ok`` only if the best pose fits well AND no clearly
        different pose fits nearly as well (within ``margin``). ``possible(x, y)``: can the robot stand there
        at all? (refined poses that cannot be are dropped). ``min_known``: on a map the LiDAR drew itself, at
        least this share of the scan must land on ground it has seen before. ``max_crossing``: places where
        more than this share of the rays would pass through known walls are dropped (see ``crossing``); that
        is what tells a right answer from a few walls that merely happen to fit."""
        pts = self.usable(points)
        positions = np.asarray(positions, float).reshape(-1, 2)
        none = lambda why: LocateResult(False, why, None, 0.0, 0.0, len(pts), [])      # noqa: E731
        if len(pts) < self.min_points:
            return none('too few LiDAR points (%d)' % len(pts))
        if not len(positions):
            return none('no place to search')
        step = max(1, int(math.ceil(len(pts) / float(max_points))))
        sub = pts[::step]
        field = self._coarse_field(coarse_sigma)
        yaws = np.arange(int(round(2 * math.pi / yaw_step))) * yaw_step - math.pi
        px, py = positions[:, 0:1], positions[:, 1:2]
        scores = np.empty((len(yaws), len(positions)), np.float32)
        floor = max(self.min_known * len(sub), 1.0)
        for k, yaw in enumerate(yaws):
            c, s_ = math.cos(yaw), math.sin(yaw)
            wx = px + (c * sub[:, 0] - s_ * sub[:, 1])[None, :]
            wy = py + (s_ * sub[:, 0] + c * sub[:, 1])[None, :]
            r, col, inside = self._cells(wx, wy)
            value = field[r, col]
            if self.edge_is_wall:
                scores[k] = np.where(inside, value, 0.0).mean(axis=1)       # (coarse: beyond the edge = no wall)
            else:
                weights = inside if self.known is None else (self.known[r, col] & inside)
                scores[k] = (value * weights).sum(axis=1) / np.maximum(weights.sum(axis=1), floor)
        # best few clearly different poses
        flat = scores.ravel()
        order = np.argsort(flat)[::-1][:4000]
        picked = []
        for index in order:
            k, j = divmod(int(index), len(positions))
            pose = (float(positions[j, 0]), float(positions[j, 1]), float(yaws[k]))
            if all(math.dist(pose[:2], other[:2]) > separate_xy or abs(wrap(pose[2] - other[2])) > separate_yaw
                   for other in picked):
                picked.append(pose)
                if len(picked) >= keep:
                    break
        refined = []
        for pose in picked:
            result = self.match(pose, points, window_xy=0.08, window_yaw=math.radians(4.0), step_xy=0.02,
                                step_yaw=math.radians(1.0), min_score=0.0, ambiguity_tol=-1.0)
            again = self.match(result.pose, points, window_xy=0.03, window_yaw=math.radians(1.5), step_xy=0.01,
                               step_yaw=math.radians(0.5), min_score=0.0, ambiguity_tol=-1.0)
            best = again if again.score >= result.score else result
            if possible is not None and not possible(best.pose[0], best.pose[1]):
                continue
            if self.crossing(best.pose, points) > max_crossing:
                continue
            if all(math.dist(best.pose[:2], other['pose'][:2]) > separate_xy / 2
                   or abs(wrap(best.pose[2] - other['pose'][2])) > separate_yaw / 2 for other in refined):
                refined.append(dict(pose=tuple(best.pose), score=float(best.score), matched=float(best.matched),
                                    known=float(self.evaluate(best.pose, points)['known'])))
        refined.sort(key=lambda item: -item['score'])
        if not refined:
            return none('the LiDAR view does not fit anywhere in the known surroundings (the robot may stand where '
                        'the LiDAR has not drawn the room yet: let it drive through the arena once, or start from a '
                        'corner)')
        top = refined[0]
        rivals = [item for item in refined[1:] if item['score'] >= top['score'] - margin]
        result = LocateResult(True, 'ok', top['pose'], top['score'], top['matched'], len(pts), [top] + rivals)
        if top['known'] < min_known and all(item['known'] < min_known for item in rivals):
            result.ok = False
            result.reason = ('the robot seems to stand where the LiDAR has not drawn the surroundings yet (only %.0f %% '
                             'of the view is known). Let it drive through the arena once, or start from a corner'
                             % (100 * top['known']))
            result.candidates = []
        elif top['score'] < min_score:
            result.ok = False
            result.reason = ('the LiDAR view does not fit anywhere well enough (best %.0f %%)' % (100 * top['matched']))
            result.candidates = [top]
        elif rivals:
            result.ok = False
            result.reason = ('%d places fit about equally well: choose the right one' % (len(rivals) + 1))
        return result
