"""Faster route planning on the edge with EXACTLY the same results as ``ugv_planner``.

``ugv_planner.astar.plan`` is unchanged and remains the reference. It spends about 85 % of its
time in an exact swept-disk check for every edge it relaxes. Inside the search that check can
never fail:

* A cell is "blocked" when its centre is within the planning radius R of an occupied square or of
  the map boundary (``inflated_mask``). The search only moves between unblocked centres, and a
  diagonal move also needs both orthogonal neighbours unblocked.
* An orthogonal move stays inside the segment between two unblocked centres; a diagonal move stays
  inside the unit square whose corners are the four unblocked centres around it. Occupied squares
  are axis-aligned and sit on the same lattice, so over such a segment/square the distance to any
  occupied square is smallest at one of those centres -- which is already > R.

So the search needs no per-edge geometry. ``plan_fast`` runs the identical search (same costs,
same heuristic, same tie-breaking, the same floating-point expressions) on flat Python lists and
then verifies the finished route with the same exact checker. If that verification ever failed it
falls back to the original planner. The inflation mask, the clearance penalties and the collision
checker are cached per map + footprint, so replanning on an unchanged map is much cheaper.

``tests/test_fast_planner.py`` compares both planners field by field on random queries.
"""
from __future__ import annotations

import hashlib
import heapq
import math
import threading
import time
from collections import OrderedDict

import numpy as np

from ugv_planner.astar import PlanResult, plan as plan_reference
from ugv_planner.clearance import clearance_field, penalties
from ugv_planner.collision import CollisionChecker
from ugv_planner.config import PlannerConfig
from ugv_planner.inflation import inflated_mask
from ugv_planner.interfaces import NavigationService

_SQRT2 = math.sqrt(2)
_DIAG = math.sqrt(2) - 1


class _Fields:
    __slots__ = ('blocked', 'blocked_flat', 'penalty', 'penalty_flat')


class PlannerCache:
    """Per map + footprint: inflation mask, clearance penalties, collision checker, distance fields."""

    def __init__(self, size=24):
        self._lock = threading.Lock()
        self._items = OrderedDict()
        self._size = size
        self.hits = 0
        self.misses = 0

    @staticmethod
    def grid_key(grid):
        return (grid.shape, float(grid.resolution), tuple(float(v) for v in grid.origin),
                hashlib.blake2b(grid.cells.tobytes(), digest_size=16).digest())

    def _get(self, key, build):
        with self._lock:
            if key in self._items:
                self._items.move_to_end(key)
                self.hits += 1
                return self._items[key]
        value = build()                      # heavy work outside the lock
        with self._lock:
            self.misses += 1
            self._items[key] = value
            self._items.move_to_end(key)
            while len(self._items) > self._size:
                self._items.popitem(last=False)
        return value

    def checker(self, grid):
        return self._get(('checker', self.grid_key(grid)), lambda: CollisionChecker(grid))

    def fields(self, grid, config):
        key = ('fields', self.grid_key(grid), float(config.planning_radius), bool(config.clearance_weight),
               float(config.preferred_clearance_m))

        def build():
            f = _Fields()
            f.blocked = inflated_mask(grid, config.planning_radius)
            f.blocked_flat = f.blocked.ravel().tolist()
            f.penalty = penalties(grid, config) if config.clearance_weight else None
            f.penalty_flat = f.penalty.ravel().tolist() if f.penalty is not None else None
            return f
        return self._get(key, build)

    def distance(self, grid, cap):
        """Exact centre-to-obstacle/boundary distance, clipped at ``cap`` (same function the planner uses)."""
        cap = round(float(cap), 4)
        return self._get(('distance', self.grid_key(grid), cap), lambda: clearance_field(grid, cap))


DEFAULT_CACHE = PlannerCache()


def _merge_collinear_cells(cells):
    out = [cells[0]]
    for prev, cur, nxt in zip(cells, cells[1:], cells[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            out.append(cur)
    if len(cells) > 1:
        out.append(cells[-1])
    return out


def plan_fast(grid, start, goal, config=PlannerConfig(), cache=None):
    """Same contract and same result as ``ugv_planner.astar.plan`` (see the module docstring)."""
    began = time.perf_counter()
    cache = cache or DEFAULT_CACHE
    result = PlanResult('unreachable', 'No path available.')

    def finish():
        result.computation_ms = (time.perf_counter() - began) * 1000
        return result

    for label, cell in (('start', start), ('goal', goal)):
        if len(cell) != 2 or any(not isinstance(x, (int, np.integer)) for x in cell) or not grid.contains(cell):
            result.status = 'invalid'
            result.message = f'Invalid {label}: outside grid or noninteger cell.'
            return finish()
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    fields = cache.fields(grid, config)
    for label, cell in (('start', start), ('goal', goal)):
        if fields.blocked[cell]:
            result.status = 'invalid'
            result.message = f'Invalid {label}: occupied or inside planning inflation.'
            return finish()
    h, w = grid.shape
    blocked = fields.blocked_flat
    weight = config.clearance_weight
    penalty = fields.penalty_flat if fields.penalty_flat is not None else [0.0] * (h * w)
    connectivity = config.connectivity
    record = config.record_search
    gr, gc = goal
    start_i, goal_i = start[0] * w + start[1], goal[0] * w + goal[1]
    inf = float('inf')
    g = [inf] * (h * w)
    g[start_i] = 0
    closed = bytearray(h * w)
    parent = [-1] * (h * w)
    # identical neighbour order to the reference: (1,0) (-1,0) (0,1) (0,-1) then the diagonals
    directions = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1)]
    if connectivity == 8:
        directions += [(1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2)]
    dy, dx = abs(start[0] - gr), abs(start[1] - gc)
    queue = [(dx + dy if connectivity == 4 else max(dx, dy) + _DIAG * min(dx, dy), 0.0, start_i)]
    push, pop = heapq.heappush, heapq.heappop
    expanded = 0
    search = []
    found_cost = None
    while queue:
        _, cost, current = pop(queue)
        if closed[current] or cost > g[current] + 1e-12:
            continue
        closed[current] = 1
        expanded += 1
        if record:
            search.append(divmod(current, w))
        if current == goal_i:
            found_cost = cost
            break
        r, c = divmod(current, w)
        pc = penalty[current]
        for dr, dc, length in directions:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= h or nc < 0 or nc >= w:
                continue
            nxt = nr * w + nc
            if blocked[nxt] or closed[nxt]:
                continue
            if dr and dc and (blocked[nr * w + c] or blocked[r * w + nc]):
                continue
            new_cost = cost + length * (1 + weight * (pc + penalty[nxt]) / 2)
            if new_cost >= g[nxt] - 1e-12:
                continue
            g[nxt] = new_cost
            parent[nxt] = current
            dy, dx = abs(nr - gr), abs(nc - gc)
            push(queue, (new_cost + (dx + dy if connectivity == 4 else max(dx, dy) + _DIAG * min(dx, dy)),
                         new_cost, nxt))
    result.expanded_nodes = expanded
    result.search_cells = search
    if found_cost is None:
        return finish()
    route = [goal_i]
    while route[-1] != start_i:
        route.append(parent[route[-1]])
    cells = [divmod(i, w) for i in reversed(route)]
    # Exact verification of the finished route (the reference checks every relaxed edge instead).
    checker = cache.checker(grid)
    corners = [grid.cell_to_world(cell) for cell in _merge_collinear_cells(cells)]
    for a, b in zip(corners, corners[1:]):
        if not checker.is_clear(a, b, config.planning_radius):
            return plan_reference(grid, start, goal, config)       # never expected; keeps the guarantee
    result.cells = cells
    result.waypoints = [grid.cell_to_world(cell) for cell in cells]
    result.objective_cost_cells = float(found_cost)
    result.cost_cells = sum(math.dist(a, b) for a, b in zip(cells, cells[1:]))
    result.distance_m = result.cost_cells * grid.resolution
    result.status = 'found'
    result.message = ('Clearance-preferred route found using occupancy only.' if config.clearance_weight
                      else 'Shortest-distance route found using occupancy only.')
    return finish()


def replan_fast(grid, pose, goal, config=PlannerConfig(), cache=None):
    """Same contract and same result as ``NavigationService(grid, pose, goal, config).replan()``."""
    if grid is None or pose is None or goal is None:
        raise ValueError('Map, localization pose and goal are required.')
    cache = cache or DEFAULT_CACHE
    pose = tuple(pose)
    result = plan_fast(grid, grid.world_to_cell(pose), grid.world_to_cell(goal), config, cache)
    if result.found and pose != tuple(result.waypoints[0]):
        if not cache.checker(grid).is_clear(pose, result.waypoints[0], config.planning_radius):
            result.status = 'invalid'
            result.message = 'Current pose cannot safely connect to its cell center.'
            result.cells = []
            result.waypoints = []
            result.distance_m = result.cost_cells = 0
        else:
            extra = math.dist(pose, result.waypoints[0])
            result.waypoints.insert(0, pose)
            result.distance_m += extra
            result.cost_cells += extra / grid.resolution
            penalty = (cache.fields(grid, config).penalty[grid.world_to_cell(pose)]
                       if config.clearance_weight else 0)
            result.objective_cost_cells += extra / grid.resolution * (1 + config.clearance_weight * penalty)
    return result


class RoutePlanner:
    """What the navigator and the console call. ``engine='original'`` uses ugv_planner directly."""

    def __init__(self, engine='fast', cache=None):
        if engine not in ('fast', 'original'):
            raise ValueError("Planner engine must be 'fast' or 'original'.")
        self.engine = engine
        self.cache = cache or PlannerCache()
        self.last_ms = 0.0

    def replan(self, grid, pose, goal, config):
        began = time.perf_counter()
        if self.engine == 'fast':
            result = replan_fast(grid, pose, goal, config, self.cache)
        else:
            result = NavigationService(grid, tuple(pose), tuple(goal), config).replan()
        self.last_ms = (time.perf_counter() - began) * 1000
        return result
