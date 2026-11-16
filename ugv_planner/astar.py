from dataclasses import dataclass, field
import heapq
import math
import time
import numpy as np
from .config import PlannerConfig
from .collision import CollisionChecker
from .inflation import inflated_mask
from .clearance import penalties


@dataclass
class PlanResult:
    status: str
    message: str
    cells: list = field(default_factory=list)
    waypoints: list = field(default_factory=list)
    cost_cells: float = 0.0
    distance_m: float = 0.0
    expanded_nodes: int = 0
    computation_ms: float = 0.0
    search_cells: list = field(default_factory=list)
    objective_cost_cells: float = 0.0

    @property
    def found(self):
        return self.status == 'found'


def heuristic(a, b, connectivity):
    dy, dx = abs(a[0]-b[0]), abs(a[1]-b[1])
    return dx+dy if connectivity == 4 else max(dx, dy)+(math.sqrt(2)-1)*min(dx, dy)


def plan(grid, start, goal, config=PlannerConfig()):
    """A* on (row, column) cell centers. Does not accept or consult GT.

    Timing includes inflation and edge collision checks. Diagonals require
    both orthogonal adjacent centers to be free in the inflated grid, AND
    the entire swept disk segment to clear original occupied squares.
    """
    began = time.perf_counter()
    result = PlanResult('unreachable', 'No path available.')
    def finish():
        result.computation_ms = (time.perf_counter()-began)*1000
        return result
    for label, cell in [('start', start), ('goal', goal)]:
        if len(cell) != 2 or any(not isinstance(x, (int, np.integer)) for x in cell) or not grid.contains(cell):
            result.status = 'invalid'; result.message = f'Invalid {label}: outside grid or noninteger cell.'
            return finish()
    start, goal = tuple(start), tuple(goal)
    blocked = inflated_mask(grid, config.planning_radius)
    for label, cell in [('start', start), ('goal', goal)]:
        if blocked[cell]:
            result.status = 'invalid'; result.message = f'Invalid {label}: occupied or inside planning inflation.'
            return finish()
    checker = CollisionChecker(grid)
    penalty = penalties(grid, config) if config.clearance_weight else np.zeros(grid.shape)
    g = np.full(grid.shape, np.inf); g[start] = 0
    closed = np.zeros(grid.shape, bool); parent = {}
    queue = [(heuristic(start, goal, config.connectivity), 0.0, start)]
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if config.connectivity == 8:
        directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    while queue:
        _, cost, current = heapq.heappop(queue)
        if closed[current] or cost > g[current]+1e-12:
            continue
        closed[current] = True; result.expanded_nodes += 1
        if config.record_search:
            result.search_cells.append(current)
        if current == goal:
            route = [current]
            while route[-1] != start:
                route.append(parent[route[-1]])
            result.cells = route[::-1]
            result.waypoints = [grid.cell_to_world(c) for c in result.cells]
            result.objective_cost_cells = float(cost)
            result.cost_cells = sum(math.dist(a, b) for a, b in zip(result.cells, result.cells[1:]))
            result.distance_m = result.cost_cells*grid.resolution
            result.status = 'found'
            result.message = 'Clearance-preferred route found using occupancy only.' if config.clearance_weight else 'Shortest-distance route found using occupancy only.'
            return finish()
        r, c = current
        for dr, dc in directions:
            nxt = (r+dr, c+dc)
            if not grid.contains(nxt) or blocked[nxt] or closed[nxt]:
                continue
            if dr and dc and (blocked[r+dr, c] or blocked[r, c+dc]):
                continue
            length = math.sqrt(2) if dr and dc else 1
            new_cost = cost+length*(1+config.clearance_weight*(penalty[current]+penalty[nxt])/2)
            if new_cost >= g[nxt]-1e-12:
                continue
            if not checker.is_clear(grid.cell_to_world(current), grid.cell_to_world(nxt), config.planning_radius):
                continue
            g[nxt] = new_cost; parent[nxt] = current
            heapq.heappush(queue, (new_cost+heuristic(nxt, goal, config.connectivity), new_cost, nxt))
    return finish()
