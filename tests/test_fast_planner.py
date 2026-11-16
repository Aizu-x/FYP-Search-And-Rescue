"""The fast planner must return exactly what the original ugv_planner A* returns."""
import math
import random
import unittest

import numpy as np

from ugv_planner import Footprint, Grid, PlannerConfig, load_environment
from ugv_planner.astar import plan
from ugv_planner.collision import CollisionChecker
from ugv_planner.interfaces import NavigationService
from ugv_nav.fast_planner import PlannerCache, RoutePlanner, plan_fast, replan_fast

CONFIGS = [
    PlannerConfig(8, Footprint(0.143, 0.02, True), True, clearance_weight=4.0),          # the robot's settings
    PlannerConfig(8, Footprint(0.143, 0.02, True), True, record_search=True, clearance_weight=4.0),
    PlannerConfig(8, Footprint(0.165, 0.0, True), True),                                  # radius on a lattice distance
    PlannerConfig(8, Footprint(0.15, 0.015, True), True, clearance_weight=1.5, preferred_clearance_m=0.12),
    PlannerConfig(4, Footprint(0.09, 0.0, True), True, clearance_weight=1.0),
    PlannerConfig(4, Footprint(0.12, 0.03, True), True),
    PlannerConfig(8, Footprint(0.10, 0.02, True), False),                                 # inflation off
    PlannerConfig(8, Footprint(None, 0.0, False), True, clearance_weight=4.0),            # unknown radius
]
FIELDS = ('status', 'message', 'cells', 'waypoints', 'cost_cells', 'distance_m', 'expanded_nodes', 'search_cells',
          'objective_cost_cells')


def same(test, a, b, context):
    for name in FIELDS:
        test.assertEqual(getattr(a, name), getattr(b, name), '%s differs for %s' % (name, context))


class EquivalenceTests(unittest.TestCase):
    def test_cell_planner_identical_on_random_queries(self):
        rng = random.Random(7)
        cache = PlannerCache()
        found = 0
        for env in ('ENV1', 'ENV2', 'ENV3'):
            grid, _ = load_environment(env)
            free = [tuple(int(v) for v in cell) for cell in np.argwhere(~grid.cells)]
            for config in CONFIGS:
                for _ in range(5):
                    a, b = rng.choice(free), rng.choice(free)
                    fast, ref = plan_fast(grid, a, b, config, cache), plan(grid, a, b, config)
                    same(self, fast, ref, (env, config, a, b))
                    found += fast.found
        self.assertGreater(found, 40)                      # the comparison covers many real routes

    def test_invalid_and_blocked_queries_identical(self):
        grid, _ = load_environment('ENV1')
        config = CONFIGS[0]
        for a, b in [((0, 0), (50, 50)), ((50, 50), (0, 0)), ((-1, 5), (50, 50)), ((50, 50), (100, 3)),
                     ((50.0, 50), (10, 10)), ((10, 11), (10, 11))]:
            same(self, plan_fast(grid, a, b, config), plan(grid, a, b, config), (a, b))

    def test_pose_connector_identical(self):
        rng = random.Random(11)
        cache = PlannerCache()
        for env in ('ENV1', 'ENV2', 'ENV3'):
            grid, _ = load_environment(env)
            for config in CONFIGS[:5]:
                for _ in range(4):
                    pose = (rng.uniform(0.1, 2.9), rng.uniform(0.1, 2.9))
                    goal = (rng.uniform(0.1, 2.9), rng.uniform(0.1, 2.9))
                    same(self, replan_fast(grid, pose, goal, config, cache),
                         NavigationService(grid, pose, goal, config).replan(), (env, pose, goal))

    def test_random_maps_identical(self):
        rng = np.random.default_rng(5)
        config = PlannerConfig(8, Footprint(0.05, 0.01, True), True, clearance_weight=2.0, preferred_clearance_m=0.1)
        for _ in range(12):
            cells = (rng.random((40, 40)) < 0.12).astype(np.uint8)
            grid = Grid(cells)
            free = [tuple(int(v) for v in cell) for cell in np.argwhere(~grid.cells)]
            a, b = free[int(rng.integers(len(free)))], free[int(rng.integers(len(free)))]
            same(self, plan_fast(grid, a, b, config, PlannerCache()), plan(grid, a, b, config), (a, b))

    def test_every_route_segment_passes_the_exact_check(self):
        grid, _ = load_environment('ENV1')
        config = CONFIGS[0]
        result = replan_fast(grid, (0.345, 0.315), (2.655, 2.595), config)
        self.assertTrue(result.found)
        checker = CollisionChecker(grid)
        for a, b in zip(result.waypoints, result.waypoints[1:]):
            self.assertTrue(checker.is_clear(a, b, config.planning_radius))


class CacheAndFacadeTests(unittest.TestCase):
    def test_cache_follows_map_content_not_object_identity(self):
        grid, _ = load_environment('ENV1')
        config = CONFIGS[0]
        cache = PlannerCache()
        first = replan_fast(grid, (0.345, 0.315), (2.655, 2.595), config, cache)
        again = replan_fast(Grid(grid.cells.copy()), (0.345, 0.315), (2.655, 2.595), config, cache)
        self.assertEqual(first.cells, again.cells)
        self.assertGreater(cache.hits, 0)
        # a wall across the route must change the answer (no stale cache)
        cells = grid.cells.copy()
        r, c = grid.world_to_cell(first.waypoints[len(first.waypoints) // 2])
        cells[max(0, r - 3):r + 4, max(0, c - 3):c + 4] = True
        blocked = replan_fast(Grid(cells), (0.345, 0.315), (2.655, 2.595), config, cache)
        self.assertNotEqual(first.cells, blocked.cells)
        same(self, blocked, NavigationService(Grid(cells), (0.345, 0.315), (2.655, 2.595), config).replan(), 'edited')

    def test_engines_agree_and_fast_is_faster(self):
        grid, _ = load_environment('ENV1')
        config = CONFIGS[0]
        fast, original = RoutePlanner('fast'), RoutePlanner('original')
        a = fast.replan(grid, (1.2, 0.4), (0.9, 2.5), config)
        b = original.replan(grid, (1.2, 0.4), (0.9, 2.5), config)
        same(self, a, b, 'engines')
        fast.replan(grid, (1.2, 0.4), (0.9, 2.5), config)          # warm cache
        self.assertLess(fast.last_ms * 3, original.last_ms)
        with self.assertRaises(ValueError):
            RoutePlanner('other')
        self.assertTrue(math.isfinite(a.distance_m))


if __name__ == '__main__':
    unittest.main()
