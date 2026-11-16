import heapq
import math
import unittest
import numpy as np
from ugv_planner import Grid, Footprint, PlannerConfig, plan, evaluate, load_environment
from ugv_planner.clearance import clearance_field
from ugv_planner.collision import CollisionChecker
from ugv_planner.config import EXAMPLES
from ugv_planner.evaluation import simulation_preflight
from ugv_planner.simulation import SimulatedUGV


class ClearanceTests(unittest.TestCase):
    def test_field_matches_exact_geometry(self):
        a = np.zeros((13, 17)); a[4, 7] = a[9, 10] = 1
        grid = Grid(a, .03); checker = CollisionChecker(grid)
        field = clearance_field(grid, .2)
        for cell in np.ndindex(grid.shape):
            p = grid.cell_to_world(cell)
            self.assertAlmostEqual(field[cell], min(.2, checker.clearance(p, p)))

    def test_preference_leaves_wall_without_changing_hard_constraints(self):
        a = np.zeros((25, 25)); a[5, :] = 1
        grid = Grid(a, 1.)
        short = plan(grid, (8, 2), (8, 22))
        roomy = plan(grid, (8, 2), (8, 22), PlannerConfig(clearance_weight=4, preferred_clearance_m=7))
        checker = CollisionChecker(grid)
        self.assertGreater(roomy.distance_m, short.distance_m)
        mean = lambda r: np.mean([checker.clearance(p, p) for p in r.waypoints])
        self.assertGreater(mean(roomy), mean(short)+.5)
        self.assertAlmostEqual(roomy.distance_m, sum(math.dist(a, b) for a, b in zip(roomy.waypoints, roomy.waypoints[1:])))

    def test_weighted_optimum_against_independent_dijkstra(self):
        rng = np.random.default_rng(51)
        for _ in range(8):
            a = rng.random((7, 7)) < .15; a[0, 0] = a[6, 6] = False
            grid = Grid(a, 1.); checker = CollisionChecker(grid)
            cfg = PlannerConfig(clearance_weight=3, preferred_clearance_m=3)
            # Independent graph solver; exact point geometry, not production field.
            penalty = {}
            for cell in np.ndindex(a.shape):
                p = grid.cell_to_world(cell)
                penalty[cell] = max(0, (3-checker.clearance(p, p))/3)**2
            costs = {(0, 0): 0.}; queue = [(0., (0, 0))]
            while queue:
                cost, cell = heapq.heappop(queue)
                if cost != costs[cell]: continue
                r, c = cell
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        n = (r+dr, c+dc)
                        if (dr == dc == 0) or not grid.contains(n) or a[n]: continue
                        if dr and dc and (a[r+dr, c] or a[r, c+dc]): continue
                        value = cost+math.hypot(dr, dc)*(1+3*(penalty[cell]+penalty[n])/2)
                        if value < costs.get(n, math.inf):
                            costs[n] = value; heapq.heappush(queue, (value, n))
            result = plan(grid, (0, 0), (6, 6), cfg)
            self.assertEqual(result.found, (6, 6) in costs)
            if result.found: self.assertAlmostEqual(result.objective_cost_cells, costs[(6, 6)])

    def test_known_unsafe_route_cannot_dispatch(self):
        occupancy = Grid(np.zeros((10, 10)), 1.)
        a = np.zeros((10, 10)); a[4, 4] = 1; gt = Grid(a, 1.)
        result = plan(occupancy, (4, 1), (4, 8))
        report = evaluate(result, gt, Footprint())
        self.assertFalse(simulation_preflight(result, occupancy, report, Footprint())[0])
        # Occupancy footprint veto remains active when planning inflation is off.
        result = plan(gt, (3, 1), (3, 8))
        report = evaluate(result, occupancy, Footprint(.6))
        self.assertFalse(simulation_preflight(result, gt, report, Footprint(.6))[0])

    def test_actual_examples_reach_goals_with_clearance_preference(self):
        cfg = PlannerConfig(footprint=Footprint(0, .1), clearance_weight=4)
        for name, (start, goal) in EXAMPLES.items():
            grid, gt = load_environment(name)
            result = plan(grid, grid.world_to_cell(start), grid.world_to_cell(goal), cfg)
            report = evaluate(result, gt, cfg.footprint)
            self.assertTrue(simulation_preflight(result, grid, report, cfg.footprint)[0], name)
            self.assertGreater(report['minimum_gt_clearance_cm'], 10)
            sim = SimulatedUGV(); sim.set_route(result.waypoints); sim.start()
            checkers = [CollisionChecker(grid), CollisionChecker(gt)]
            for _ in range(1000):
                sim.step(.1, lambda a, b: all(c.is_clear(a, b, .1) for c in checkers))
                if sim.state != 'running': break
            self.assertEqual(sim.state, 'arrived', name)
            self.assertEqual(sim.position, result.waypoints[-1])

    def test_invalid_preferences(self):
        for weight in [-1, float('nan')]:
            with self.assertRaises(ValueError): PlannerConfig(clearance_weight=weight)
        with self.assertRaises(ValueError): PlannerConfig(preferred_clearance_m=0)
