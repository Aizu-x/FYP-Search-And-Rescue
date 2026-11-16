import unittest
import tempfile
from pathlib import Path
import heapq
import math
import json
import numpy as np
from ugv_planner import Grid, Footprint, PlannerConfig, plan, evaluate, load_environment
from ugv_planner.config import EXAMPLES
from ugv_planner.collision import CollisionChecker
from ugv_planner.inflation import inflated_mask
from ugv_planner.interfaces import NavigationService, grid_from_ros_values
from ugv_planner.simulation import SimulatedUGV
from ugv_planner.grid import dataset_manifest
from ugv_planner.exporting import export_run, export_summary


def dijkstra_reference(a, start, goal, connectivity):
    """Independent tiny graph oracle: no production heuristic/collision/inflation code."""
    queue = [(0., start)]; costs = {start: 0.}
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if connectivity == 8:
        moves += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    while queue:
        cost, (r, c) = heapq.heappop(queue)
        if cost != costs[(r, c)]: continue
        if (r, c) == goal: return cost
        for dr, dc in moves:
            nr, nc = r+dr, c+dc
            if not (0 <= nr < len(a) and 0 <= nc < len(a[0])) or a[nr, nc]: continue
            if dr and dc and (a[r+dr, c] or a[r, c+dc]): continue
            value = cost+math.hypot(dr, dc)
            if value < costs.get((nr, nc), math.inf):
                costs[nr, nc] = value; heapq.heappush(queue, (value, (nr, nc)))
    return math.inf


class AlgorithmTests(unittest.TestCase):
    def test_known_optimal_costs(self):
        grid = Grid(np.zeros((8, 8)), 1.)
        for mode, cost in [(4, 9.), (8, 3+3*math.sqrt(2))]:
            r = plan(grid, (1, 1), (4, 7), PlannerConfig(mode))
            self.assertTrue(r.found); self.assertAlmostEqual(r.cost_cells, cost)
            self.assertEqual(r.cells[0], (1, 1)); self.assertEqual(r.cells[-1], (4, 7))

    def test_random_optimality_against_independent_dijkstra(self):
        rng = np.random.default_rng(1234)
        for mode in (4, 8):
            for _ in range(25):
                a = rng.random((9, 9)) < .23; a[0, 0] = a[-1, -1] = False
                expected = dijkstra_reference(a, (0, 0), (8, 8), mode)
                r = plan(Grid(a, 1.), (0, 0), (8, 8), PlannerConfig(mode))
                self.assertEqual(r.found, math.isfinite(expected))
                if r.found: self.assertAlmostEqual(r.cost_cells, expected)

    def test_invalid_and_occupied(self):
        a = np.zeros((5, 5)); a[2, 2] = 1; grid = Grid(a)
        for start, goal in [((-1, 0), (4, 4)), ((0, 0), (5, 4)), ((2, 2), (4, 4)), ((0, 0), (2, 2)), ((.5, 1), (4, 4))]:
            self.assertEqual(plan(grid, start, goal).status, 'invalid')

    def test_no_corner_cutting(self):
        a = np.array([[0, 1], [1, 0]])
        self.assertEqual(plan(Grid(a, 1.), (0, 0), (1, 1)).status, 'unreachable')

    def test_unreachable(self):
        a = np.zeros((7, 7)); a[3, :] = 1
        self.assertEqual(plan(Grid(a), (1, 1), (5, 5)).status, 'unreachable')

    def test_identical_endpoints(self):
        r = plan(Grid(np.zeros((5, 5))), (2, 2), (2, 2))
        self.assertEqual(len(r.waypoints), 1); self.assertEqual(r.distance_m, 0)
        report = evaluate(r, Grid(np.zeros((5, 5))), Footprint())
        self.assertEqual(report['detour_ratio'], 1.)

    def test_inflation_closes_passage(self):
        a = np.zeros((11, 11)); a[5, :] = 1; a[5, 5] = 0
        grid = Grid(a, 1.)
        self.assertTrue(plan(grid, (2, 5), (8, 5)).found)
        self.assertEqual(plan(grid, (2, 5), (8, 5), PlannerConfig(8, Footprint(.51))).status, 'unreachable')
        self.assertTrue(plan(grid, (2, 5), (8, 5), PlannerConfig(8, Footprint(.51), False)).found)

    def test_config_validation(self):
        for v in [-1, math.inf, math.nan]:
            with self.assertRaises(ValueError): Footprint(v)
        with self.assertRaises(ValueError): Footprint(None, 0, True)
        with self.assertRaises(ValueError): PlannerConfig(6)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        a = np.zeros((10, 10)); a[4, 4] = 1
        self.grid = Grid(a, 1.); self.check = CollisionChecker(self.grid)

    def test_continuous_crossing_between_free_endpoints(self):
        self.assertTrue(self.check.is_clear((3.5, 4.5), (3.5, 4.5)))
        self.assertTrue(self.check.is_clear((5.5, 4.5), (5.5, 4.5)))
        self.assertEqual(self.check.clearance((3.5, 4.5), (5.5, 4.5)), 0.)

    def test_corner_and_disk_tangency(self):
        self.assertFalse(self.check.is_clear((3.5, 4.5), (4.5, 3.5)))
        self.assertAlmostEqual(self.check.clearance((3.5, 3.5), (3.5, 3.5)), math.sqrt(.5))
        self.assertFalse(self.check.is_clear((3., 4.), (3., 5.), 1.))
        self.assertTrue(self.check.is_clear((3., 4.), (3., 5.), .99))

    def test_swept_disk_rejects_edge_with_clear_endpoints(self):
        a, b = (3.5, 4.5), (4.5, 3.5)
        self.assertTrue(self.check.is_clear(a, a, .2)); self.assertTrue(self.check.is_clear(b, b, .2))
        self.assertFalse(self.check.is_clear(a, b, .2))

    def test_inflation_matches_point_geometry(self):
        for radius in [0, .49, .5, .71, 1.1]:
            mask = inflated_mask(self.grid, radius)
            for r in range(10):
                for c in range(10):
                    p = self.grid.cell_to_world((r, c))
                    self.assertEqual(bool(mask[r, c]), not self.check.is_clear(p, p, radius))

    def test_map_boundary(self):
        self.assertFalse(self.check.is_clear((0, 2), (1, 2)))
        self.assertFalse(self.check.is_clear((.5, 2), (1, 2), .5))

    def test_gt_detects_center_and_footprint_separately(self):
        occupancy = Grid(np.zeros((10, 10)), 1.)
        r = plan(occupancy, (4, 2), (4, 7))
        report = evaluate(r, self.grid, Footprint())
        self.assertTrue(report['gt_center_collision']); self.assertTrue(report['unsafe_segments'])
        r = plan(occupancy, (3, 2), (3, 7))
        report = evaluate(r, self.grid, Footprint(.6, 0, True))
        self.assertFalse(report['gt_center_collision']); self.assertTrue(report['gt_footprint_collision'])
        self.assertIn('failed', report['validation'])

    def test_safety_requires_verified_geometry(self):
        r = plan(self.grid, (1, 1), (1, 7))
        for fp in [Footprint(), Footprint(.1)]:
            self.assertIn('undetermined', evaluate(r, self.grid, fp)['validation'])
        self.assertIn('passed', evaluate(r, self.grid, Footprint(.1, 0, True))['validation'])


class MotionAndIntegrationTests(unittest.TestCase):
    def test_motion_pause_resume_reset_and_completion(self):
        sim = SimulatedUGV(); sim.speed = 1.; sim.set_route([(0., 0.), (1., 0.), (1., 1.)]); sim.start()
        sim.step(.25); self.assertEqual(sim.position, (.25, 0.))
        sim.pause(); sim.step(1); self.assertEqual(sim.position, (.25, 0.))
        sim.start(); sim.step(1); self.assertEqual(sim.position, (1., .25))
        sim.step(1); self.assertEqual(sim.state, 'arrived'); self.assertEqual(sim.position, (1., 1.))
        sim.reset(); self.assertEqual(sim.position, (0., 0.)); self.assertEqual(sim.state, 'ready')

    def test_motion_checks_each_turn_with_large_timestep(self):
        sim = SimulatedUGV(); sim.speed = 10; sim.set_route([(0., 0.), (1., 0.), (1., 1.)]); sim.start()
        seen = []
        def guard(a, b):
            seen.append((a, b)); return b != (1., 1.)
        sim.step(1, guard)
        self.assertEqual(len(seen), 2); self.assertEqual(sim.state, 'collision'); self.assertEqual(sim.position, (1., 0.))

    def test_replan_from_exact_current_pose(self):
        grid = Grid(np.zeros((10, 10)), 1.)
        service = NavigationService(grid, (1.7, 1.6), (7.5, 7.5))
        first = service.replan(); self.assertEqual(first.waypoints[0], (1.7, 1.6))
        service.update_pose((2.2, 2.3)); service.set_goal((1.5, 8.5))
        second = service.replan(); self.assertEqual(second.waypoints[0], (2.2, 2.3)); self.assertEqual(second.waypoints[-1], (1.5, 8.5))
        self.assertNotEqual(first.cells, second.cells)

    def test_replan_rejects_unsafe_connector(self):
        a = np.zeros((8, 8)); a[2, 2] = 1
        service = NavigationService(Grid(a, 1), (3.01, 2.5), (6.5, 6.5), PlannerConfig(8, Footprint(.1)))
        self.assertEqual(service.replan().status, 'invalid')

    def test_ros_adapter_unknown_blocked_and_origin(self):
        grid = grid_from_ros_values([0, -1, 50, 49], 2, 2, .1, (2., 3.))
        np.testing.assert_array_equal(grid.cells, [[0, 1], [1, 0]])
        self.assertEqual(grid.cell_to_world((0, 0)), (2.05, 3.05))


class ActualDatasetTests(unittest.TestCase):
    def test_all_files_alignment_and_source_integrity(self):
        before = dataset_manifest(); self.assertEqual(len(before), 6)
        for name in EXAMPLES:
            grid, gt = load_environment(name)
            self.assertEqual(grid.shape, (100, 100)); self.assertEqual(grid.extent, gt.extent)
            self.assertEqual(grid.extent, (0., 3., 0., 3.))
            changed = grid.edited((50, 50), not grid.cells[50, 50])
            self.assertNotEqual(grid.cells[50, 50], changed.cells[50, 50])
        self.assertEqual(dataset_manifest(), before)

    def test_examples_and_export(self):
        reports = {}
        with tempfile.TemporaryDirectory(prefix='UGV test with spaces ') as folder:
            for name, (start, goal) in EXAMPLES.items():
                grid, gt = load_environment(name)
                r = plan(grid, grid.world_to_cell(start), grid.world_to_cell(goal))
                self.assertTrue(r.found, f'{name}: {r.message}')
                for a, b in zip(r.waypoints, r.waypoints[1:]):
                    self.assertTrue(CollisionChecker(grid).is_clear(a, b))
                report = evaluate(r, gt, Footprint()); reports[name] = report
                export_run(folder, name, grid, gt, r, report, PlannerConfig())
                self.assertGreater((Path(folder)/f'{name}.png').stat().st_size, 1000)
                payload = json.loads((Path(folder)/f'{name}.json').read_text())
                self.assertEqual(payload['metrics']['path_found'], r.found)
                if name == 'ENV3': self.assertTrue(report['gt_center_collision'])
            export_summary(folder, reports)
            self.assertTrue((Path(folder)/'summary.csv').is_file())

    def test_loader_rejects_malformed_maps(self):
        with tempfile.TemporaryDirectory() as folder:
            np.save(Path(folder)/'ENV2_occupancy.npy', np.zeros((3, 3)))
            np.save(Path(folder)/'ENV2_GT.npy', np.zeros((3, 3)))
            with self.assertRaises(ValueError): load_environment('ENV2', folder)
        with self.assertRaises(ValueError): Grid(np.full((2, 2), 2))


if __name__ == '__main__':
    unittest.main()
