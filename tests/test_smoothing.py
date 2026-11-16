"""Route smoothing: never unsafe, never worse for the planner's objective, ends never moved."""
import math
import random
import unittest

import numpy as np

from ugv_planner import Footprint, PlannerConfig, load_environment
from ugv_planner.collision import CollisionChecker
from ugv_nav.fast_planner import PlannerCache, replan_fast
from ugv_nav.path_processing import path_length, remove_collinear
from ugv_nav.smoothing import _Evaluator, smooth_route


def turning_deg(points):
    total = 0.0
    for a, b, c in zip(points, points[1:], points[2:]):
        total += abs(math.degrees(math.remainder(
            math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0]), 2 * math.pi)))
    return total


class SmoothingTests(unittest.TestCase):
    def routes(self, config, count=6, seed=3, envs=('ENV1', 'ENV2', 'ENV3')):
        rng = random.Random(seed)
        cache = PlannerCache()
        for env in envs:
            grid, _ = load_environment(env)
            made = 0
            for _ in range(200):
                if made >= count:
                    break
                pose = (rng.uniform(0.2, 2.8), rng.uniform(0.2, 2.8))
                goal = (rng.uniform(0.2, 2.8), rng.uniform(0.2, 2.8))
                result = replan_fast(grid, pose, goal, config, cache)
                if result.found and result.distance_m > 0.8:
                    made += 1
                    yield grid, cache, remove_collinear(result.waypoints)

    def check(self, config):
        improved = 0
        for grid, cache, raw in self.routes(config):
            smooth, info = smooth_route(raw, grid, config, cache)
            ev = _Evaluator(grid, config, cache)
            checker = CollisionChecker(grid)
            self.assertEqual(smooth[0], raw[0])                       # robot pose untouched
            self.assertEqual(smooth[-1], raw[-1])                     # goal untouched
            for a, b in zip(smooth, smooth[1:]):                      # exact swept-disk safety, full radius
                self.assertTrue(checker.is_clear(a, b, config.planning_radius))
            cost = lambda pts: sum(ev.cost(a, b) for a, b in zip(pts, pts[1:]))   # noqa: E731
            self.assertLessEqual(cost(smooth), cost(raw) + 1e-6)      # planner objective never worse
            self.assertLessEqual(path_length(smooth), path_length(raw) + 1e-9)
            self.assertLessEqual(turning_deg(smooth), turning_deg(raw) + 1e-6)
            self.assertAlmostEqual(info['length_m'], path_length(smooth))
            improved += info['smoothed']
        return improved

    def test_clearance_preferred_routes(self):
        improved = self.check(PlannerConfig(8, Footprint(0.09, 0.02, True), True, clearance_weight=4.0))
        self.assertGreater(improved, 8)

    def test_shortest_routes(self):
        self.assertGreater(self.check(PlannerConfig(8, Footprint(0.09, 0.02, True), True)), 8)

    def test_robot_sized_footprint(self):
        self.assertGreater(self.check(PlannerConfig(8, Footprint(0.143, 0.02, True), True, clearance_weight=4.0)), 2)

    def test_clearance_preference_is_respected(self):
        """With the clearance preference, smoothing must not pull the route against the walls."""
        config = PlannerConfig(8, Footprint(0.143, 0.02, True), True, clearance_weight=4.0)
        grid, _ = load_environment('ENV1')
        cache = PlannerCache()
        raw = remove_collinear(replan_fast(grid, (0.345, 0.315), (2.655, 2.595), config, cache).waypoints)
        smooth, info = smooth_route(raw, grid, config, cache)
        self.assertTrue(info['smoothed'])
        checker = CollisionChecker(grid)
        clearance = lambda pts: min(checker.clearance(a, b) for a, b in zip(pts, pts[1:]))   # noqa: E731
        self.assertGreaterEqual(clearance(smooth), clearance(raw) - 0.01)
        self.assertLess(turning_deg(smooth), 0.5 * turning_deg(raw))      # far fewer direction changes
        sharp = lambda pts: sum(abs(math.degrees(math.remainder(                               # noqa: E731
            math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0]), 2 * math.pi))) > 35
            for a, b, c in zip(pts, pts[1:], pts[2:]))
        self.assertLess(sharp(smooth), sharp(raw))                        # fewer stop-and-turn corners

    def test_short_and_degenerate_routes_unchanged(self):
        config = PlannerConfig(8, Footprint(0.1, 0.0, True), True)
        grid, _ = load_environment('ENV1')
        for pts in ([(1.5, 0.4)], [(1.5, 0.4), (1.8, 0.4)]):
            out, info = smooth_route(pts, grid, config)
            self.assertEqual(out, pts)
            self.assertFalse(info['smoothed'])

    def test_navigator_can_switch_smoothing_off(self):
        from types import SimpleNamespace
        from ugv_nav.navigator import PLANNED, Navigator, NavigatorConfig
        grid, _ = load_environment('ENV1')
        config = PlannerConfig(8, Footprint(0.143, 0.02, True), True, clearance_weight=4.0)
        tel = dict(odom=dict(x=0.0, y=0.0, yaw=0.0, age=0.0))
        paths = {}
        for smooth in (False, True):
            client = SimpleNamespace(stop=lambda: None, snapshot=lambda: dict(telemetry=tel, telemetry_age=0.0))
            nav = Navigator(client, grid, config, NavigatorConfig(smooth_path=smooth))
            nav.set_initial_pose((0.345, 0.315, 0.0))
            nav.set_goal((2.655, 2.595))
            self.assertEqual(nav.state, PLANNED)
            paths[smooth] = list(nav.path)
            self.assertEqual(nav.plan_info['smoothed'], smooth)
        reference = remove_collinear(replan_fast(grid, (0.345, 0.315), (2.655, 2.595), config).waypoints)
        self.assertEqual(paths[False], reference)                         # exactly the A* route when off
        self.assertLess(len(paths[True]), len(paths[False]))
        self.assertTrue(np.isfinite(path_length(paths[True])))


if __name__ == '__main__':
    unittest.main()
