"""Test A: coordinate transforms, localization, path processing, controller, map updates."""
import math
import unittest

import numpy as np

from ugv_planner import Footprint, Grid, PlannerConfig, load_environment, plan
from ugv_planner.config import EXAMPLES
from ugv_nav.controller import ControllerConfig, PathTracker
from ugv_nav.localization import LocalizationConfig, PoseEstimator, compose, inverse, transform_point, wrap
from ugv_nav.map_update import mark_scan_obstacles
from ugv_nav.path_processing import remove_collinear, shortcut, validate_path, path_length


def tel(x, y, yaw, age=0.0):
    return {'odom': dict(x=x, y=y, yaw=yaw, age=age)}


class TransformTests(unittest.TestCase):
    def test_se2_identities(self):
        rng = np.random.default_rng(3)
        for _ in range(50):
            a = (*rng.uniform(-3, 3, 2), rng.uniform(-4, 4))
            b = (*rng.uniform(-3, 3, 2), rng.uniform(-4, 4))
            ident = compose(a, inverse(a))
            self.assertTrue(np.allclose(ident, (0, 0, 0), atol=1e-12))
            p = transform_point(compose(a, b), (0, 0))
            self.assertTrue(np.allclose(p, compose(a, b)[:2]))
        self.assertAlmostEqual(wrap(3 * math.pi), -math.pi)

    def test_grid_frame_convention_matches_planner(self):
        grid, _ = load_environment('ENV1')
        # Bottom-left cell centre and top-right cell centre, 0.03 m cells, origin lower-left corner.
        self.assertEqual(grid.cell_to_world((0, 0)), (0.015, 0.015))
        self.assertAlmostEqual(grid.cell_to_world((99, 99))[0], 2.985)
        self.assertEqual(grid.world_to_cell((0.345, 0.315)), (10, 11))


class LocalizationTests(unittest.TestCase):
    def test_calibration_maps_odometry_into_map_frame(self):
        est = PoseEstimator()
        est.update(tel(5.0, -2.0, math.pi / 2), received_at=10.0)
        est.calibrate((0.5, 0.4, 0.0))
        self.assertTrue(np.allclose(est.pose(), (0.5, 0.4, 0.0)))
        # Robot drives 1 m forward: odom +y (its heading is +90° in odom), map +x.
        est.update(tel(5.0, -1.0, math.pi / 2), received_at=10.5)
        self.assertTrue(np.allclose(est.pose(), (1.5, 0.4, 0.0), atol=1e-9))

    def test_status_transitions(self):
        cfg = LocalizationConfig(max_sigma_m=0.1, drift_per_m=0.05, sigma0_m=0.02)
        est = PoseEstimator(cfg)
        self.assertEqual(est.status(0)[0], 'STALE')
        est.update(tel(0, 0, 0), 1.0)
        self.assertEqual(est.status(1.0)[0], 'UNCALIBRATED')
        est.calibrate((1, 1, 0))
        self.assertEqual(est.status(1.1)[0], 'OK')
        self.assertEqual(est.status(2.0)[0], 'STALE')
        for i in range(1, 21):  # 2 m travelled -> sigma 0.12 > 0.1
            est.update(tel(i * 0.1, 0, 0), 1.0 + i * 0.1)
        self.assertEqual(est.status(3.0)[0], 'LOW_CONFIDENCE')
        est.calibrate((1, 1, 0))
        self.assertEqual(est.status(3.0)[0], 'OK')
        est.update(tel(5.0, 0, 0), 3.1)  # odometry jump / reset
        self.assertEqual(est.status(3.1)[0], 'LOST')

    def test_rejects_non_finite_and_uses_imu_heading(self):
        est = PoseEstimator(LocalizationConfig(heading_source='imu'))
        est.update({'odom': dict(x=0, y=0, yaw=0.0), 'imu': dict(yaw=1.0)}, 1.0)
        est.calibrate((0, 0, 0.5))
        est.update({'odom': dict(x=0, y=0, yaw=0.3), 'imu': dict(yaw=1.2)}, 1.1)
        self.assertAlmostEqual(est.pose()[2], 0.7)
        est.update(tel(float('nan'), 0, 0), 1.2)
        self.assertEqual(est.status(1.2)[0], 'LOST')


class ScaleCalibrationTests(unittest.TestCase):
    def test_scale_applies_to_displacement_from_anchor(self):
        est = PoseEstimator()
        est.update(tel(0, 0, 0), 1.0)
        est.calibrate((0.5, 0.5, 0.0))
        est.update(tel(1.0, 0.0, 0.0), 1.1)
        self.assertTrue(np.allclose(est.pose(), (1.5, 0.5, 0.0)))
        est.set_scale(1.1, 0.9)
        self.assertTrue(np.allclose(est.pose(), (1.6, 0.5, 0.0)))       # 1.0 m measured -> 1.1 m on the map
        self.assertTrue(np.allclose(est.raw_pose(), (1.5, 0.5, 0.0)))   # raw stays unscaled for calibration
        with self.assertRaises(ValueError):
            est.set_scale(2.0, 1.0)

    def test_compute_axis_scale(self):
        from ugv_nav.localization import compute_axis_scale
        scale, info = compute_axis_scale('x', (0.3, 0.3), (2.5, 0.32), (0.3, 0.3), (2.6, 0.3))
        self.assertAlmostEqual(scale, 2.3 / 2.2)
        self.assertAlmostEqual(info['sideways_error_m'], -0.02)
        scale, _ = compute_axis_scale('y', (0.3, 0.3), (0.3, 2.0), (0.3, 0.3), (0.3, 1.9))
        self.assertAlmostEqual(scale, 1.6 / 1.7)
        for bad in [('x', (0, 0), (0.1, 0), (0, 0), (0.1, 0)),       # too short
                    ('x', (0, 0), (1.0, 0), (0, 0), (-1.0, 0)),      # opposite direction
                    ('x', (0, 0), (1.0, 0), (0, 0), (2.0, 0))]:      # absurd factor 2.0
            with self.assertRaises(ValueError):
                compute_axis_scale(*bad)


class PathProcessingTests(unittest.TestCase):
    def test_collinear_and_validation_on_real_maps(self):
        for name, (start, goal) in EXAMPLES.items():
            grid, _ = load_environment(name)
            cfg = PlannerConfig(8, Footprint(0.08, 0.02, True), clearance_weight=4.0)
            r = plan(grid, grid.world_to_cell(start), grid.world_to_cell(goal), cfg)
            if not r.found:
                continue
            simple = remove_collinear(r.waypoints)
            self.assertLess(len(simple), len(r.waypoints))
            self.assertAlmostEqual(path_length(simple), r.distance_m, places=9)
            self.assertTrue(validate_path(simple, grid, cfg.planning_radius)[0], name)
            short = shortcut(simple, grid, cfg.planning_radius)
            self.assertTrue(validate_path(short, grid, cfg.planning_radius)[0])
            self.assertLessEqual(path_length(short), path_length(simple) + 1e-9)

    def test_validation_rejects_blocked_path(self):
        a = np.zeros((10, 10)); a[5, :] = 1
        grid = Grid(a, 0.1)
        ok, msg = validate_path([(0.25, 0.25), (0.25, 0.85)], grid, 0.0)
        self.assertFalse(ok); self.assertIn('segment 0', msg)
        self.assertFalse(validate_path([], grid, 0)[0])
        self.assertFalse(validate_path([(float('nan'), 0)], grid, 0)[0])


def simulate(tracker, pose, dt=0.1, steps=2000, slip=1.0, freeze=False):
    """Unicycle kinematics; returns final status and poses."""
    poses = [pose]
    for k in range(steps):
        cmd = tracker.update(pose, k * dt, dt)
        if cmd.status in ('arrived', 'failed'):
            return cmd, poses
        if not freeze:
            x, y, yaw = pose
            pose = (x + cmd.vx * slip * math.cos(yaw) * dt, y + cmd.vx * slip * math.sin(yaw) * dt, wrap(yaw + cmd.wz * dt))
        poses.append(pose)
    return cmd, poses


class ControllerTests(unittest.TestCase):
    def test_follows_l_path_and_arrives(self):
        path = [(0, 0), (1, 0), (1, 1)]
        tracker = PathTracker(path, ControllerConfig())
        cmd, poses = simulate(tracker, (0, 0, math.pi))  # starts facing backwards
        self.assertEqual(cmd.status, 'arrived', cmd.message)
        self.assertLess(math.dist(poses[-1][:2], (1, 1)), 0.06)
        # never strays far from the path
        for x, y, _ in poses:
            d = min(abs(y) if x <= 1 else math.hypot(x - 1, y), abs(x - 1) if 0 <= y <= 1 else 9)
            self.assertLess(d, ControllerConfig().max_cross_track_m + 1e-6)

    def test_respects_limits_and_rotates_in_place_first(self):
        tracker = PathTracker([(0, 0), (0, 1)], ControllerConfig(max_v=0.1, max_w=0.5))
        cmd = tracker.update((0, 0, 0), 0, 0.1)  # path goes +y, robot faces +x: 90° error
        self.assertEqual(cmd.status, 'rotating'); self.assertEqual(cmd.vx, 0.0); self.assertGreater(cmd.wz, 0)
        cmd, poses = simulate(tracker, (0, 0, 0))
        self.assertEqual(cmd.status, 'arrived')

    def test_works_with_odometry_scale_error(self):
        tracker = PathTracker([(0, 0), (0.8, 0), (0.8, 0.6), (1.4, 0.6)])
        cmd, _ = simulate(tracker, (0, 0, 0), slip=0.8)
        self.assertEqual(cmd.status, 'arrived')

    def test_detects_no_progress(self):
        tracker = PathTracker([(0, 0), (1, 0)], ControllerConfig(progress_timeout_s=2.0))
        cmd, _ = simulate(tracker, (0, 0, 0), freeze=True)
        self.assertEqual(cmd.status, 'failed'); self.assertIn('progress', cmd.message)

    def test_detects_cross_track(self):
        tracker = PathTracker([(0, 0), (1, 0)])
        self.assertEqual(tracker.update((0.5, 0.3, 0), 0, 0.1).status, 'failed')

    def test_single_point_path(self):
        self.assertEqual(PathTracker([(1, 1)]).update((1.01, 1, 0), 0, 0.1).status, 'arrived')


class EdgeSafetyTests(unittest.TestCase):
    def test_known_walls_filtered_unknown_obstacles_block(self):
        from ugv_nav.safety_monitor import SafetyConfig, SafetyMonitor
        a = np.zeros((40, 40)); a[:, 28] = 1          # mapped wall at x = 0.84..0.87 m
        grid = Grid(a, 0.03)
        mon = SafetyMonitor(SafetyConfig(0.08, 0.07))
        pose = (0.60, 0.60, 0.0)                       # facing the wall, 0.24 m away (inside 0.18 m zone + 0.08 body)
        tel = {'scan': {'age': 0.0, 'points': [[0.245, 0.0]]}, 'safety_stop': None}
        ok = ('OK', '')
        self.assertTrue(mon.check(True, tel, ok, (0.1, 0, 0)))                     # unfiltered: wall blocks
        self.assertEqual(mon.check(True, tel, ok, (0.1, 0, 0), pose, grid, 0.02), [])  # mapped wall ignored
        tel['scan']['points'] = [[0.15, 0.0]]          # unmapped box 0.15 m ahead, far from any mapped wall
        self.assertTrue(mon.check(True, tel, ok, (0.1, 0, 0), (0.30, 0.60, 0.0), grid, 0.02))
        # Returns within sigma + known_tolerance_cells of a mapped wall count as that wall
        # (the UGV bridge's own unfiltered stop zone still protects against them).
        self.assertEqual(mon.unknown_points([(0.15, 0.0)], pose, grid, 0.02), [])
        self.assertIn('operator STOP latched', (setattr(mon, 'estop', True) or mon.check(True, tel, ok)))

    def test_stopping_distance_grows_with_speed(self):
        from ugv_bridge.safety import stopping_distance
        self.assertAlmostEqual(stopping_distance(0, 0.08, 0.3, 0.3), 0.08)
        self.assertAlmostEqual(stopping_distance(0.2, 0.08, 0.3, 0.3), 0.08 + 0.06 + 0.04 / 0.6)


class MapUpdateTests(unittest.TestCase):
    def test_marks_obstacles_in_map_frame(self):
        grid = Grid(np.zeros((20, 20)), 0.1)
        new, added = mark_scan_obstacles(grid, (1.0, 1.0, math.pi / 2), [(0.5, 0.0), (5.0, 0.0), (0.01, 0.0)],
                                         max_range_m=1.0, protect_radius_m=0.1)
        self.assertEqual(added, 1)
        self.assertTrue(new.cells[grid.world_to_cell((1.0, 1.5))])  # 0.5 m ahead while facing +y
        self.assertFalse(grid.cells.any())                            # original untouched
        same, zero = mark_scan_obstacles(new, (1.0, 1.0, math.pi / 2), [(0.5, 0.0)])
        self.assertEqual(zero, 0); self.assertIs(same, new)


if __name__ == '__main__':
    unittest.main()
