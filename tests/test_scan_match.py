"""LiDAR scan <-> map matching: quality number, operator re-localization, optional continuous correction.

Simulation only (the simulated LiDAR ray-casts the GT map; matching uses the occupancy map).
"""
import math
import time
import unittest

import numpy as np

from ugv_planner import Grid, load_environment
from ugv_bridge.safety import scan_points
from ugv_nav.localization import LocalizationConfig, wrap
from ugv_nav.navigator import ARRIVED, RUNNING, Navigator, NavigatorConfig
from ugv_nav.scan_match import ScanMatcher
from ugv_nav.sim_backend import SimBackend
from ugv_nav.sim_session import SimulatedUGVSession

from nav_helpers import wait_for
from test_nav_integration import FAST, HALF_L, HALF_W, make_nav


def scan_at(world, pose, max_range=3.0, noise=0.008, seed=0):
    rng = np.random.default_rng(seed)
    scan = SimBackend(world, pose, realtime=False).latest().scan
    pts = scan_points(scan, (0.0, 0.0, 0.0), max_range=max_range, min_valid=0.05)[::2]
    return [(x + rng.normal(0, noise), y + rng.normal(0, noise)) for x, y in pts]


class MatcherTests(unittest.TestCase):
    def test_recovers_a_wrong_pose(self):
        occ, gt = load_environment('ENV1')
        matcher = ScanMatcher(occ)
        for k, true in enumerate([(1.215, 0.405, 0.0), (2.6, 1.2, 1.9), (0.9, 2.5, -2.4), (1.6, 1.6, 0.7)]):
            pts = scan_at(gt, true, seed=k)
            self.assertGreater(matcher.evaluate(true, pts)['matched'], 0.9)        # the scan fits at the truth
            wrong = (true[0] + 0.08, true[1] - 0.06, true[2] + math.radians(6))
            self.assertLess(matcher.evaluate(wrong, pts)['matched'], 0.8)          # ... and visibly not here
            result = matcher.match(wrong, pts)
            self.assertTrue(result.ok, result.reason)
            self.assertLess(math.dist(result.pose[:2], true[:2]), 0.03)
            self.assertLess(abs(math.degrees(wrap(result.pose[2] - true[2]))), 2.0)
            self.assertGreater(result.score, result.start_score)

    def test_featureless_corridor_is_ambiguous(self):
        cells = np.zeros((40, 400), np.uint8)
        cells[0:3, :] = 1; cells[-3:, :] = 1                         # two long parallel walls, ends out of range
        corridor = Grid(cells)
        true = (6.0, 0.6, 0.0)
        pts = scan_at(corridor, true, max_range=2.0)
        result = ScanMatcher(corridor).match((true[0] + 0.06, true[1] + 0.04, true[2]), pts)
        self.assertFalse(result.ok)
        self.assertIn('ambiguous', result.reason)

    def test_too_few_points_or_bad_fit_is_refused(self):
        occ, gt = load_environment('ENV1')
        matcher = ScanMatcher(occ)
        self.assertIsNone(matcher.evaluate((1.2, 0.4, 0.0), [(0.5, 0.0)] * 5))
        self.assertFalse(matcher.match((1.2, 0.4, 0.0), []).ok)
        elsewhere = scan_at(gt, (2.6, 2.6, 2.0))                      # a scan from a completely different place
        far = matcher.match((0.6, 0.4, 0.0), elsewhere)
        self.assertFalse(far.ok)


class NavigatorRelocalizeTests(unittest.TestCase):
    START = (1.215, 0.405, 0.0)

    def setUp(self):
        self.occ, self.gt = load_environment('ENV1')
        self.session = SimulatedUGVSession(self.gt, self.START, HALF_L, HALF_W)
        self.assertTrue(wait_for(self.session.client.link_ok, timeout=5)); time.sleep(0.2)
        self.nav = None

    def tearDown(self):
        if self.nav:
            self.nav.shutdown()
        self.session.close()

    def test_operator_relocalize_fixes_a_wrong_pose_estimate(self):
        self.nav = make_nav(self.session, self.occ)
        believed = (self.START[0] + 0.07, self.START[1] - 0.05, math.radians(5.0))     # operator placed it badly
        self.nav.set_initial_pose(believed)
        quality = self.nav.map_match()
        self.assertLess(quality['matched'], 0.85)
        ok, msg = self.nav.relocalize_with_lidar()
        self.assertTrue(ok, msg)
        pose = self.nav.snapshot()['pose']
        self.assertLess(math.dist(pose[:2], self.START[:2]), 0.03)
        self.assertLess(abs(math.degrees(wrap(pose[2] - self.START[2]))), 2.0)
        self.assertGreater(self.nav.map_match()['matched'], 0.9)
        self.assertFalse(self.session.server.armed)                   # never moves the robot
        # refused while driving
        self.nav.set_goal((2.205, 0.405))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: self.nav.state == RUNNING, timeout=3))
        self.assertFalse(self.nav.relocalize_with_lidar()[0])

    def test_refused_without_a_confirmed_pose(self):
        self.nav = make_nav(self.session, self.occ)
        ok, msg = self.nav.relocalize_with_lidar()
        self.assertFalse(ok)
        self.assertIn('Confirm', msg)

    def drive_far(self, lidar_correction):
        """A drive of more than 3 m with 8 % odometry scale error + yaw drift and the DEFAULT drift limits
        (0.05 m per metre, at most 0.15 m)."""
        self.session.backend.scale_err = 0.08
        self.session.backend.yaw_drift = 0.004
        self.nav = Navigator(self.session.client, self.occ, make_nav(self.session, self.occ).planner_config,
                             NavigatorConfig(controller=FAST, localization=LocalizationConfig(),
                                             safety=make_nav(self.session, self.occ).cfg.safety,
                                             lidar_correction=lidar_correction, lidar_correction_period_s=0.5))
        self.nav.set_initial_pose(self.START)
        self.assertTrue(self.nav.set_goal((0.9, 2.5)).found)
        self.assertGreater(self.nav.plan_info['length_m'], 3.0)
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        wait_for(lambda: self.nav.state != RUNNING, timeout=120, step=0.1)
        state, message = self.nav.state, self.nav.message
        at_stop = math.dist(self.nav.snapshot()['pose'][:2], self.session.backend.truth_pose()[:2])
        time.sleep(2.0)                                               # standing still: corrections settle
        settled = math.dist(self.nav.snapshot()['pose'][:2], self.session.backend.truth_pose()[:2])
        return state, message, (at_stop, settled)

    def test_without_correction_the_drift_limit_stops_a_long_drive(self):
        state, message, _ = self.drive_far(lidar_correction=False)
        self.assertNotEqual(state, ARRIVED)
        self.assertIn('LOW_CONFIDENCE', message)

    def test_continuous_correction_keeps_the_pose_true_on_a_long_drive(self):
        state, message, (at_stop, settled) = self.drive_far(lidar_correction=True)
        self.assertEqual(state, ARRIVED, message)
        self.assertLess(at_stop, 0.06)                                # estimated vs true position on arrival
        self.assertLess(settled, 0.03)                                # ... and after standing still for 2 s
        self.assertFalse(self.session.backend.collided)
        self.assertTrue(self.nav.lidar_fix['ok'])


if __name__ == '__main__':
    unittest.main()
