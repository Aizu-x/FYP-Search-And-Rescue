"""Closed-loop software tests: edge A* + controller <-> real bridge/protocol <-> simulated robot.

These exercise the exact code that will run on the robot, with a simulated
backend. They prove the software behaves as designed; they do NOT prove
physical navigation works.
"""
import math
import time
import unittest

from ugv_planner import Footprint, PlannerConfig, load_environment
from ugv_planner.collision import CollisionChecker
from ugv_nav.controller import ControllerConfig
from ugv_nav.localization import LocalizationConfig
from ugv_nav.navigator import ARRIVED, IDLE, PLANNED, RUNNING, STOPPED, Navigator, NavigatorConfig
from ugv_nav.safety_monitor import SafetyConfig
from ugv_nav.sim_session import SimulatedUGVSession

from nav_helpers import wait_for

HALF_L, HALF_W = 0.08, 0.07
FAST = ControllerConfig(max_v=0.25, max_w=1.0, min_w=0.2)


def make_nav(session, grid, controller=FAST, **kwargs):
    cfg = PlannerConfig(8, Footprint(math.hypot(HALF_L, HALF_W), 0.02, True), True, clearance_weight=4.0)
    nav_cfg = NavigatorConfig(controller=controller,
                              localization=LocalizationConfig(drift_per_m=0.02, max_sigma_m=0.2),
                              safety=SafetyConfig(HALF_L, HALF_W), **kwargs)
    return Navigator(session.client, grid, cfg, nav_cfg)


class ClosedLoopTests(unittest.TestCase):
    def setUp(self):
        self.occ, self.gt = load_environment('ENV1')
        self.start = (0.345, 0.315, 0.0)
        self.session = SimulatedUGVSession(self.gt, self.start, HALF_L, HALF_W)
        self.assertTrue(wait_for(self.session.client.link_ok, timeout=5))
        time.sleep(0.2)
        self.nav = None

    def tearDown(self):
        if self.nav:
            self.nav.shutdown()
        self.session.close()

    def run_until_done(self, timeout=90):
        self.nav.run_in_background()
        wait_for(lambda: self.nav.state != RUNNING, timeout=timeout, step=0.1)
        return self.nav.snapshot()

    def test_no_motion_without_authorization(self):
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.2, 0.9))
        self.assertEqual(self.nav.state, PLANNED)
        self.nav.run_in_background()
        time.sleep(1.0)
        self.assertEqual(self.session.backend.truth_pose(), self.start)
        self.assertFalse(self.session.server.armed)

    def test_navigates_astar_route_with_odometry_error(self):
        # Odometry has 3 % scale error and slow yaw drift; the robot starts misaligned with the path.
        self.session.backend.scale_err = 0.03
        self.session.backend.yaw_drift = 0.002
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        goal = (1.395, 1.335)
        result = self.nav.set_goal(goal)
        self.assertTrue(result.found)
        ok, msg = self.nav.authorize()
        self.assertTrue(ok, msg)
        snap = self.run_until_done()
        self.assertEqual(snap['state'], ARRIVED, snap['message'])
        truth = self.session.backend.truth_pose()
        self.assertLess(math.dist(truth[:2], goal), 0.15)       # true error includes odometry error
        self.assertLess(math.dist(snap['pose'][:2], goal), 0.06)  # estimate within goal tolerance
        self.assertFalse(self.session.backend.collided)
        # True trajectory never touches GT obstacles with the body radius.
        checker = CollisionChecker(self.gt)
        hist = self.session.backend.truth_history
        self.assertTrue(all(checker.is_clear(a[:2], b[:2], max(HALF_L, HALF_W)) for a, b in zip(hist, hist[1:])))
        self.assertTrue(wait_for(lambda: not self.session.server.armed, timeout=1))

    def test_unexpected_obstacle_stops_and_replans_on_edge(self):
        self._obstacle_scenario()  # auto-continue OFF (default)
        self.assertTrue(wait_for(lambda: self.nav.state != RUNNING, timeout=40, step=0.1))
        snap = self.nav.snapshot()
        self.assertEqual(snap['state'], PLANNED, snap['message'])     # new A* route found on the edge
        self.assertIn('authorize to continue', snap['message'])
        self.assertNotEqual(self.first_path, snap['path'])
        self.assertFalse(self.session.backend.collided)
        self.assertFalse(self.session.server.armed)
        time.sleep(0.5)  # allow the modelled motor lag to settle (a few mm of coasting)
        pos = self.session.backend.truth_pose()
        time.sleep(1.5)
        self.assertLess(math.dist(self.session.backend.truth_pose()[:2], pos[:2]), 1e-3)  # waits for the operator
        self.assertGreater(self.BOX[0] - self.BOX[2] - pos[0], HALF_L)  # stopped before reaching the box

    def test_blocked_corridor_without_detour_stays_stopped(self):
        # Box in a 0.4 m corridor: no route exists around it for this robot, so it must stay stopped.
        self.nav = make_nav(self.session, self.occ, replan_on_obstacle=True, auto_continue_after_replan=True)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 0.315))
        self.session.backend.add_obstacle(0.95, 0.315, 0.06)
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: 'no alternative route' in self.nav.message, timeout=40))
        time.sleep(1.5)
        self.assertNotEqual(self.nav.state, RUNNING)
        self.assertFalse(self.session.server.armed)
        self.assertFalse(self.session.backend.collided)

    OPEN_START, OPEN_GOAL, BOX = (1.215, 0.405, 0.0), (2.205, 0.405), (1.70, 0.405, 0.06)

    def _obstacle_scenario(self, **kwargs):
        """Open area of ENV1 where a detour around an unmapped box exists."""
        self.session.backend.place(self.OPEN_START); time.sleep(0.2)
        self.nav = make_nav(self.session, self.occ, replan_on_obstacle=True, **kwargs)
        self.nav.set_initial_pose(self.OPEN_START)
        self.nav.set_goal(self.OPEN_GOAL)
        self.first_path = list(self.nav.path)
        self.session.backend.add_obstacle(*self.BOX)  # box dropped on the straight route (not in the map)
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()

    def test_auto_continue_drives_around_obstacle_to_goal(self):
        self._obstacle_scenario(auto_continue_after_replan=True, auto_continue_delay_s=0.5)
        reached = wait_for(lambda: self.nav.state == ARRIVED, timeout=90, step=0.1)
        snap = self.nav.snapshot()
        self.assertTrue(reached, snap['message'])
        self.assertGreaterEqual(snap['auto_replans'], 1)          # it really replanned and continued itself
        self.assertIn('AUTO-CONTINUE', ' '.join(m for _, m in self.nav.log))
        self.assertFalse(self.session.backend.collided)
        self.assertLess(math.dist(self.session.backend.truth_pose()[:2], self.OPEN_GOAL), 0.12)
        self.assertTrue(wait_for(lambda: not self.session.server.armed, timeout=1))

    def test_auto_continue_respects_limit(self):
        self._obstacle_scenario(auto_continue_after_replan=True, max_auto_replans=0)
        self.assertTrue(wait_for(lambda: self.nav.state == PLANNED and 'limit' in self.nav.message, timeout=40))
        time.sleep(1.5)
        self.assertEqual(self.nav.state, PLANNED)
        self.assertFalse(self.session.server.armed)

    def test_operator_stop_cancels_pending_auto_continue(self):
        self._obstacle_scenario(auto_continue_after_replan=True, auto_continue_delay_s=3.0)
        self.assertTrue(wait_for(lambda: self.nav.snapshot()['auto_pending'], timeout=40))
        self.nav.emergency_stop()
        time.sleep(0.6)                      # planning is fast now: let the robot finish braking first
        pos = self.session.backend.truth_pose()
        time.sleep(4.0)
        self.assertEqual(self.nav.state, STOPPED)
        self.assertFalse(self.session.server.armed)
        self.assertLess(math.dist(self.session.backend.truth_pose()[:2], pos[:2]), 0.01)

    def test_scale_calibration_run_corrects_odometry_error(self):
        # Odometry over-reads distance by 10 %: without calibration the robot stops short of its goal.
        self.session.backend.scale_err = 0.10
        start, goal = (1.215, 0.405, 0.0), (2.205, 0.405)
        self.session.backend.place(start); time.sleep(0.2)
        self.nav = make_nav(self.session, self.occ, use_shortcut=True)
        self.nav.set_initial_pose(start)
        self.nav.set_goal(goal)
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: self.nav.state == ARRIVED, timeout=40, step=0.1), self.nav.message)
        time.sleep(0.4)
        raw_end = self.nav.snapshot()['raw_pose'][:2]
        true_end = self.session.backend.truth_pose()[:2]          # what the operator would measure with a tape
        self.assertGreater(math.dist(true_end, goal), 0.06)      # visibly short before calibration
        from ugv_nav.localization import compute_axis_scale
        scale_x, info = compute_axis_scale('x', start[:2], raw_end, start[:2], true_end)
        self.assertAlmostEqual(scale_x, 1 / 1.10, delta=0.01)
        # Apply and drive back: now the robot really arrives where the map says.
        self.nav.set_scale(scale_x, 1.0)
        self.nav.set_initial_pose((true_end[0], true_end[1], self.session.backend.truth_pose()[2]))
        self.nav.set_goal((1.215, 0.405))
        self.assertTrue(self.nav.authorize()[0])
        self.assertTrue(wait_for(lambda: self.nav.state == ARRIVED, timeout=60, step=0.1), self.nav.message)
        time.sleep(0.4)
        self.assertLess(math.dist(self.session.backend.truth_pose()[:2], (1.215, 0.405)), 0.06)

    def test_speed_fraction_limits_commands(self):
        self.nav = make_nav(self.session, self.occ)
        full = self.nav.base_controller.max_v
        v, w = self.nav.set_speed_fraction(0.3)
        self.assertAlmostEqual(v, 0.3 * full)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 0.315))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        peak = 0.0
        end = time.monotonic() + 3
        while time.monotonic() < end:
            peak = max(peak, abs(self.session.server.output[0])); time.sleep(0.02)
        self.assertGreater(peak, 0.0)
        self.assertLessEqual(peak, 0.3 * full + 1e-6)
        self.nav.set_speed_fraction(1.0)                           # change while driving
        end = time.monotonic() + 3
        while time.monotonic() < end and self.nav.state == RUNNING:
            peak = max(peak, abs(self.session.server.output[0])); time.sleep(0.02)
        self.assertGreater(peak, 0.3 * full + 0.01)
        with self.assertRaises(ValueError):
            self.nav.set_speed_fraction(1.5)

    def test_camera_block_in_corridor_then_removed_robot_continues(self):
        # 0.4 m corridor with no detour: a camera block stops the robot (no route); when the block is
        # removed the edge replans and, as the robot was driving and auto-continue is on, drives on.
        self.nav = make_nav(self.session, self.occ, auto_continue_after_replan=True, auto_continue_delay_s=0.3)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 0.315))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: self.session.backend.truth_pose()[0] > 0.45, timeout=10))
        self.nav.set_vision_obstacles([dict(id=1, cls='red_obstacle', x=0.95, y=0.315, size=0.05)])
        self.assertIn(self.nav.state, (IDLE, STOPPED))
        self.assertIn('no alternative route', self.nav.message)
        time.sleep(1.0)
        self.assertNotEqual(self.nav.state, RUNNING)               # waits while blocked
        self.nav.set_vision_obstacles([])                          # block taken away
        self.assertTrue(wait_for(lambda: self.nav.state == ARRIVED, timeout=40, step=0.1), self.nav.message)
        self.assertFalse(self.session.backend.collided)

    def test_removed_block_while_not_driving_waits_for_authorize(self):
        self.nav = make_nav(self.session, self.occ, auto_continue_after_replan=True, auto_continue_delay_s=0.3)
        self.nav.set_initial_pose(self.start)
        self.nav.set_vision_obstacles([dict(id=1, cls='red_obstacle', x=0.95, y=0.315, size=0.05)])
        self.nav.set_goal((1.395, 0.315))
        self.assertEqual(self.nav.state, IDLE)                     # no route at all
        self.nav.run_in_background()
        self.nav.set_vision_obstacles([])
        self.assertEqual(self.nav.state, PLANNED)                  # route found again ...
        time.sleep(1.5)
        self.assertEqual(self.nav.state, PLANNED)                  # ... but never drives without AUTHORIZE
        self.assertFalse(self.session.server.armed)

    def test_connection_loss_stops_robot_and_does_not_resume(self):
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 1.335))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: self.session.server.output != (0.0, 0.0, 0.0), timeout=3))
        # Simulate Wi-Fi loss: kill the client's socket from the UGV side.
        with self.session.server._lock:
            sock = self.session.server.client[0]
        sock.shutdown(2)
        self.assertTrue(wait_for(lambda: self.session.server.output == (0.0, 0.0, 0.0), timeout=1.0))
        self.assertTrue(wait_for(lambda: self.nav.state == STOPPED, timeout=2))
        # Client reconnects automatically, but the robot must stay disarmed and still.
        self.assertTrue(wait_for(lambda: self.session.client.connections >= 2 and self.session.client.link_ok(), timeout=8))
        pos = self.session.backend.truth_pose()
        time.sleep(1.0)
        self.assertFalse(self.session.server.armed)
        self.assertEqual(self.nav.state, STOPPED)
        self.assertLess(math.dist(self.session.backend.truth_pose()[:2], pos[:2]), 1e-3)

    def test_edge_crash_watchdog_stops_robot(self):
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 1.335))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        self.assertTrue(wait_for(lambda: self.session.server.output != (0.0, 0.0, 0.0), timeout=3))
        self.nav._stop_thread.set()  # edge control loop dies without sending STOP
        t0 = time.monotonic()
        self.assertTrue(wait_for(lambda: self.session.server.output == (0.0, 0.0, 0.0), timeout=1.0))
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertTrue(wait_for(lambda: not self.session.server.armed, timeout=2))

    def test_emergency_stop_latches(self):
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 1.335))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        time.sleep(0.8)
        self.nav.emergency_stop()
        self.assertTrue(wait_for(lambda: self.session.server.output == (0.0, 0.0, 0.0), timeout=0.5))
        self.assertEqual(self.nav.state, STOPPED)
        self.assertFalse(self.nav.authorize()[0])  # no plan any more -> must replan + authorize
        self.nav.set_goal((1.395, 1.335))
        ok, _ = self.nav.authorize()
        self.assertTrue(ok)  # explicit re-authorization clears the latch

    def test_localization_loss_stops(self):
        self.nav = make_nav(self.session, self.occ)
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((1.395, 1.335))
        self.assertTrue(self.nav.authorize()[0])
        self.nav.run_in_background()
        time.sleep(0.6)
        with self.session.backend._lock:
            x, y, t = self.session.backend.odom
            self.session.backend.odom = (x + 1.0, y, t)  # odometry reset/glitch
        self.assertTrue(wait_for(lambda: self.nav.state == STOPPED, timeout=2))
        self.assertIn('localization', self.nav.message)

    def test_refuses_invalid_requests(self):
        self.nav = make_nav(self.session, self.occ)
        with self.assertRaises(ValueError):
            self.nav.set_initial_pose((1.5, 1.08, 0.0) if self.occ.cells[self.occ.world_to_cell((1.5, 1.08))] else (-1, -1, 0))
        self.nav.set_goal((2.0, 2.0))
        self.assertEqual(self.nav.state, IDLE)  # uncalibrated -> cannot plan
        self.nav.set_initial_pose(self.start)
        self.nav.set_goal((0.015, 0.015))       # inside wall/inflation
        self.assertEqual(self.nav.state, IDLE)
        self.assertFalse(self.nav.authorize()[0])


class FootprintConsistencyTest(unittest.TestCase):
    def test_refuses_planning_radius_smaller_than_turning_envelope(self):
        occ, gt = load_environment('ENV1')
        session = SimulatedUGVSession(gt, (0.345, 0.315, 0.0), HALF_L, HALF_W)
        try:
            wait_for(session.client.link_ok, timeout=5); time.sleep(0.2)
            nav = Navigator(session.client, occ, PlannerConfig(8, Footprint(0.05, 0.0, True)),
                            NavigatorConfig(safety=SafetyConfig(HALF_L, HALF_W)))
            nav.set_initial_pose((0.345, 0.315, 0.0)); nav.set_goal((1.2, 0.9))
            ok, msg = nav.authorize()
            self.assertFalse(ok); self.assertIn('turning envelope', msg)
            self.assertFalse(session.server.armed)
            nav.cfg.allow_tight_radius = True          # operator explicitly accepts the tight radius
            ok, msg = nav.authorize()
            self.assertTrue(ok, msg)
            self.assertIn('tight radius', nav.message)
            nav.emergency_stop()
        finally:
            session.close()


class UnknownFootprintTest(unittest.TestCase):
    def test_physical_run_refused_without_radius(self):
        occ, gt = load_environment('ENV1')
        session = SimulatedUGVSession(gt, (0.345, 0.315, 0.0))
        try:
            wait_for(session.client.link_ok, timeout=5); time.sleep(0.2)
            nav = Navigator(session.client, occ, PlannerConfig(), NavigatorConfig(safety=SafetyConfig(HALF_L, HALF_W)))
            nav.set_initial_pose((0.345, 0.315, 0.0))
            nav.set_goal((1.2, 0.9))
            ok, msg = nav.authorize()
            self.assertFalse(ok); self.assertIn('radius', msg)
        finally:
            session.close()


if __name__ == '__main__':
    unittest.main()
