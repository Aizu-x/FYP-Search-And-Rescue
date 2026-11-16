"""Survivor search mission: turn-in-place tracker, survivor list rules, closed-loop search in the simulator."""
import importlib.util
import math
import threading
import time
import unittest
from types import SimpleNamespace


from ugv_planner import Footprint, PlannerConfig, load_environment
from ugv_nav.controller import ControllerConfig
from ugv_nav.localization import LocalizationConfig
from ugv_nav.mission import MissionConfig, SearchMission
from ugv_nav.navigator import RUNNING, STOPPED, Navigator, NavigatorConfig, SpinTracker
from ugv_nav.safety_monitor import SafetyConfig

from nav_helpers import wait_for

HAVE_CV2 = importlib.util.find_spec('cv2') is not None


class SpinTrackerTests(unittest.TestCase):
    def test_turns_through_headings_and_holds_still_at_each(self):
        cfg = ControllerConfig(max_w=0.6, min_w=0.15)
        spin = SpinTracker([0.0, math.pi / 2, math.pi], 0.5, cfg, max_rate=0.3)
        yaw, t, looks = 0.0, 0.0, 0
        for _ in range(2000):
            cmd = spin.update((1.0, 1.0, yaw), t, 0.05)
            self.assertEqual((cmd.vx, cmd.vy), (0.0, 0.0))          # never drives, only turns
            self.assertLessEqual(abs(cmd.wz), 0.3 + 1e-9)            # look-around speed limit
            if cmd.status == 'arrived':
                break
            looks += cmd.wz == 0.0
            yaw += cmd.wz * 0.05
            t += 0.05
        self.assertEqual(cmd.status, 'arrived')
        self.assertAlmostEqual(yaw, math.pi, delta=math.radians(5))
        self.assertGreater(looks, 25)                                # held still ~0.5 s at each of 3 headings

    def test_no_progress_fails(self):
        spin = SpinTracker([math.pi / 2], 0.5, ControllerConfig(progress_timeout_s=2.0))
        t = 0.0
        while t < 5:
            cmd = spin.update((0, 0, 0.0), t, 0.1)                  # robot does not turn (blocked)
            if cmd.status == 'failed':
                break
            t += 0.1
        self.assertEqual(cmd.status, 'failed')


class FakeNav:
    """Just enough of a Navigator for the survivor-list rules."""

    def __init__(self):
        self._lock = threading.RLock()
        self.grid, _ = load_environment('ENV1')
        self.clock = time.monotonic
        self.halt_seq = 0
        self.vision_obstacles = []
        self.estimator = SimpleNamespace(pose=lambda: (1.0, 1.0, 0.0))
        self.safety = SimpleNamespace(estop=False, cfg=SafetyConfig(0.11, 0.09))
        self.auto_replans = 0
        self.notes = []

    def _note(self, text):
        self.notes.append(text)


class SurvivorListTests(unittest.TestCase):
    def test_green_objects_become_survivors_once_red_never(self):
        nav = FakeNav()
        m = SearchMission(nav, MissionConfig(same_survivor_m=0.15))
        m.start(transit=False)
        nav.vision_obstacles = [dict(id=1, cls='green_obstacle', x=1.50, y=1.00),
                                dict(id=2, cls='red_obstacle', x=2.00, y=0.40)]
        m._register_survivors()
        self.assertEqual([(s.id, s.x, s.y) for s in m.survivors], [(1, 1.50, 1.00)])
        self.assertTrue(any('SURVIVOR S1' in n for n in nav.notes))
        # same object seen again (same track, refined position) and a new track 8 cm away: still ONE survivor
        nav.vision_obstacles = [dict(id=1, cls='green_obstacle', x=1.52, y=1.01),
                                dict(id=7, cls='green_obstacle', x=1.58, y=1.05)]
        m._register_survivors()
        self.assertEqual(len(m.survivors), 1)
        self.assertAlmostEqual(m.survivors[0].x, 1.52)
        # a second green object far away is a second survivor
        nav.vision_obstacles = [dict(id=9, cls='green_obstacle', x=2.60, y=1.70)]
        m._register_survivors()
        self.assertEqual([s.id for s in m.survivors], [1, 2])
        # the tracker forgetting an object does not remove the survivor
        nav.vision_obstacles = []
        m._register_survivors()
        self.assertEqual(len(m.survivors), 2)

    def test_coverage_counts_only_reachable_floor_in_view(self):
        nav = FakeNav()
        nav.client = SimpleNamespace(snapshot=lambda: dict(telemetry=None))
        m = SearchMission(nav, MissionConfig(view_range_m=1.0, fov_deg=60))
        m.start(transit=False)
        self.assertEqual(m.coverage(), 0.0)
        m._update_coverage((1.0, 1.0, 0.0), time.monotonic())
        seen = m.seen
        self.assertTrue(seen[nav.grid.world_to_cell((1.6, 1.0))])      # 0.6 m straight ahead
        self.assertFalse(seen[nav.grid.world_to_cell((0.6, 1.0))])     # behind the robot
        self.assertFalse(seen[nav.grid.world_to_cell((1.0, 1.8))])     # outside the field of view
        self.assertGreater(m.coverage(), 0.02)
        self.assertLess(m.coverage(), 0.3)


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class ClosedLoopSearchTests(unittest.TestCase):
    """Real bridge + simulated robot + simulated camera; blocks are seen by the camera only."""

    START = (1.215, 0.405, 0.0)

    def setUp(self):
        from ugv_nav.sim_session import SimulatedUGVSession
        from ugv_nav.vision import CameraModel, ObstacleTracker, VisionWorker
        from ugv_nav.vision_sim import SimBlocks, SimCamera, SimDetector
        self.occ, gt = load_environment('ENV1')
        self.session = SimulatedUGVSession(gt, self.START, 0.08, 0.07)
        cam = CameraModel(pitch_rad=math.radians(15), x_offset_m=0.08)
        self.blocks = SimBlocks()
        self.session.backend.blocks = self.blocks
        self.session.backend.camera_source = SimCamera(self.session.backend, self.blocks, cam)
        self.assertTrue(wait_for(self.session.client.link_ok, timeout=5)); time.sleep(0.2)
        cfg = PlannerConfig(8, Footprint(math.hypot(0.08, 0.07), 0.02, True), True, clearance_weight=4.0)
        self.nav = Navigator(self.session.client, self.occ, cfg, NavigatorConfig(
            controller=ControllerConfig(max_v=0.2, max_w=1.0, min_w=0.2),
            localization=LocalizationConfig(drift_per_m=0.0, drift_per_rad_m=0.0, max_sigma_m=0.2),
            safety=SafetyConfig(0.08, 0.07), auto_continue_delay_s=0.3))
        self.nav.set_initial_pose(self.START)
        self.mission = SearchMission(self.nav, MissionConfig(scan_step_deg=90, scan_dwell_s=0.8, scan_turn_rate=0.8,
                                                             secure_hold_s=1.0, end_when_secured=False))
        self.nav.mission = self.mission
        self.nav.run_in_background()
        self.session.client.request_camera(10)
        self.worker = VisionWorker(self.session.client, lambda: self.nav,
                                   SimDetector(self.session.backend, self.blocks, cam), cam, ObstacleTracker()).start()

    def tearDown(self):
        self.mission.active = False
        self.worker.stop(); self.nav.shutdown(); self.session.close()

    def test_search_finds_and_reaches_green_survivors_until_stopped(self):
        self.blocks.add(1.50, 1.00, 'green_obstacle')
        self.blocks.add(2.60, 1.70, 'green_obstacle')
        self.blocks.add(1.95, 0.30, 'red_obstacle')                # obstacle, not a survivor
        self.nav.set_goal((1.80, 0.405))
        self.assertTrue(self.nav.authorize()[0])
        self.mission.start(transit=True)

        def both_reached():
            return len(self.mission.survivors) >= 2 and all(s.status == 'secured' for s in self.mission.survivors)
        self.assertTrue(wait_for(both_reached, timeout=150, step=0.2),
                        (self.nav.message, [(s.id, s.x, s.y, s.status) for s in self.mission.survivors]))
        found = sorted((round(s.x, 1), round(s.y, 1)) for s in self.mission.survivors)
        self.assertEqual(found, [(1.5, 1.0), (2.6, 1.7)])            # exactly the green blocks, within ~5 cm
        self.assertFalse(self.session.backend.collided)
        robot = self.session.backend.truth_pose()
        self.assertLess(math.dist(robot[:2], (2.60, 1.70)), 0.55)    # stopped in front of the last survivor
        self.assertGreater(math.dist(robot[:2], (2.60, 1.70)), 0.25)
        # the mission does not end by itself
        time.sleep(2.0)
        self.assertTrue(self.mission.active and not self.mission.paused)
        # operator STOP pauses it; nothing moves until the operator resumes
        self.nav.emergency_stop()
        self.assertTrue(self.mission.paused)
        time.sleep(1.5)
        self.assertNotEqual(self.nav.state, RUNNING)
        self.assertFalse(self.session.server.armed)
        self.assertTrue(self.mission.resume()[0])
        self.assertTrue(wait_for(lambda: self.nav.state == RUNNING, timeout=10), self.nav.message)
        # Stop mission: ends it, robot stopped and STOP latched
        self.mission.stop()
        self.assertFalse(self.mission.active)
        self.assertEqual(self.nav.state, STOPPED)
        self.assertTrue(self.nav.safety.estop)
        time.sleep(1.0)
        self.assertNotEqual(self.nav.state, RUNNING)
        self.assertEqual(len(self.mission.survivors), 2)             # list is kept

    def test_first_green_is_approached_held_then_secured_and_the_mission_ends(self):
        """Operator's rule: go to the first green object seen, stand there 3 s, mark it secured, end."""
        self.mission.cfg = MissionConfig(scan_step_deg=90, scan_dwell_s=0.8, scan_turn_rate=0.8,
                                         approach_gap_m=0.20, secure_hold_s=3.0)
        self.blocks.add(1.50, 1.00, 'green_obstacle')
        self.blocks.add(1.95, 0.30, 'red_obstacle')                # never a survivor
        self.mission.start(transit=False)
        self.assertTrue(wait_for(lambda: self.mission.complete, timeout=120, step=0.2),
                        (self.nav.message, [(s.id, s.x, s.y, s.status) for s in self.mission.survivors]))
        self.assertFalse(self.mission.active)
        self.assertEqual([s.status for s in self.mission.survivors], ['secured'])          # ONE green, not two
        s = self.mission.survivors[0]
        self.assertLess(math.dist((s.x, s.y), (1.50, 1.00)), 0.06)
        robot = self.session.backend.truth_pose()
        gap = math.dist(robot[:2], (1.50, 1.00)) - 0.08 - 0.025    # robot front to the block face
        self.assertLess(abs(gap - 0.20), 0.10)                     # stopped at the chosen distance
        events = [text for _, text in self.mission.events]
        held = [t for t, text in self.mission.events if 'In position at survivor' in text]
        done = [t for t, text in self.mission.events if 'SECURED' in text]
        self.assertTrue(held and done, events)
        self.assertGreaterEqual(done[0] - held[0], 3.0)            # stood in front of it for the hold time
        self.assertFalse(self.nav.safety.estop)                    # nothing latched: a new route can be planned
        self.assertFalse(self.session.backend.collided)
        time.sleep(1.0)
        self.assertNotEqual(self.nav.state, RUNNING)               # and it stays stopped

    def test_link_or_localization_loss_pauses_mission(self):
        self.mission.start(transit=False)
        self.assertTrue(wait_for(lambda: self.nav.state == RUNNING, timeout=10), self.nav.message)
        self.nav.estimator.fault = 'test: odometry jumped'            # localization lost while turning
        self.assertTrue(wait_for(lambda: self.mission.paused, timeout=5), self.nav.message)
        self.assertIn('localization', self.mission.pause_reason)
        time.sleep(1.5)
        self.assertNotEqual(self.nav.state, RUNNING)                 # waits for the operator


class NavigatorStopKindTests(unittest.TestCase):
    def test_operator_stop_is_never_cleared_by_automatic_actions(self):
        from ugv_nav.navigator import Navigator
        nav = Navigator(SimpleNamespace(stop=lambda: None, snapshot=lambda: dict(telemetry=None, telemetry_age=None)),
                        load_environment('ENV1')[0], PlannerConfig(8, Footprint(0.15, 0.02, True), True))
        nav.emergency_stop()
        self.assertEqual(nav.stop_kind, 'operator')
        ok, msg = nav._arm('auto', 'spin', SpinTracker([0.0], 0.1, ControllerConfig()))
        self.assertFalse(ok)
        self.assertIn('STOP', msg)
        self.assertTrue(nav.safety.estop)


if __name__ == '__main__':
    unittest.main()


class StallCheckTests(unittest.TestCase):
    """Odometry computed from commands keeps 'moving' when the motors do nothing: the LiDAR must notice."""

    def make(self):
        nav = Navigator(SimpleNamespace(stop=lambda: None), load_environment('ENV1')[0],
                        PlannerConfig(8, Footprint(0.15, 0.02, True), True))
        return nav

    @staticmethod
    def room(dx=0.0):
        """LiDAR points of a 2 x 1.4 m room seen from a robot that has driven dx forward."""
        pts = []
        for i in range(180):
            a = -math.pi + i * 2 * math.pi / 180
            c, s = math.cos(a), math.sin(a)
            t = min(t for t in ((1.0 - dx) / c if c > 1e-9 else 9, (-1.0 - dx) / c if c < -1e-9 else 9,
                                0.7 / s if s > 1e-9 else 9, -0.7 / s if s < -1e-9 else 9))
            pts.append([t * c, t * s])
        return dict(scan=dict(points=pts))

    def test_odometry_moving_but_lidar_unchanged_stops(self):
        nav = self.make()
        self.assertIsNone(nav._not_really_moving(self.room()))
        nav.estimator.odometer_m = 0.35                             # odometry "drove" 35 cm
        reason = nav._not_really_moving(self.room())                # ... but the view is identical
        self.assertIn('NOT really moving', reason)

    def test_really_moving_is_fine(self):
        nav = self.make()
        nav._not_really_moving(self.room())
        nav.estimator.odometer_m = 0.35
        self.assertIsNone(nav._not_really_moving(self.room(0.35)))
