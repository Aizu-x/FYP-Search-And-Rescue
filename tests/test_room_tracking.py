"""LiDAR that looks OVER the arena (MentorPi): the room is the landmark.

Everything here uses a simulated room around the arena; the simulated LiDAR never sees an arena wall.
"""
import math
import time
import unittest

import numpy as np

from ugv_bridge.safety import scan_points
from ugv_planner import Footprint, PlannerConfig
from ugv_nav.controller import ControllerConfig
from ugv_nav.localization import LocalizationConfig, PoseEstimator, wrap
from ugv_nav.map_store import corner_pose, load_env
from ugv_nav.navigator import ARRIVED, RUNNING, SURE, Navigator, NavigatorConfig
from ugv_nav.room_map import RoomMap, RoomTracker, apply_mount, capped_distance
from ugv_nav.safety_monitor import SafetyConfig
from ugv_nav.scan_match import ScanMatcher
from ugv_nav.sim_backend import SimBackend, simulated_room
from ugv_nav.sim_session import SimulatedUGVSession

from nav_helpers import wait_for


def telemetry_points(sim, mount=(0.0, 0.0, 0.0), limit=180):
    """What the bridge would send: robot-frame points within 3 m, at most ``limit`` of them."""
    pts = scan_points(sim._scan(), mount, max_range=3.0, min_valid=0.05)
    step = max(1, int(math.ceil(len(pts) / float(limit))))
    return [(round(x, 3), round(y, 3)) for x, y in pts[::step]]


def standing_spots(grid):
    free = ~grid.cells.astype(bool)
    return [grid.cell_to_world((r, c)) for r in range(0, grid.shape[0], 2) for c in range(0, grid.shape[1], 2)
            if free[r, c]]


class RoomWorld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.occ, cls.gt = load_env('ENV1')
        cls.room = simulated_room(cls.gt)

    def sim(self, pose=(1.2, 0.42, 0.0), **options):
        options.setdefault('scan_noise_m', 0.01)
        return SimBackend(self.gt, pose, realtime=False, lidar_world=self.room, **options)

    def drawn_room(self, sim, tour):
        """Room drawing made by standing at the poses of ``tour`` (true poses)."""
        drawing = RoomMap(self.occ.extent)
        for pose in tour:
            sim.place(pose)
            drawing.integrate(pose, telemetry_points(sim))
        return drawing

    TOUR = [(1.2, 0.42, 0.0), (2.3, 0.45, 1.2), (2.5, 1.6, 2.0), (2.4, 2.5, -2.5), (1.2, 2.55, 0.4), (0.5, 2.3, -1.0),
            (0.45, 0.4, 0.8)]


class TheLidarLooksOverTheArena(RoomWorld):
    def test_simulated_lidar_sees_the_room_and_not_one_arena_wall(self):
        sim = self.sim(scan_noise_m=0.0)
        pts = telemetry_points(sim)
        self.assertGreater(len(pts), 80)
        arena = ScanMatcher(self.occ)                          # the old way: compare the scan with the arena map
        self.assertLess(arena.evaluate(sim.truth, pts)['matched'], 0.25)
        c, s = math.cos(sim.truth[2]), math.sin(sim.truth[2])
        inner = 0
        for px, py in pts:                                     # hits lie on the room walls / furniture ...
            wx, wy = sim.truth[0] + c * px - s * py, sim.truth[1] + s * px + c * py
            r, col = self.room.world_to_cell((wx, wy))
            near = self.room.cells[max(0, r - 1):r + 2, max(0, col - 1):col + 2].any()
            self.assertTrue(near, (wx, wy))
            inner += 0.2 < wx < 2.8 and 0.2 < wy < 2.8
        self.assertEqual(inner, 0)                             # ... never inside the arena

    def test_distance_transform_is_exact(self):
        rng = np.random.default_rng(1)
        occupied = rng.random((45, 60)) > 0.985
        occupied[20, 30] = True
        rows, cols = np.mgrid[0:45, 0:60]
        brute = np.full(occupied.shape, 1e9)
        for r, c in np.argwhere(occupied):
            brute = np.minimum(brute, np.hypot(rows - r, cols - c))
        self.assertTrue(np.allclose(capped_distance(occupied, 10), np.minimum(brute, 10), atol=1e-5))


class RoomDrawingTests(RoomWorld):
    def test_scan_fits_its_own_drawing_and_a_wrong_guess_is_corrected(self):
        sim = self.sim()
        drawing = self.drawn_room(sim, self.TOUR[:1] * 3)
        self.assertGreater(drawing.cells(), 120)
        quality = drawing.matcher().evaluate(sim.truth, telemetry_points(sim))
        self.assertGreater(quality['matched'], 0.9)
        for true in [(1.32, 0.46, 0.15), (1.6, 0.6, 0.5), (2.2, 0.5, 1.2)]:        # the robot drives on
            sim.place(true)
            pts = telemetry_points(sim)
            guess = (true[0] + 0.06, true[1] - 0.05, true[2] + math.radians(4))
            result = drawing.matcher().match(guess, pts, 0.09, math.radians(6), precise=True)
            self.assertTrue(result.ok, result.reason)
            self.assertLess(math.dist(result.pose[:2], true[:2]), 0.025)
            self.assertLess(abs(wrap(result.pose[2] - true[2])), math.radians(1.0))
            drawing.integrate(result.pose, pts)

    def test_someone_walking_by_never_becomes_a_wall_and_someone_standing_fades_out(self):
        sim = self.sim()
        drawing = self.drawn_room(sim, self.TOUR[:1] * 4)
        before = drawing.occupied().copy()
        rows, cols = np.mgrid[0:drawing.shape[0], 0:drawing.shape[1]]
        around = np.hypot(drawing.origin[0] + (cols + 0.5) * drawing.res - 1.2,
                          drawing.origin[1] + (rows + 0.5) * drawing.res + 0.05) < 0.18
        self.assertEqual(int((before & around).sum()), 0)
        sim.add_obstacle(1.2, -0.05, 0.10)                     # somebody steps next to the arena, 0.5 m from the robot
        self.assertLess(min(math.hypot(*p) for p in telemetry_points(sim)), 0.45)        # the LiDAR does see them
        drawing.integrate(sim.truth, telemetry_points(sim))
        self.assertEqual(int((drawing.occupied() & around).sum()), 0)                    # one look: not a wall
        for _ in range(2):                                     # they keep standing there
            drawing.integrate(sim.truth, telemetry_points(sim))
        self.assertGreater(int((drawing.occupied() & around).sum()), 0)                  # now they are in the drawing
        sim.obstacles.clear()                                  # ... and walk away
        for _ in range(8):
            drawing.integrate(sim.truth, telemetry_points(sim))
        self.assertEqual(int((drawing.occupied() & around).sum()), 0)                    # gone again
        self.assertGreater(int((drawing.occupied() & before).sum()), 0.9 * before.sum())    # the walls stayed

    def test_drawing_survives_being_saved(self):
        sim = self.sim()
        drawing = self.drawn_room(sim, self.TOUR[:3])
        again, meta = RoomMap.from_bytes(drawing.to_bytes(dict(mount=[0.04, 0.0, 0.03])))
        self.assertTrue(np.array_equal(again.occupied(), drawing.occupied()))
        self.assertTrue(np.array_equal(again.seen, drawing.seen))
        self.assertEqual(meta['mount'], [0.04, 0.0, 0.03])
        self.assertLess(len(drawing.to_bytes()), 60000)
        for junk in (b'', b'not a room', drawing.to_bytes()[:200]):
            with self.assertRaises(ValueError):
                RoomMap.from_bytes(junk)


class FindTheRobotTests(RoomWorld):
    def test_robot_is_found_anywhere_without_a_starting_guess(self):
        sim = self.sim()
        matcher = self.drawn_room(sim, self.TOUR).matcher()
        spots = standing_spots(self.occ)
        free = [self.occ.cell_to_world((r, c)) for r in range(self.occ.shape[0]) for c in range(self.occ.shape[1])
                if not self.occ.cells[r, c]]
        rng = np.random.default_rng(11)
        accepted = 0
        for _ in range(14):
            x, y = free[int(rng.integers(len(free)))]
            true = (x, y, float(rng.uniform(-math.pi, math.pi)))
            sim.place(true)
            result = matcher.locate(telemetry_points(sim), spots)
            if result.ok:                                       # an accepted answer must be the right one
                accepted += 1
                self.assertLess(math.dist(result.pose[:2], true[:2]), 0.04, (true, result.pose))
                self.assertLess(abs(wrap(result.pose[2] - true[2])), math.radians(2.0))
            else:                                               # otherwise the truth is among the offered places
                self.assertTrue(any(math.dist(c['pose'][:2], true[:2]) < 0.05 for c in result.candidates), result.reason)
        self.assertGreaterEqual(accepted, 11)

    def test_a_room_that_looks_the_same_both_ways_is_not_guessed(self):
        """Empty rectangular room, arena in its middle: every view fits twice (turned by 180 degrees)."""
        bare = simulated_room(self.gt, left=0.6, right=0.6, bottom=0.6, top=0.6, furniture=False)
        bare_cells = bare.cells.copy().astype(np.uint8)
        bare_cells[:2, :] = 1                                   # close the door
        from ugv_planner.grid import Grid
        bare = Grid(bare_cells, bare.resolution, bare.origin)
        sim = SimBackend(self.gt, (1.0, 0.9, 0.3), realtime=False, lidar_world=bare)
        drawing = RoomMap(self.occ.extent)
        for pose in self.TOUR:
            sim.place(pose)
            drawing.integrate(pose, telemetry_points(sim))
        sim.place((1.0, 0.9, 0.3))
        result = drawing.matcher().locate(telemetry_points(sim), standing_spots(self.occ))
        self.assertFalse(result.ok)
        self.assertGreaterEqual(len(result.candidates), 2)
        self.assertIn('choose', result.reason)
        mirror = (3.0 - 1.0, 3.0 - 0.9)                         # the same view from the opposite side of the room
        self.assertTrue(any(math.dist(c['pose'][:2], (1.0, 0.9)) < 0.05 for c in result.candidates))
        self.assertTrue(any(math.dist(c['pose'][:2], mirror) < 0.08 for c in result.candidates))

    def test_in_a_half_drawn_room_a_wrong_place_is_refused_not_guessed(self):
        """Only the bottom of the room was drawn. Elsewhere a few walls may happen to fit somewhere wrong. With
        the limits for placing the robot WITHOUT asking (strong fit, mostly known surroundings, no rays through
        drawn walls, no rival) such a place is never accepted; the doubtful cases go to the operator."""
        sim = self.sim(scan_noise_m=0.008)
        drawing = RoomMap(self.occ.extent)
        x = 0.36
        while x < 2.4:
            sim.place((x, 0.3, 0.0))
            drawing.integrate(sim.truth, telemetry_points(sim))
            x += 0.06
        matcher = drawing.matcher()
        spots = standing_spots(self.occ)
        free = [self.occ.cell_to_world((r, c)) for r in range(self.occ.shape[0]) for c in range(self.occ.shape[1])
                if not self.occ.cells[r, c]]
        rng = np.random.default_rng(3)
        right = refused = 0
        for _ in range(40):
            x, y = free[int(rng.integers(len(free)))]
            true = (x, y, float(rng.uniform(-math.pi, math.pi)))
            sim.place(true)
            pts = telemetry_points(sim)
            self.assertLess(matcher.crossing(true, pts), 0.06)             # at the true pose no ray crosses a wall
            result = matcher.locate(pts, spots, **SURE)
            if result.ok:
                right += 1
                self.assertLess(math.dist(result.pose[:2], true[:2]), 0.05, (true, result.pose, result.score))
            else:
                refused += 1
        self.assertGreater(right, 20)
        self.assertGreater(refused, 0)                                     # (far from the drawn part: honest "not sure")
        # a place that only looks similar: most rays would have to pass through drawn walls
        sim.place((2.835, 2.715, -1.17))
        pts = telemetry_points(sim)
        self.assertGreater(matcher.crossing((0.425, 0.6, 0.4), pts), 0.12)

    def test_too_little_in_view_is_not_an_answer(self):
        sim = self.sim()
        matcher = self.drawn_room(sim, self.TOUR[:1]).matcher()
        result = matcher.locate([(1.0, 0.0)] * 10, standing_spots(self.occ))
        self.assertFalse(result.ok)
        self.assertIn('too few', result.reason)


class LidarMountingTests(RoomWorld):
    """Nobody measured where the LiDAR sits on the robot. The tracker finds out from the robot's own motion and
    only then starts correcting the position."""

    def drive(self, mount, legs, odom_error=0.08):
        start = (1.2, 0.42, 0.0)
        sim = self.sim(start, lidar_mount=mount, odom_scale_error=odom_error)
        est = PoseEstimator(LocalizationConfig(drift_per_m=0.0, drift_per_rad_m=0.0))
        tracker = RoomTracker(self.occ.extent)
        clock = [0.0]
        notes, errors, corrected_early = [], [], []

        def feed():
            est.update(dict(odom=dict(x=sim.odom[0], y=sim.odom[1], yaw=sim.odom[2], age=0.0)), clock[0])

        def run(vx, wz, seconds):
            sim.send_velocity(vx, 0.0, wz)
            for _ in range(int(round(seconds / 0.01))):
                sim.advance(0.01)
                clock[0] += 0.01
                if int(round(clock[0] * 100)) % 10:
                    continue
                feed()
                still = abs(sim.vel[0]) < 0.02 and abs(sim.vel[2]) < 0.05
                was_active = tracker.active
                step = tracker.step(clock[0], est.pose(), apply_mount(telemetry_points(sim), tracker.mount), est.odom,
                                    sim.vel[0], sim.vel[2], 0.0, still)
                notes.extend(step.notes)
                if step.correct is not None:
                    if not was_active and not tracker.active:
                        corrected_early.append(clock[0])
                    est.correct(step.correct)
                tracker.estimator_is(est.pose())
                if tracker.active:
                    errors.append(math.dist(est.pose()[:2], sim.truth[:2]))
        feed()
        est.calibrate(start)
        tracker.restart(start, trusted=True)
        run(0, 0, 2.0)
        for _ in range(legs):
            run(0, 0.3, math.radians(90) / 0.3); run(0, 0, 1.2)
            run(0, -0.3, math.radians(90) / 0.3); run(0, 0, 1.2)
            run(0.12, 0, 0.6 / 0.12); run(0, 0, 1.0)
            run(0, 0.3, math.radians(180) / 0.3); run(0, 0, 1.2)
        self.assertFalse(sim.collided)
        return tracker, notes, errors, corrected_early

    def check(self, mount, legs=4):
        tracker, notes, errors, early = self.drive(mount, legs)
        self.assertTrue(tracker.calibrated, notes)
        self.assertEqual(early, [])                             # nothing was corrected before the mounting was verified
        self.assertLess(math.dist(tracker.mount[:2], mount[:2]), 0.012, tracker.mount)
        self.assertLess(abs(wrap(tracker.mount[2] - mount[2])), math.radians(1.2), tracker.mount)
        self.assertTrue(any('verified' in n for n in notes))
        self.assertGreater(len(errors), 100)
        self.assertLess(max(errors), 0.055)                     # odometry alone is 4.8 cm off after every 0.6 m here
        self.assertLess(sum(errors) / len(errors), 0.035)
        return notes

    def test_lidar_in_the_centre_needs_no_change(self):
        tracker, notes, errors, early = self.drive((0.0, 0.0, 0.0), 3)
        self.assertTrue(tracker.calibrated)
        self.assertEqual(tracker.mount, (0.0, 0.0, 0.0))
        self.assertFalse(any('measured' in n for n in notes))
        self.assertLess(max(errors), 0.05)

    def test_lidar_ahead_of_the_centre_and_slightly_turned(self):
        notes = self.check((0.045, 0.01, math.radians(3.0)))
        self.assertTrue(any('measured' in n for n in notes))
        self.assertFalse(any('WARNING' in n for n in notes))

    def test_lidar_mounted_backwards_is_noticed_and_reported(self):
        notes = self.check((-0.06, 0.01, math.radians(178.0)), legs=5)
        self.assertTrue(any('WARNING' in n and 'bridge_config.json' in n for n in notes))


class ClosedLoopRoomTests(unittest.TestCase):
    """Real bridge + protocol + simulated robot. Odometry reads 8 % too far and drifts in heading."""

    def setUp(self):
        self.occ, self.gt = load_env('ENV1')
        x, y, heading = corner_pose(self.occ, 'bl', 0, 0.11, 0.09, 0.10)
        self.start = (x, y, math.radians(heading))
        self.session = None

    def connect(self, lidar_correction=True, verified=True, mount=(0.0, 0.0, 0.0)):
        self.session = SimulatedUGVSession(self.gt, self.start, 0.11, 0.09, sim_kwargs=dict(
            odom_scale_error=0.08, yaw_drift_rad_per_s=0.002, lidar_world=simulated_room(self.gt),
            lidar_mount=mount, scan_noise_m=0.01))
        cfg = PlannerConfig(8, Footprint(0.143, 0.02, True), True, clearance_weight=4.0)
        self.nav = Navigator(self.session.client, self.occ, cfg, NavigatorConfig(
            controller=ControllerConfig(max_v=0.15, max_w=0.6), lidar_correction=lidar_correction,
            lidar_reference='room', localization=LocalizationConfig(), safety=SafetyConfig(0.11, 0.09)))
        self.nav.set_lidar_mount((0.0, 0.0, 0.0), verified)
        self.assertTrue(wait_for(self.session.client.link_ok, timeout=5))
        time.sleep(0.3)
        self.nav.run_in_background()

    def tearDown(self):
        if self.session is not None:
            self.nav.shutdown()
            self.session.close()

    def error(self):
        estimate, truth = self.nav.estimator.pose(), self.session.backend.truth_pose()
        return math.dist(estimate[:2], truth[:2])

    def drive(self, goal, timeout=120):
        self.nav.set_goal(goal)
        ok, message = self.nav.authorize()
        self.assertTrue(ok, message)
        worst, t0 = 0.0, time.monotonic()
        while self.nav.state == RUNNING and time.monotonic() - t0 < timeout:
            worst = max(worst, self.error())
            time.sleep(0.1)
        return worst

    def test_long_drive_stays_in_step_with_reality(self):
        self.connect()
        self.nav.set_initial_pose(self.start, source='dock')
        self.assertTrue(wait_for(lambda: self.nav.room.state == 'locked', timeout=6), self.nav.room.status())
        worst = 0.0
        for goal in [(2.4, 0.42), (2.6, 2.5), (1.2, 2.55), (2.5, 0.45)]:          # about 8.5 m, more than 3x the old limit
            worst = max(worst, self.drive(goal))
            self.assertEqual(self.nav.state, ARRIVED, self.nav.message)
            self.assertLess(self.nav.estimator.sigma, 0.06)                       # the drift estimate never builds up
        self.assertLess(worst, 0.07)
        self.assertLess(self.error(), 0.04)
        self.assertFalse(self.session.backend.collided)
        self.assertIsNotNone(self.nav.room_file())                                # worth keeping: corner start + verified

    def test_without_tracking_the_drift_limit_stops_the_robot(self):
        self.connect(lidar_correction=False)
        self.nav.set_initial_pose(self.start, source='dock')
        self.drive((2.4, 0.42))
        self.drive((2.6, 2.5))
        self.assertNotEqual(self.nav.state, ARRIVED)
        self.assertIn('localization', self.nav.message)
        self.assertGreater(self.error(), 0.08)                                    # what the tracking above prevents

    def test_carried_robot_is_found_again_and_a_new_session_needs_no_pin(self):
        self.connect()
        self.nav.set_initial_pose(self.start, source='dock')
        self.drive((2.4, 0.42))
        self.drive((2.5, 2.4))
        self.session.backend.place((1.5, 2.5, 2.0))                               # picked up and put down elsewhere
        self.assertTrue(wait_for(lambda: self.error() < 0.04, timeout=15, step=0.2), self.nav.message)
        self.assertIn('found again', ' '.join(text for _, text in self.nav.log))
        saved = self.nav.room_file()
        self.assertIsNotNone(saved)
        self.nav.shutdown(); self.session.close()
        # next day: the robot stands somewhere, nobody sets a pin
        self.start = (1.2, 2.55, -1.0)
        self.connect()
        self.assertIsNone(self.nav.estimator.pose())
        self.assertIn('room outline', self.nav.load_room(saved))
        self.assertTrue(wait_for(lambda: self.nav.estimator.pose() is not None, timeout=15, step=0.2))
        self.assertLess(self.error(), 0.04)
        self.assertLess(self.drive((2.6, 2.5)), 0.08)                             # and it can drive from there
        self.assertEqual(self.nav.state, ARRIVED, self.nav.message)

    def test_operator_pose_is_never_overridden_by_a_search_that_was_still_running(self):
        self.connect()
        self.nav.set_initial_pose(self.start, source='dock')
        self.drive((2.4, 0.42))
        saved = self.nav.room_file()
        self.assertIsNotNone(saved)
        self.nav.shutdown(); self.session.close()
        self.start = (1.6, 0.42, 0.0)                          # next session: the robot stands here
        self.connect()
        self.nav.load_room(saved)
        self.assertTrue(wait_for(lambda: self.nav._search is not None, timeout=6, step=0.02))   # looking for it ...
        operator = (1.9, 0.42, 0.0)                            # ... while the operator confirms a pose of his own
        self.nav.set_initial_pose(operator, source='manual')
        self.assertTrue(wait_for(lambda: self.nav._search is None, timeout=10, step=0.05))
        time.sleep(1.5)
        pose = self.nav.estimator.pose()
        self.assertLess(math.dist(pose[:2], operator[:2]), 0.02)                  # his word counts
        self.assertFalse(any('found' in text for _, text in self.nav.log))

    def test_unverified_mounting_is_watched_not_trusted(self):
        self.connect(verified=False, mount=(0.05, 0.0, math.radians(4.0)))
        self.nav.set_initial_pose(self.start, source='dock')
        self.drive((1.6, 0.42))
        self.assertFalse(self.nav.room.active)
        self.assertGreater(self.nav.estimator.sigma, 0.06)                        # odometry only: the drift estimate grows
        self.assertIsNone(self.nav.room_file())                                   # and nothing is remembered yet
        with self.assertRaises(ValueError):
            self.nav.load_room(b'junk')


if __name__ == '__main__':
    unittest.main()
