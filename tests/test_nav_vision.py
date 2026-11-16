"""Camera/YOLO pipeline: geometry, calibration, tracker, bridge camera stream, closed loop with A*."""
import math
import time
import unittest

import numpy as np

import importlib.util

HAVE_CV2 = importlib.util.find_spec('cv2') is not None

from ugv_planner import Footprint, PlannerConfig, load_environment
from ugv_nav.controller import ControllerConfig
from ugv_nav.localization import LocalizationConfig
from ugv_nav.navigator import ARRIVED, PLANNED, Navigator, NavigatorConfig
from ugv_nav.safety_monitor import SafetyConfig
from ugv_nav.vision import CameraModel, ObstacleTracker, TrackerConfig, block_centre_from_box, calibrate

from nav_helpers import wait_for


def face_pixel(cam, tx, ty, depth=0.05):
    """Bottom-edge pixel of a block whose centre is at robot-frame (tx, ty)."""
    dx, dy = tx - cam.x_offset_m, ty - cam.y_offset_m
    n = math.hypot(dx, dy)
    return cam.ground_to_pixel(tx - dx / n * depth / 2, ty - dy / n * depth / 2)


class GeometryTests(unittest.TestCase):
    def test_round_trip_with_real_lens(self):
        cam = CameraModel(pitch_rad=math.radians(15), x_offset_m=0.07)
        for x, y in [(0.3, 0.0), (0.7, 0.25), (1.3, -0.4)]:
            u, v = cam.ground_to_pixel(x, y)
            gx, gy = cam.pixel_to_ground(u, v)
            self.assertAlmostEqual(gx, x, places=4); self.assertAlmostEqual(gy, y, places=4)
        self.assertIsNone(cam.pixel_to_ground(320, 5))          # above the horizon (tilt 15 deg)

    def test_block_centre_from_box(self):
        cam = CameraModel(pitch_rad=math.radians(12))
        u, v = face_pixel(cam, 0.9, 0.1)
        x, y = block_centre_from_box((u - 15, v - 40, u + 15, v), cam)
        self.assertAlmostEqual(x, 0.9, places=3); self.assertAlmostEqual(y, 0.1, places=3)
        # very close block: floor contact below the image, position from the top edge (10 cm tall block)
        from ugv_nav.vision_sim import _box_for_block
        close = _box_for_block(cam, (0.0, 0.0, 0.0), 0.25, 0.02, 0.05, 0.10)
        self.assertGreaterEqual(close[3], cam.height - 1)
        x, y = block_centre_from_box(close, cam)
        self.assertLess(math.dist((x, y), (0.25, 0.02)), 0.02)
        self.assertIsNone(block_centre_from_box((0, 300, 30, 360), cam))                # cut off at the side

    def test_calibration_recovers_mounting(self):
        true = CameraModel(pitch_rad=math.radians(21), x_offset_m=0.11, yaw_offset_rad=math.radians(4))
        samples = []
        for tx, ty in [(0.45, 0.0), (0.75, -0.2), (1.05, 0.25)]:
            u, v = face_pixel(true, tx, ty)
            samples.append(((u - 15, v - 50, u + 15, v), (tx, ty)))
        before = block_centre_from_box(samples[1][0], CameraModel())
        self.assertGreater(math.dist(before, samples[1][1]), 0.05)      # default mounting is clearly wrong
        fit, rms = calibrate(samples, CameraModel())
        self.assertLess(rms, 0.005)
        # What matters: blocks at NEW positions (not used for fitting) are placed correctly.
        for tx, ty in [(0.6, 0.15), (0.9, -0.1), (1.2, 0.0)]:
            u, v = face_pixel(true, tx, ty)
            x, y = block_centre_from_box((u - 15, v - 50, u + 15, v), fit)
            self.assertLess(math.dist((x, y), (tx, ty)), 0.02)
        with self.assertRaises(ValueError):
            calibrate(samples[:1], CameraModel())


class ImpossiblePositionTests(unittest.TestCase):
    def test_detections_in_walls_or_outside_are_impossible(self):
        from ugv_nav.vision import _impossible_position
        grid, _ = load_environment('ENV1')
        self.assertTrue(_impossible_position(grid, (-0.2, 1.0)))     # outside the arena
        self.assertTrue(_impossible_position(grid, (1.0, 3.4)))
        self.assertTrue(_impossible_position(grid, (0.03, 1.5)))     # inside the thick outer wall
        self.assertFalse(_impossible_position(grid, (1.5, 0.5)))     # open floor


class TrackerTests(unittest.TestCase):
    def test_confirm_then_remove_only_when_in_view(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=3, remove_misses=4))
        det = [(1.0, 1.0, 'red_obstacle', 0.9)]
        self.assertFalse(tr.update(det, lambda x, y: True, 0.0))
        self.assertFalse(tr.update(det, lambda x, y: True, 0.1))
        self.assertTrue(tr.update(det, lambda x, y: True, 0.2))       # confirmed after 3 sightings
        self.assertEqual(len(tr.obstacles()), 1)
        for k in range(20):                                           # out of view: kept forever
            self.assertFalse(tr.update([], lambda x, y: False, 1 + k))
        self.assertEqual(len(tr.obstacles()), 1)
        changed = [tr.update([], lambda x, y: True, 30 + k) for k in range(4)]  # in view, nothing seen
        self.assertEqual(changed, [False, False, False, True])
        self.assertEqual(tr.obstacles(), [])

    def test_covered_obstacle_is_not_removed(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=2, remove_misses=3))
        for t in (0.0, 0.1):
            tr.update([(1.0, 1.0, 'red_obstacle', 0.9)], lambda x, y: True, t)
        for k in range(10):   # in view, position not computable (box cut off), but a red box covers it
            self.assertFalse(tr.update([], lambda x, y: True, 1 + k, covered=lambda x, y, c: c == 'red_obstacle'))
        self.assertEqual(len(tr.obstacles()), 1)

    def test_noise_and_low_confidence_ignored(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=3, confirm_window_s=1.0))
        tr.update([(1, 1, 'red_obstacle', 0.9)], lambda x, y: True, 0.0)   # single flash
        tr.update([(2, 2, 'red_obstacle', 0.2)], lambda x, y: True, 0.1)   # low confidence
        for t in (2.0, 2.1, 2.2):
            tr.update([], lambda x, y: True, t)
        self.assertEqual(tr.obstacles(), []); self.assertEqual(tr.tracks, {})

    def test_class_and_distance_association(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=2))
        for t in (0.0, 0.1):
            tr.update([(1.0, 1.0, 'red_obstacle', 0.9), (1.05, 1.0, 'green_obstacle', 0.9)], lambda x, y: True, t)
        self.assertEqual(sorted(o.cls for o in tr.obstacles()), ['green_obstacle', 'red_obstacle'])


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class RawColourEncodingTests(unittest.TestCase):
    def test_mentorpi_yuyv_frames_keep_their_colour(self):
        """The MentorPi publishes yuv422_yuy2; HiWonder's compressed topic is mono8 (no colour)."""
        import cv2
        from ugv_bridge.backends.ros2_backend import encode_raw_image
        img = np.full((480, 640, 3), 200, np.uint8)
        img[200:300, 100:200] = (0, 0, 255); img[200:300, 400:500] = (0, 180, 0)   # red and green blocks
        yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        yuyv = np.zeros((480, 1280), np.uint8)
        yuyv[:, 0::2] = yuv[:, :, 0]; yuyv[:, 1::4] = yuv[:, 0::2, 1]; yuyv[:, 3::4] = yuv[:, 0::2, 2]
        back = cv2.imdecode(np.frombuffer(encode_raw_image(yuyv.tobytes(), 'yuv422_yuy2', 640, 480, 1280),
                                          np.uint8), cv2.IMREAD_COLOR)
        b, g, r = (int(v) for v in back[250, 150]); self.assertTrue(r > 200 and g < 60 and b < 60)
        b, g, r = (int(v) for v in back[250, 450]); self.assertTrue(g > 150 and r < 60 and b < 60)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.assertIsNotNone(encode_raw_image(rgb.tobytes(), 'rgb8', 640, 480, 1920))
        self.assertIsNone(encode_raw_image(b'\x00' * 10, 'unknown_format', 2, 1, 10))


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class BridgeCameraStreamTests(unittest.TestCase):
    def test_frames_only_when_requested(self):
        import cv2
        from ugv_bridge.backends.dryrun_backend import DryRunBackend
        from ugv_nav.edge_client import EdgeClient
        from nav_helpers import KEY, start_bridge
        backend = DryRunBackend()
        img = np.random.default_rng(0).integers(0, 255, (480, 640, 3), dtype=np.uint8)   # worst case for JPEG
        jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
        self.assertGreater(len(jpeg), 64 * 1024)           # bigger than the old frame limit
        server, _ = start_bridge(backend)
        client = EdgeClient('127.0.0.1', server.port, KEY).start()
        try:
            self.assertTrue(wait_for(client.link_ok, timeout=5))
            backend.info = dict(width=640, height=480, k=[473.0, 0, 320, 0, 474.0, 240, 0, 0, 1], d=[0.0] * 5)
            stop = [False]

            import threading
            def feed():   # a camera publishing at ~20 Hz
                while not stop[0]:
                    backend.image = (jpeg, 'jpeg', time.monotonic()); time.sleep(0.05)
            threading.Thread(target=feed, daemon=True).start()
            time.sleep(0.5)
            self.assertEqual(client.image_count, 0)        # nothing sent until asked
            client.request_camera(8)
            self.assertTrue(wait_for(lambda: client.image_count >= 5, timeout=4))
            data = client.latest_image()[0]
            self.assertEqual(data, jpeg)
            self.assertEqual(client.camera_info['k'][0], 473.0)
            self.assertTrue(client.link_ok())               # telemetry still flowing with big frames
            client.request_camera(0)
            time.sleep(0.4); n = client.image_count; time.sleep(0.6)
            self.assertLessEqual(client.image_count - n, 1)
            stop[0] = True
        finally:
            client.close(); server.shutdown()


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class ClosedLoopVisionTests(unittest.TestCase):
    """Real bridge + simulated robot/camera; blocks are invisible to the LiDAR (camera only)."""

    START, GOAL, BLOCK = (1.215, 0.405, 0.0), (2.205, 0.405), (1.70, 0.405)

    def setUp(self):
        from ugv_nav.sim_session import SimulatedUGVSession
        from ugv_nav.vision_sim import SimBlocks, SimCamera, SimDetector
        from ugv_nav.vision import VisionWorker
        self.occ, gt = load_environment('ENV1')
        self.session = SimulatedUGVSession(gt, self.START, 0.08, 0.07)
        self.cam = CameraModel(pitch_rad=math.radians(15), x_offset_m=0.08, height_m=0.145)
        self.blocks = SimBlocks()
        self.session.backend.blocks = self.blocks
        self.session.backend.camera_source = SimCamera(self.session.backend, self.blocks, self.cam)
        self.assertTrue(wait_for(self.session.client.link_ok, timeout=5)); time.sleep(0.2)
        cfg = PlannerConfig(8, Footprint(math.hypot(0.08, 0.07), 0.02, True), True, clearance_weight=4.0)
        self.nav = Navigator(self.session.client, self.occ, cfg, NavigatorConfig(
            controller=ControllerConfig(max_v=0.2, max_w=1.0, min_w=0.2),
            localization=LocalizationConfig(drift_per_m=0.02, max_sigma_m=0.2),
            safety=SafetyConfig(0.08, 0.07), auto_continue_after_replan=True, auto_continue_delay_s=0.3))
        self.nav.set_initial_pose(self.START)
        self.nav.run_in_background()          # as in the console: the control loop keeps the pose fresh
        self.session.client.request_camera(10)
        self.worker = VisionWorker(self.session.client, lambda: self.nav,
                                   SimDetector(self.session.backend, self.blocks, self.cam), self.cam,
                                   ObstacleTracker(TrackerConfig(remove_misses=6))).start()

    def tearDown(self):
        self.worker.stop(); self.nav.shutdown(); self.session.close()

    def test_block_seen_only_by_camera_is_avoided(self):
        self.blocks.add(*self.BLOCK, cls='red_obstacle')
        self.nav.set_goal(self.GOAL)
        straight = list(self.nav.path)
        # The camera sees the block, confirms it and A* replans around it before we drive.
        self.assertTrue(wait_for(lambda: len(self.nav.snapshot()['vision_obstacles']) == 1, timeout=6))
        obs = self.nav.snapshot()['vision_obstacles'][0]
        self.assertLess(math.dist((obs['x'], obs['y']), self.BLOCK), 0.03)   # pin within 3 cm
        self.assertEqual(self.nav.state, PLANNED)
        self.assertNotEqual(self.nav.path, straight)
        self.assertTrue(self.nav.authorize()[0])
        self.assertTrue(wait_for(lambda: self.nav.state == ARRIVED, timeout=60, step=0.1), self.nav.message)
        self.assertFalse(self.session.backend.collided)

    def test_removed_block_cleared_when_spot_in_view_and_route_restored(self):
        block = self.blocks.add(*self.BLOCK, cls='red_obstacle')
        self.nav.set_goal(self.GOAL)
        straight_len = sum(math.dist(a, b) for a, b in zip(self.nav.path, self.nav.path[1:]))
        self.assertTrue(wait_for(lambda: len(self.nav.snapshot()['vision_obstacles']) == 1, timeout=6))
        detour_len = sum(math.dist(a, b) for a, b in zip(self.nav.path, self.nav.path[1:]))
        self.assertGreater(detour_len, straight_len + 0.01)
        self.blocks.remove(block)                        # robot is facing the spot: camera sees it is empty
        self.assertTrue(wait_for(lambda: self.nav.snapshot()['vision_obstacles'] == [], timeout=5))
        self.assertFalse(self.nav.vision_mask.any())
        restored = sum(math.dist(a, b) for a, b in zip(self.nav.path, self.nav.path[1:]))
        self.assertAlmostEqual(restored, straight_len, delta=0.01)   # A* replanned the direct route

    def test_close_block_filling_the_view_is_not_removed(self):
        self.blocks.add(1.45, 0.405, cls='green_obstacle')     # 0.24 m ahead: box cut off at the image edges
        self.blocks.add(1.75, 0.405, cls='red_obstacle')
        self.assertTrue(wait_for(lambda: len(self.nav.snapshot()['vision_obstacles']) >= 1, timeout=6))
        n = len(self.nav.snapshot()['vision_obstacles'])
        self.session.backend.place((1.27, 0.405, 0.0))          # move right up to the green block
        self.nav.set_initial_pose((1.27, 0.405, 0.0))
        time.sleep(3.0)
        self.assertEqual(len(self.nav.snapshot()['vision_obstacles']), n)

    def test_block_out_of_view_is_kept(self):
        block = self.blocks.add(*self.BLOCK, cls='red_obstacle')
        self.assertTrue(wait_for(lambda: len(self.nav.snapshot()['vision_obstacles']) == 1, timeout=6))
        self.session.backend.place((1.215, 0.405, math.pi))   # robot turned away (camera faces the other way)
        self.nav.set_initial_pose((1.215, 0.405, math.pi))
        self.blocks.remove(block)
        time.sleep(2.5)
        self.assertEqual(len(self.nav.snapshot()['vision_obstacles']), 1)   # not seen = not proven gone

    def test_block_appearing_on_route_while_driving_triggers_replan(self):
        self.nav.set_goal(self.GOAL)
        self.assertTrue(self.nav.authorize()[0])
        self.assertTrue(wait_for(lambda: self.session.backend.truth_pose()[0] > self.START[0] + 0.05, timeout=10))
        x = self.session.backend.truth_pose()[0]
        self.blocks.add(x + 0.45, self.BLOCK[1], cls='green_obstacle')   # dropped 45 cm ahead of the robot
        self.assertTrue(wait_for(lambda: 'Camera obstacle' in ' '.join(m for _, m in self.nav.log), timeout=8),
                        self.nav.message)
        self.assertTrue(wait_for(lambda: self.nav.state == ARRIVED, timeout=60, step=0.1), self.nav.message)
        self.assertFalse(self.session.backend.collided)


class TiltFromBlockSizeTests(unittest.TestCase):
    """The known block height tells the camera tilt, whatever tilt is configured."""

    def boxes(self, true_cam, noise=1.0, seed=0, count=60):
        from ugv_nav.vision_sim import _box_for_block
        rng = np.random.default_rng(seed)
        out = []
        while len(out) < count:
            d, yaw = rng.uniform(0.35, 1.4), rng.uniform(-math.pi, math.pi)
            lat = rng.uniform(-0.3, 0.3) * d
            bx, by = d * math.cos(yaw) - lat * math.sin(yaw), d * math.sin(yaw) + lat * math.cos(yaw)
            box = _box_for_block(true_cam, (0.0, 0.0, yaw), bx, by, 0.05, 0.10)
            if box is not None:
                out.append([round(v + rng.uniform(-noise, noise)) for v in box])
        return out

    def test_single_box_gives_the_true_tilt(self):
        from ugv_nav.vision import pitch_from_box
        wrong = CameraModel(pitch_rad=math.radians(3.0), x_offset_m=0.08)       # what the console believes
        for true_deg in (8.0, 15.0, 24.0):
            true = CameraModel(pitch_rad=math.radians(true_deg), x_offset_m=0.08)
            errors = []
            for box in self.boxes(true):
                sample = pitch_from_box(box, wrong)
                if sample is not None:
                    errors.append(abs(math.degrees(sample[0]) - true_deg))
            self.assertGreater(len(errors), 40)
            self.assertLess(float(np.percentile(errors, 95)), 0.6)              # within ~half a degree per box

    def test_cut_off_boxes_give_no_sample(self):
        from ugv_nav.vision import pitch_from_box
        cam = CameraModel(pitch_rad=math.radians(15))
        self.assertIsNone(pitch_from_box((300, 200, 340, 479), cam))            # touches the bottom
        self.assertIsNone(pitch_from_box((0, 200, 30, 300), cam))               # touches the side
        self.assertIsNone(pitch_from_box((300, 0, 340, 100), cam))              # touches the top
        self.assertIsNone(pitch_from_box((300, 200, 340, 300), CameraModel(height_m=0.08)))   # camera below block top

    def test_estimator_is_confident_only_with_enough_agreeing_sightings(self):
        from ugv_nav.vision import TiltEstimator
        est = TiltEstimator(min_samples=12, max_spread_deg=1.2)
        self.assertIsNone(est.estimate())
        for k in range(8):
            est.add(math.radians(15.0 + 0.1 * (k % 3)), 0.8)
        self.assertFalse(est.estimate()['confident'])                           # too few
        for k in range(8):
            est.add(math.radians(15.0 - 0.1 * (k % 3)), 0.9)
        e = est.estimate()
        self.assertTrue(e['confident']); self.assertAlmostEqual(e['deg'], 15.0, delta=0.2)
        est.add(math.radians(15.0), 2.5)                                        # too far: ignored
        self.assertEqual(est.estimate()['n'], 16)
        noisy = TiltEstimator(min_samples=12, max_spread_deg=1.2)
        for k in range(30):
            noisy.add(math.radians(5.0 + 20.0 * ((k * 7) % 10) / 10.0), 0.8)    # sightings that disagree
        self.assertFalse(noisy.estimate()['confident'])


class TrackerImprovementTests(unittest.TestCase):
    def test_close_sightings_outweigh_far_ones(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=3))
        for k in range(8):                                   # far away: position 8 cm off (tilt error)
            tr.update([(1.08, 1.0, 'green_obstacle', 0.9, 1.4)], lambda x, y: True, 0.1 * k)
        far = tr.obstacles()[0]
        self.assertAlmostEqual(far.x, 1.08, delta=0.005)
        for k in range(3):                                   # then three close, accurate sightings
            tr.update([(1.00, 1.0, 'green_obstacle', 0.9, 0.4)], lambda x, y: True, 1.0 + 0.1 * k)
        self.assertEqual(len(tr.obstacles()), 1)
        self.assertAlmostEqual(tr.obstacles()[0].x, 1.00, delta=0.02)

    def test_red_green_flicker_is_one_obstacle_with_the_majority_class(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=3))
        seq = ['red_obstacle', 'red_obstacle', 'green_obstacle', 'red_obstacle', 'red_obstacle', 'green_obstacle',
               'red_obstacle', 'red_obstacle']
        for k, cls in enumerate(seq):
            tr.update([(1.0, 1.0, cls, 0.8, 0.7)], lambda x, y: True, 0.1 * k)
        self.assertEqual([o.cls for o in tr.obstacles()], ['red_obstacle'])       # never a second "green" object

    def test_two_boxes_in_one_frame_stay_two_objects(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=2))
        for t in (0.0, 0.1):
            tr.update([(1.0, 1.0, 'red_obstacle', 0.9, 0.6), (1.06, 1.0, 'green_obstacle', 0.9, 0.6)],
                      lambda x, y: True, t)
        self.assertEqual(sorted(o.cls for o in tr.obstacles()), ['green_obstacle', 'red_obstacle'])

    def test_navigator_is_told_when_an_estimate_really_moved(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=2))
        self.assertFalse(tr.update([(1.0, 1.0, 'red_obstacle', 0.9, 1.2)], lambda x, y: True, 0.0))
        self.assertTrue(tr.update([(1.0, 1.0, 'red_obstacle', 0.9, 1.2)], lambda x, y: True, 0.1))    # confirmed
        self.assertFalse(tr.update([(1.005, 1.0, 'red_obstacle', 0.9, 1.2)], lambda x, y: True, 0.2))  # wobble
        moved = [tr.update([(1.07, 1.0, 'red_obstacle', 0.9, 0.35)], lambda x, y: True, 0.3 + 0.1 * k)
                 for k in range(4)]
        self.assertTrue(any(moved))                                               # > 3 cm shift is reported


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class ShapeFilterTests(unittest.TestCase):
    def frame(self):
        import cv2
        return cv2.imencode('.jpg', np.full((480, 640, 3), 128, np.uint8))[1].tobytes()

    def worker(self, boxes, **options):
        from ugv_nav.vision import VisionWorker
        detector = type('D', (), {'predict': lambda self, f: [
            dict(class_name=c, confidence=0.9, bbox_xyxy=list(b)) for c, b in boxes]})()
        client = type('C', (), {'snapshot': lambda self: dict(telemetry=None)})()
        return VisionWorker(client, lambda: None, detector, CameraModel(pitch_rad=math.radians(15)), **options)

    def test_long_flat_object_is_not_a_block(self):
        barrier = ('green_obstacle', (60, 250, 600, 330))            # e.g. the yellow-green arena bumper
        block = ('green_obstacle', (300, 215, 330, 268))
        shown = self.worker([barrier, block]).process(self.frame())
        self.assertEqual(shown[0]['note'], 'not block-shaped')
        self.assertIsNone(shown[0]['range'])                         # never projected onto the map
        self.assertNotEqual(shown[1]['note'], 'not block-shaped')
        self.assertIsNotNone(shown[1]['range'])
        unfiltered = self.worker([barrier], shape_filter=False).process(self.frame())
        self.assertNotEqual(unfiltered[0]['note'], 'not block-shaped')

    def test_frames_are_handed_to_the_display_with_their_boxes(self):
        w = self.worker([('red_obstacle', (300, 215, 330, 268))])
        self.assertIsNone(w.wait_frame(timeout=0.05))
        w.process(self.frame())
        frame_id, jpeg, width, height, boxes = w.wait_frame(timeout=0.05)
        self.assertEqual((frame_id, width, height, len(boxes)), (1, 640, 480, 1))
        self.assertEqual(jpeg[:2], b'\xff\xd8')                      # the UGV's own JPEG, not re-encoded
        self.assertIsNone(w.wait_frame(after_id=1, timeout=0.05))    # nothing newer yet


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class NotABlockTests(ShapeFilterTests):
    """A door seen as a 'red block', a red circle, a wrong-coloured box: shown, never put on the map."""

    def real_block(self, cam, x=0.8, y=0.05):
        from ugv_nav.vision_sim import _box_for_block
        return _box_for_block(cam, (0.0, 0.0, 0.0), x, y, 0.05, 0.10)

    def test_door_is_too_tall_for_a_block(self):
        cam = CameraModel(pitch_rad=math.radians(15))
        block = self.real_block(cam)
        door_in_view = (250, 20, 330, 300)                           # tall red thing standing on the floor
        door_cut_off = (200, 0, 420, 330)                            # runs out of the top of the picture
        poster = (300, 40, 340, 110)                                 # red thing on the wall, above the floor line
        shown = self.worker([('red_obstacle', door_in_view), ('red_obstacle', door_cut_off),
                             ('red_obstacle', poster), ('red_obstacle', block)]).process(self.frame())
        self.assertEqual([b['note'] for b in shown[:2]], ['too tall for a block'] * 2)
        self.assertIn(shown[2]['note'], ('above the floor line', 'too tall for a block'))
        self.assertTrue(all(b['range'] is None for b in shown[:3]))  # never projected onto the map
        self.assertIsNotNone(shown[3]['range'])                      # the real block is still used
        self.assertAlmostEqual(shown[3]['range'], 0.75, delta=0.08)       # measured from the camera, not the robot centre

    def test_real_blocks_pass_at_every_distance_and_with_a_slightly_wrong_tilt(self):
        from ugv_nav.vision import block_size_fault
        true = CameraModel(pitch_rad=math.radians(15))
        for assumed_deg in (13, 15, 17):                             # tilt not perfectly known
            cam = CameraModel(pitch_rad=math.radians(assumed_deg))
            for x in (0.3, 0.5, 0.8, 1.1, 1.4):
                for y in (-0.2, 0.0, 0.2):
                    box = self.real_block(true, x, y)
                    if box is not None:
                        self.assertEqual(block_size_fault(box, cam), '', (assumed_deg, x, y, box))

    def test_round_or_square_thing_is_not_a_block(self):
        cam = CameraModel(pitch_rad=math.radians(15))
        b = self.real_block(cam)
        h = b[3] - b[1]
        circle = (300, b[3] - h, 300 + h, b[3])                      # as tall as a block but as wide as tall
        shown = self.worker([('red_obstacle', circle), ('red_obstacle', b)]).process(self.frame())
        self.assertEqual(shown[0]['note'], 'not block-shaped')
        self.assertEqual(shown[1]['note'] in ('', 'no map pose'), True, shown[1]['note'])

    def test_box_without_its_colour_is_ignored(self):
        import cv2
        cam = CameraModel(pitch_rad=math.radians(15))
        green, brown = self.real_block(cam, 0.8, -0.15), self.real_block(cam, 0.8, 0.15)
        img = np.full((480, 640, 3), 128, np.uint8)
        cv2.rectangle(img, (int(green[0]), int(green[1])), (int(green[2]), int(green[3])), (60, 160, 60), -1)
        cv2.rectangle(img, (int(brown[0]), int(brown[1])), (int(brown[2]), int(brown[3])), (40, 70, 110), -1)   # wood
        jpeg = cv2.imencode('.jpg', img)[1].tobytes()
        shown = self.worker([('green_obstacle', green), ('red_obstacle', brown)], colour_check=True).process(jpeg)
        self.assertNotEqual(shown[0]['note'], 'colour does not match')
        self.assertEqual(shown[1]['note'], 'colour does not match')
        off = self.worker([('red_obstacle', brown)], colour_check=False).process(jpeg)
        self.assertNotEqual(off[0]['note'], 'colour does not match')


class OneBlockOnePinTests(unittest.TestCase):
    def test_far_scattered_sightings_of_one_block_stay_one_pin(self):
        tr = ObstacleTracker()
        rng = np.random.default_rng(3)
        for k in range(40):                                          # 1.3 m away: +-9 cm scatter along the view
            x = 1.0 + float(rng.uniform(-0.09, 0.09))
            tr.update([(x, 1.0, 'green_obstacle', 0.8, 1.3)], lambda x, y: True, 0.1 * k)
        self.assertEqual(len(tr.obstacles()), 1)
        self.assertAlmostEqual(tr.obstacles()[0].x, 1.0, delta=0.05)

    def test_two_pins_of_the_same_colour_close_together_are_joined(self):
        tr = ObstacleTracker(TrackerConfig(confirm_hits=2))
        for t in (0.0, 0.1):                                         # seen from far: 16 cm apart -> two tracks
            tr.update([(1.00, 1.0, 'green_obstacle', 0.9, 0.5)], lambda x, y: True, t)
        tr.tracks[99] = type(next(iter(tr.tracks.values())))(99, 'green_obstacle', 1.16, 1.0, 0.9, hits=2,
                                                             confirmed=True, weight=1.0, reported=(1.16, 1.0))
        self.assertEqual(len(tr.obstacles()), 2)
        self.assertTrue(tr.update([(1.00, 1.0, 'green_obstacle', 0.9, 0.5)], lambda x, y: True, 0.2))
        self.assertEqual(len(tr.obstacles()), 1)                     # one block, one pin
        # ... but a red and a green block next to each other stay two objects
        tr.update([(1.17, 1.0, 'red_obstacle', 0.9, 0.5)], lambda x, y: True, 0.3)
        tr.update([(1.17, 1.0, 'red_obstacle', 0.9, 0.5)], lambda x, y: True, 0.4)
        self.assertEqual(sorted(o.cls for o in tr.obstacles()), ['green_obstacle', 'red_obstacle'])

    def test_low_confidence_boxes_never_become_pins(self):
        tr = ObstacleTracker()
        for k in range(20):
            tr.update([(1.0, 1.0, 'red_obstacle', 0.55, 0.6)], lambda x, y: True, 0.1 * k)
        self.assertEqual(tr.obstacles(), [])                         # default minimum confidence is 60 %


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class AutoTiltClosedLoopTests(unittest.TestCase):
    """Console believes 5 deg tilt, the (simulated) camera really has 15 deg; boxes jitter by 1 px."""

    START, BLOCK = (1.215, 0.405, 0.0), (1.60, 0.47)

    def run_worker(self, auto_tilt):
        from ugv_nav.sim_session import SimulatedUGVSession
        from ugv_nav.vision_sim import SimBlocks, SimCamera, SimDetector
        from ugv_nav.vision import VisionWorker
        occ, gt = load_environment('ENV1')
        session = SimulatedUGVSession(gt, self.START, 0.08, 0.07)
        true_cam = CameraModel(pitch_rad=math.radians(15), x_offset_m=0.08)
        believed = CameraModel(pitch_rad=math.radians(5), x_offset_m=0.08)
        blocks = SimBlocks()
        session.backend.blocks = blocks
        session.backend.camera_source = SimCamera(session.backend, blocks, true_cam)
        self.assertTrue(wait_for(session.client.link_ok, timeout=5)); time.sleep(0.2)
        cfg = PlannerConfig(8, Footprint(math.hypot(0.08, 0.07), 0.02, True), True, clearance_weight=4.0)
        nav = Navigator(session.client, occ, cfg, NavigatorConfig(
            localization=LocalizationConfig(drift_per_m=0.02, max_sigma_m=0.2), safety=SafetyConfig(0.08, 0.07)))
        nav.set_initial_pose(self.START)
        nav.run_in_background()
        session.client.request_camera(10)
        changes = []
        worker = VisionWorker(session.client, lambda: nav, SimDetector(session.backend, blocks, true_cam, noise_px=1.0),
                              believed, ObstacleTracker(), auto_tilt=auto_tilt,
                              on_camera_change=lambda cam, est: changes.append((cam, est))).start()
        try:
            blocks.add(*self.BLOCK, cls='green_obstacle')
            self.assertTrue(wait_for(lambda: len(nav.snapshot()['vision_obstacles']) == 1, timeout=8))
            self.assertTrue(wait_for(lambda: (worker.snapshot()['tilt'] or {}).get('confident'), timeout=8))
            time.sleep(1.5)
            pin = nav.snapshot()['vision_obstacles'][0]
            return math.dist((pin['x'], pin['y']), self.BLOCK), worker.snapshot(), changes
        finally:
            worker.stop(); nav.shutdown(); session.close()

    def test_wrong_tilt_is_found_and_corrected(self):
        error_off, snap_off, changes_off = self.run_worker(auto_tilt=False)
        self.assertGreater(error_off, 0.10)                              # 10 deg tilt error = pin far off
        self.assertAlmostEqual(snap_off['tilt']['deg'], 15.0, delta=0.8)   # ... but the estimate already knows
        self.assertAlmostEqual(math.degrees(snap_off['camera'].pitch_rad), 5.0, delta=0.01)   # not applied
        self.assertEqual(changes_off, [])
        error_on, snap_on, changes_on = self.run_worker(auto_tilt=True)
        self.assertAlmostEqual(math.degrees(snap_on['camera'].pitch_rad), 15.0, delta=0.8)
        self.assertLess(error_on, 0.04)                                  # pin within 4 cm without manual calibration
        self.assertGreaterEqual(len(changes_on), 1)                      # console was told (to save the new tilt)


if __name__ == '__main__':
    unittest.main()
