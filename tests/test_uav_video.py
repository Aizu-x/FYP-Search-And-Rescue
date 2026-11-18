"""The drone's camera path: the simulated camera (drawn room, H.264 without an encoder) and the worker process
that receives the video, decodes it, sends preview pictures and writes footage and photos (uav/video.py).

Everything runs on 127.0.0.1 with the simulated drone. The real Tello's video (960 x 720 from a hardware encoder,
over Wi-Fi) has not been through this code yet.
"""
import os
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from uav.h264 import IpcmEncoder
from uav.sim import FakeTello, Scene
from uav.video import VideoError, VideoFeed

from nav_helpers import wait_for


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def green_centre(image):
    """Where the green blocks are in a picture: (mean column, mean row)."""
    b, g, r = (image[:, :, k].astype(int) for k in range(3))
    rows, cols = np.nonzero((g > 110) & (b < 100) & (r < 100))
    return float(cols.mean()), float(rows.mean()), len(cols)


class SimulatedCameraTests(unittest.TestCase):
    def test_a_normal_h264_decoder_reads_it(self):
        encoder = IpcmEncoder(160, 96)
        pictures, data = [], b''
        for k in range(4):
            image = np.zeros((96, 160, 3), np.uint8)
            image[:] = (40 + 50 * k, 120, 200 - 40 * k)
            cv2.rectangle(image, (20 + 10 * k, 20), (60 + 10 * k, 60), (250, 250, 250), -1)
            pictures.append(image)
            data += encoder.encode(image)
        with tempfile.TemporaryDirectory(prefix='uav h264 test ') as folder:
            path = Path(folder) / 'camera.h264'
            path.write_bytes(data)
            capture = cv2.VideoCapture(str(path))
            decoded = []
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                decoded.append(frame)
            capture.release()
        self.assertEqual(len(decoded), 4)
        for want, got in zip(pictures, decoded):
            self.assertEqual(got.shape, want.shape)
            self.assertLess(float(np.abs(got.astype(int) - want.astype(int)).mean()), 4.0)

    def test_picture_size_must_fit_whole_macroblocks(self):
        with self.assertRaises(ValueError):
            IpcmEncoder(650, 480)
        with self.assertRaises(ValueError):
            IpcmEncoder(160, 96).encode(np.zeros((90, 160, 3), np.uint8))

    def test_the_view_follows_the_drone(self):
        scene = Scene()
        start = scene.render(0.0, 0.0, 1.0, 0.0, 320, 240)
        self.assertEqual(start.shape, (240, 320, 3))
        x0, y0, n0 = green_centre(start)
        self.assertGreater(n0, 30)                                              # the green blocks are in view
        turned_right = scene.render(0.0, 0.0, 1.0, -0.25, 320, 240)             # clockwise seen from above
        self.assertLess(green_centre(turned_right)[0], x0 - 20)                 # ... so the scene moves to the left
        turned_left = scene.render(0.0, 0.0, 1.0, 0.25, 320, 240)
        self.assertGreater(green_centre(turned_left)[0], x0 + 20)
        lower = scene.render(0.0, 0.0, 0.6, 0.0, 320, 240)
        self.assertLess(green_centre(lower)[1], y0 - 10)                        # from lower down the floor is higher in the picture
        further = scene.render(-0.8, 0.0, 1.0, 0.0, 320, 240, ('SIMULATED DRONE',))
        self.assertLess(green_centre(further)[2], 0.8 * n0)                     # flying backwards: the blocks get smaller
        on_the_ground = scene.render(0.0, 0.0, 0.0, 0.0, 320, 240)
        self.assertFalse(np.array_equal(on_the_ground, start))


class WorkerCase(unittest.TestCase):
    """The real worker process, fed by the simulated drone's video (320 x 240 to keep the tests quick)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='uav video test ')
        self.folder = Path(self.tmp.name) / 'footage'
        self.sim = FakeTello(video=True, video_port=0, state_port=0, size=(320, 240), takeoff_s=0.5).start()
        self.feed = VideoFeed('127.0.0.1', 0, '127.0.0.1', command_port=self.sim.cmd_port).start()
        self.sim.video_port = self.feed.port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(3.0)

    def tearDown(self):
        self.feed.stop()
        self.sim.stop()
        self.sock.close()
        self.tmp.cleanup()

    def say(self, text):
        self.sock.sendto(text.encode('ascii'), ('127.0.0.1', self.sim.cmd_port))
        return self.sock.recvfrom(200)[0].decode('ascii')

    def stream(self):
        self.assertEqual(self.say('command'), 'ok')
        self.assertEqual(self.say('streamon'), 'ok')
        first = self.feed.wait_frame(None, 15.0)
        self.assertIsNotNone(first, 'no picture arrived')
        return first

    def event(self, kind, timeout=10.0):
        found = []

        def look():
            for event in self.feed.take_events():
                if event[0] == kind:
                    found.append(event)
            return bool(found)
        self.assertTrue(wait_for(look, timeout), 'no "%s" from the worker' % kind)
        return found[0]


class WorkerTests(WorkerCase):
    def test_pictures_arrive_for_the_preview(self):
        self.assertGreater(self.feed.port, 0)
        self.assertIsNone(self.feed.wait_frame(None, 0.3))                      # no video yet
        frame_id, jpeg, width, height = self.stream()
        image = decode(jpeg)
        self.assertEqual((image.shape, width, height), ((240, 320, 3), 320, 240))
        self.assertGreater(green_centre(image)[2] + int(image.std()), 10)       # a real picture, not one flat colour
        newer = self.feed.wait_frame(frame_id, 3.0)
        self.assertGreater(newer[0], frame_id)
        time.sleep(2.2)
        status = self.feed.status()
        self.assertGreater(status['fps'], 6.0)
        self.assertEqual((status['width'], status['height'], status['rejected'], status['dead']), (320, 240, 0, False))
        self.assertLess(status['frame_age'], 1.0)
        self.assertIsNone(status['recording'])

    def test_the_picture_comes_back_after_the_video_stopped(self):
        last = self.stream()[0]
        self.assertEqual(self.say('streamoff'), 'ok')
        time.sleep(4.0)                                                         # longer than the decoder waits
        last = self.feed.wait_frame(None, 0)[0]
        self.assertIsNone(self.feed.wait_frame(last, 1.0))
        self.assertEqual(self.say('streamon'), 'ok')
        again = self.feed.wait_frame(last, 15.0)
        self.assertIsNotNone(again, 'no picture after the video was switched on again')
        self.assertGreater(again[0], last)

    def test_footage_is_as_long_as_the_recording(self):
        self.stream()
        self.feed.record_start(self.folder, 'test run')
        self.assertEqual(self.event('record_started')[1], str(self.folder))
        time.sleep(3.0)
        self.assertGreater(self.feed.status()['recording']['frames'], 30)
        self.assertEqual(len(list(self.folder.glob('*_recording.*'))), 1)        # the unfinished file is marked
        self.feed.record_stop('test finished')
        _, recording, fault = self.event('recorded')
        self.assertEqual(fault, '')
        self.assertRegex(recording.name, r'^flight_\d{4}-\d\d-\d\d_\d\d-\d\d-\d\d_test_run\.(mp4|avi)$')
        self.assertAlmostEqual(recording.seconds, 3.0, delta=0.7)
        self.assertEqual(recording.reason, 'test finished')
        self.assertEqual([p.name for p in self.folder.iterdir()], [recording.name])
        capture = cv2.VideoCapture(str(recording.path))
        frames, first = 0, None
        while True:
            ok, image = capture.read()
            if not ok:
                break
            frames, first = frames + 1, image if first is None else first
        fps = capture.get(cv2.CAP_PROP_FPS)
        capture.release()
        self.assertEqual(first.shape, (240, 320, 3))                            # full size, nothing drawn over it
        self.assertAlmostEqual(fps, 30.0, delta=0.5)
        self.assertAlmostEqual(frames / fps, recording.seconds, delta=0.2)
        self.assertGreater(int(first.std()), 5)

    def test_photo(self):
        self.feed.photo(self.folder)
        self.assertEqual(self.event('photo')[1], None)                          # no picture yet: no photo
        self.stream()
        self.feed.photo(self.folder)
        _, path, why = self.event('photo')
        self.assertEqual(why, '')
        self.assertRegex(Path(path).name, r'^photo_\d{4}-\d\d-\d\d_\d\d-\d\d-\d\d(_\d+)?\.jpg$')
        self.assertEqual(cv2.imread(path).shape, (240, 320, 3))

    def test_nothing_is_saved_when_no_picture_arrived(self):
        self.feed.record_start(self.folder)                                     # the video was never switched on
        self.event('record_started')
        time.sleep(0.8)
        self.feed.record_stop('stopped')
        _, recording, fault = self.event('recorded')
        self.assertIsNone(recording)
        self.assertEqual(list(self.folder.iterdir()), [])

    def test_a_folder_that_cannot_be_used_is_refused(self):
        blocker = Path(self.tmp.name) / 'a file'
        blocker.write_text('x')
        self.feed.record_start(blocker)
        self.assertIn('cannot be used', self.event('record_failed')[1])
        self.feed.photo(blocker)
        self.event('photo')                                                     # (answered, nothing crashed)
        self.assertFalse(self.feed.dead)

    def test_video_from_another_address_is_dropped(self):
        other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(other.close)
        try:
            other.bind(('127.0.0.2', 0))                                        # another address of this computer
        except OSError:
            self.skipTest('this system has no second loopback address')
        data = IpcmEncoder(320, 240).encode(np.full((240, 320, 3), 200, np.uint8))
        for _ in range(3):
            for start in range(0, len(data), 1460):
                other.sendto(data[start:start + 1460], ('127.0.0.1', self.feed.port))
        self.assertTrue(wait_for(lambda: self.feed.status().get('rejected', 0) > 100, 4))
        self.assertEqual(self.feed.status()['packets'], 0)
        self.assertIsNone(self.feed.wait_frame(None, 1.0))                      # no picture was made from it

    def test_footage_is_saved_when_the_console_goes_away(self):
        self.stream()
        self.feed.record_start(self.folder, 'crash')
        self.event('record_started')
        time.sleep(2.0)
        self.feed._proc.stdin.close()                                           # the console program is gone
        self.assertTrue(wait_for(lambda: self.feed._proc.poll() is not None, 10))
        files = [p.name for p in self.folder.iterdir()]
        self.assertEqual(len(files), 1, files)
        self.assertNotIn('_recording', files[0])
        capture = cv2.VideoCapture(str(self.folder / files[0]))
        self.assertGreater(capture.get(cv2.CAP_PROP_FRAME_COUNT), 30)
        capture.release()

    def test_the_worker_lands_the_drone_when_the_console_dies_in_flight(self):
        sim = self.sim
        self.stream()
        self.assertEqual(self.say('takeoff'), 'ok')
        self.feed.set_airborne(True)                                            # what the console tells the worker
        for _ in range(6):                                                      # flying forward
            self.sock.sendto(b'rc 0 40 0 0', ('127.0.0.1', sim.cmd_port))
            time.sleep(0.1)
        self.assertEqual((sim.phase, sim.rc), ('air', (0, 40, 0, 0)))
        self.feed._proc.stdin.close()                                           # the console program is gone
        self.assertTrue(wait_for(lambda: sim.phase in ('landing', 'ground'), 5), sim.phase)
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 6))
        self.assertIn('land', list(sim.commands))
        self.assertFalse(sim.landed_by_itself)                                  # told to, not the 15 s safety landing

    def test_on_the_ground_the_worker_sends_the_drone_nothing(self):
        self.stream()
        self.feed.set_airborne(True)
        self.feed.set_airborne(False)                                           # a flight that has ended
        time.sleep(0.3)
        self.feed._proc.stdin.close()
        self.assertTrue(wait_for(lambda: self.feed._proc.poll() is not None, 10))
        self.assertNotIn('land', list(self.sim.commands))

    def test_a_worker_that_dies_is_reported(self):
        self.stream()
        pid = self.feed._proc.pid
        if os.name == 'nt':                                                     # (the launcher and the real interpreter)
            subprocess.run(['taskkill', '/T', '/F', '/PID', str(pid)], capture_output=True, timeout=10)
        else:
            self.feed._proc.kill()
        self.assertTrue(wait_for(lambda: self.feed.dead, 6))
        self.event('gone', 3)
        t0 = time.monotonic()
        self.feed.wait_frame(10 ** 9, 5.0)
        self.assertLess(time.monotonic() - t0, 1.0)                             # nobody waits for a dead worker
        self.assertTrue(self.feed.status()['dead'])
        self.feed.record_start(self.folder)                                     # ... and nothing raises
        self.feed.photo(self.folder)


class StartTests(unittest.TestCase):
    def test_port_in_use_is_explained(self):
        holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(holder.close)
        holder.bind(('127.0.0.1', 0))
        feed = VideoFeed('127.0.0.1', holder.getsockname()[1], '127.0.0.1')
        with self.assertRaises(VideoError) as caught:
            feed.start()
        self.assertIn('used by another program', str(caught.exception))
        self.assertTrue(wait_for(lambda: feed._proc.poll() is not None, 6))     # no worker left behind


if __name__ == '__main__':
    unittest.main()
