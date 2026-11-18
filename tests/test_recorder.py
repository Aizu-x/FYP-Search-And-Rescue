"""Mission video recorder: the video file (deterministic, driven by explicit times) and the worker process
around it (real process, real time). No robot needed."""
import importlib.util
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

HAVE_CV2 = importlib.util.find_spec('cv2') is not None

if HAVE_CV2:
    import cv2
    from ugv_nav import recorder as R

T0 = 1_000_000.0                       # an arbitrary start time for the deterministic tests


def jpeg(grey=200, size=(640, 480)):
    ok, buf = cv2.imencode('.jpg', np.full((size[1], size[0], 3), grey, np.uint8))
    return buf.tobytes()


def video_info(path):
    cap = cv2.VideoCapture(str(path))
    info = dict(frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), fps=cap.get(cv2.CAP_PROP_FPS))
    ok, first = cap.read()
    info['first'] = first if ok else None
    cap.release()
    return info


class TempFolder(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv recorder test ')
        self.dir = Path(self.tmp.name)
        self.jpg = jpeg()

    def tearDown(self):
        self.tmp.cleanup()


# ================================================================================================ the video file
@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class VideoCoreTests(TempFolder):
    def core(self, **options):
        return R.VideoCore(self.dir, label='ENV3', **options)

    def run_core(self, core, seconds, rate=8.0, boxes=(), info=None, skip=0.0):
        """The console's poller: a message every 0.1 s, carrying a picture only when the camera made a new one."""
        last = None
        for k in range(int(round(seconds * 10))):
            t = T0 + k * 0.1
            if (t - T0) < skip:
                core.feed(t, None, info)                           # (the camera has not delivered anything yet)
                continue
            camera_id = int((t - T0 - skip) * rate) + 1
            core.feed(t, (camera_id, self.jpg, list(boxes)) if camera_id != last else None, info)
            last = camera_id
        return core

    # ------------------------------------------------------------------ the file
    def test_video_is_as_long_as_the_mission_whatever_the_camera_rate(self):
        for rate in (8.0, 3.0, 25.0):                          # normal, slow (pictures repeat), fast (extra ones skipped)
            core = self.run_core(self.core(), 2.0, rate)
            result, fault = core.finish(T0 + 2.0, 'test')
            self.assertEqual(fault, '')
            self.assertAlmostEqual(result.seconds, 2.0, delta=0.11)
            self.assertAlmostEqual(result.frames, 20, delta=1)
            info = video_info(result.path)
            self.assertEqual((info['width'], info['height']), (640, 480))
            self.assertAlmostEqual(info['frames'], result.frames, delta=1)
            self.assertAlmostEqual(info['fps'], 10.0, delta=0.5)
            self.assertEqual((result.codec, result.path.suffix), ('mp4v', '.mp4'))
            self.assertGreater(result.size_bytes, 1000)
            for old in list(self.dir.iterdir()):
                old.unlink()

    def test_file_name_and_no_unfinished_file_is_left_behind(self):
        core = self.run_core(self.core(stamp='2026-11-17_14-03-22'), 1.0)
        result, _ = core.finish(T0 + 1.0, 'test')
        self.assertEqual(result.name, 'mission_2026-11-17_14-03-22_ENV3.mp4')
        self.assertEqual([p.name for p in self.dir.iterdir()], [result.name])      # no "_recording" file, no probe file
        self.assertEqual(result.reason, 'test')

    def test_the_tail_after_the_mission_ends_is_in_the_video(self):
        core = self.run_core(self.core(), 1.0)
        result, _ = core.finish(T0 + 1.8, 'mission complete')                      # the last picture is held for 0.8 s
        self.assertAlmostEqual(result.seconds, 1.8, delta=0.11)

    def test_the_video_starts_with_the_first_camera_picture(self):
        core = self.run_core(self.core(), 2.2, skip=1.2)       # the camera needs 1.2 s before its first picture
        result, _ = core.finish(T0 + 2.2, 'test')
        self.assertAlmostEqual(result.seconds, 1.0, delta=0.11)                    # the empty start is not in the video

    def test_no_camera_picture_means_no_file(self):
        core = self.core()
        for k in range(10):
            core.feed(T0 + k * 0.1, None, dict(phase='x', elapsed=0.0, found=0, secured=0))
        self.assertEqual(core.finish(T0 + 1.0, 'nothing came'), (None, ''))
        self.assertEqual(list(self.dir.iterdir()), [])

    def test_a_picture_that_cannot_be_decoded_is_ignored(self):
        core = self.core()
        core.feed(T0, (1, b'this is not a jpeg', []), None)
        core.feed(T0 + 0.1, (2, self.jpg, []), None)
        for k in range(2, 12):
            core.feed(T0 + k * 0.1, None, None)
        result, _ = core.finish(T0 + 1.2, 'test')
        self.assertAlmostEqual(result.seconds, 1.1, delta=0.11)

    def test_mp4_missing_falls_back_to_avi_and_no_codec_at_all_is_an_error(self):
        with mock.patch.object(R, 'CODECS', (('mp4', 'ZZZZ'), ('avi', 'MJPG'))):
            core = self.run_core(self.core(), 0.8)
            result, fault = core.finish(T0 + 0.8, 'test')
        self.assertEqual((result.codec, result.path.suffix, fault), ('MJPG', '.avi', ''))
        self.assertGreaterEqual(video_info(result.path)['frames'], 5)
        with mock.patch.object(R, 'CODECS', (('mp4', 'ZZZZ'),)):
            with self.assertRaises(R.RecorderError):
                self.core().feed(T0, (1, self.jpg, []), None)

    # ------------------------------------------------------------------ safeguards
    def test_the_disk_filling_up_ends_the_video_and_keeps_what_was_filmed(self):
        small = SimpleNamespace(total=1, used=1, free=200 * 1024 ** 2)
        core = self.core()
        with mock.patch.object(R.shutil, 'disk_usage', return_value=small), mock.patch.object(R, 'DISK_CHECK_FRAMES', 5):
            ended = False
            for k in range(30):
                ended = core.feed(T0 + k * 0.1, (k + 1, self.jpg, []), None)
                if ended:
                    break
        self.assertTrue(ended)
        result, fault = core.finish(T0 + 3.0, 'test')
        self.assertEqual(result.reason, 'disk almost full')
        self.assertIn('almost full', fault)
        self.assertGreaterEqual(video_info(result.path)['frames'], 3)

    def test_a_video_has_a_maximum_length(self):
        core = self.core()
        with mock.patch.object(R, 'MAX_SECONDS', 1.0), mock.patch.object(R, 'DISK_CHECK_FRAMES', 5):
            ended = any(core.feed(T0 + k * 0.1, (k + 1, self.jpg, []), None) for k in range(40))
        self.assertTrue(ended)
        result, fault = core.finish(T0 + 4.0, 'test')
        self.assertEqual(result.reason, 'maximum length reached')
        self.assertLessEqual(result.seconds, 1.6)

    def test_a_jump_of_the_clock_does_not_write_minutes_of_the_same_picture(self):
        core = self.core()
        core.feed(T0, (1, self.jpg, []), None)
        core.feed(T0 + 5000.0, (2, self.jpg, []), None)                            # the clock jumped
        result, _ = core.finish(T0 + 5001.0, 'test')
        self.assertLessEqual(result.seconds, R.MAX_GAP_S + 2)

    # ------------------------------------------------------------------ the picture
    def picture(self, **options):
        return R.VideoCore(self.dir, **options), np.full((480, 640, 3), 200, np.uint8)

    def test_overlay_text_strips_events_and_the_clock(self):
        core, image = self.picture()
        info = dict(phase='Approaching survivor S1', elapsed=95.0, found=1, secured=0,
                    event=(92.0, 'SURVIVOR S1 found at (1.52, 1.02) m.'))
        out = core._compose(image, [], 0.0, info, T0)
        self.assertLess(out[:20].mean(), 100)                                       # darkened strips carry the text
        self.assertLess(out[-20:].mean(), 100)
        self.assertEqual(int(out[200:280].mean()), 200)                             # the middle of the picture is untouched
        self.assertTrue((out[:24] != image[:24]).any() and (out[-24:] != image[-24:]).any())
        later = core._compose(image, [], 0.0, dict(info, elapsed=105.0), T0)
        self.assertTrue((out[-24:, :320] != later[-24:, :320]).any())               # the event text is gone after 6 s
        self.assertLess(int(later[-22:-4, :320].max()), 110)                          # only the dark strip is left: no text
        stale = core._compose(image, [], 4.0, info, T0)
        self.assertTrue((stale[-24:, :320] != out[-24:, :320]).any())               # "camera frame is 4 s old" replaces it
        b, g, r = stale[-24:, :320].reshape(-1, 3).astype(int).T
        self.assertTrue(((r - b) > 100).any())                                      # ... in orange
        next_second = core._compose(image, [], 0.0, info, T0 + 1.0)
        self.assertTrue((next_second[-24:, 400:] != out[-24:, 400:]).any())         # the wall-clock text moves on

    def test_long_texts_never_run_into_the_clock(self):
        core, image = self.picture()
        info = dict(phase='Approaching survivor S1', elapsed=5.0, found=1, secured=1,
                    event=(4.0, 'Survivor S1 SECURED at (1.60, 0.48) after standing 3 s in front of it.'))
        out = core._compose(image, [], 0.0, info, T0)
        stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(T0))
        stamp_start = 640 - cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] - 8
        gap = out[-22:-4, stamp_start - 14:stamp_start - 2].reshape(-1, 3)          # the event text is cut 16 px before the clock
        self.assertLess(int(gap.max()), 110)
        self.assertGreater(int(out[-22:-4, 8:stamp_start - 16].max()), 200)         # ... while the event text itself is there
        tiny = np.full((120, 160, 3), 200, np.uint8)                                # a very small picture: still no error
        self.assertEqual(core._compose(tiny, [], 0.0, info, T0).shape, tiny.shape)

    def test_non_ascii_mission_text_is_safe(self):
        core, image = self.picture()
        info = dict(phase='Näher ran · Überleben', elapsed=1.0, found=0, secured=0)
        self.assertEqual(core._compose(image, [], 0.0, info, T0).shape, image.shape)

    def test_boxes_are_drawn_in_the_live_view_colours(self):
        boxes = [dict(box=[100, 150, 180, 300], cls='green_obstacle', conf=0.95, range=0.71, used=True, note=''),
                 dict(box=[400, 150, 480, 300], cls='red_obstacle', conf=0.9, range=0.9, used=True, note=''),
                 dict(box=[250, 150, 300, 300], cls='red_obstacle', conf=0.7, range=None, used=False, note='too tall for a block'),
                 dict(cls='broken entry')]
        core, image = self.picture()
        out = core._compose(image, boxes, 0.0, {}, T0)
        self.assertEqual(tuple(out[225, 100]), R.GREEN)                             # left edge of the green box
        self.assertEqual(tuple(out[225, 400]), R.RED)
        grey = [int(v) for v in out[225, 250]]                                      # not used: a thin grey line (anti-aliased)
        self.assertTrue(grey[0] == grey[1] == grey[2] and 165 <= grey[0] <= 185, grey)
        self.assertEqual(tuple(out[225, 140]), (200, 200, 200))                     # inside a box the picture stays visible

    def test_plain_footage_without_overlay(self):
        core = self.run_core(self.core(overlay=False), 0.8)
        result, _ = core.finish(T0 + 0.8, 'test')
        first = video_info(result.path)['first']
        self.assertAlmostEqual(float(first[:20].mean()), 200.0, delta=6)           # no darkened strip: just the camera picture

    def test_folder_checks(self):
        nested = R.check_folder(self.dir / 'a' / 'b')
        self.assertTrue(nested.is_dir())
        self.assertEqual(list(nested.iterdir()), [])                                # the write probe is removed again
        blocker = self.dir / 'file.txt'
        blocker.write_text('x')
        with self.assertRaises(R.RecorderError):
            R.check_folder(blocker)                                                 # a file is not a folder
        self.assertEqual(R.default_folder(), Path.home() / 'Videos' / 'UGV missions')


# ================================================================================================ the worker process
@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class WorkerProcessTests(TempFolder):
    """A real worker process and real time."""

    def camera(self, rate_hz=8.0, boxes=()):
        """A camera that delivers a NEW frame (new id) rate_hz times a second."""
        t0 = time.monotonic()
        return lambda: (int((time.monotonic() - t0) * rate_hz) + 1, self.jpg, 640, 480,
                        [dict(b, robot=(0.5, 0.1), map=np.float64(1.0)) for b in boxes])

    def record(self, source, seconds, tail=0.0, **options):
        rec = R.MissionRecorder(self.dir, source, label='ENV3', **options).start()
        began = time.monotonic()
        time.sleep(seconds)
        rec.stop('test', tail_s=tail, wait=True)
        return rec, time.monotonic() - began

    def test_video_is_as_long_as_the_mission_with_a_real_worker(self):
        for rate in (8.0, 3.0):
            rec, wall = self.record(self.camera(rate, boxes=[dict(box=[100, 150, 180, 300], cls='green_obstacle', conf=0.9,
                                                                  range=0.5, used=True, note='')]), 3.0)
            result, fault = rec.take_result()
            self.assertEqual(fault, '')
            self.assertAlmostEqual(result.seconds, wall, delta=0.5)
            info = video_info(result.path)
            self.assertEqual((info['width'], info['height']), (640, 480))
            self.assertAlmostEqual(info['frames'], result.frames, delta=1)
            self.assertEqual(rec.take_result(), (None, ''))                        # handed out once
            self.assertEqual([p.name for p in self.dir.iterdir()], [result.name])
            self.assertRegex(result.name, r'^mission_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_ENV3\.mp4$')
            for old in list(self.dir.iterdir()):
                old.unlink()

    def test_progress_is_reported_while_recording_and_the_tail_is_kept(self):
        rec = R.MissionRecorder(self.dir, self.camera()).start()
        self.assertTrue(rec.status()['waiting'])
        deadline = time.monotonic() + 20
        while rec.status()['seconds'] < 1.0 and time.monotonic() < deadline:
            time.sleep(0.2)
        status = rec.status()
        self.assertGreaterEqual(status['frames'], 10)
        self.assertFalse(status['waiting'] or status['done'])
        began = time.monotonic()
        rec.stop('mission complete', tail_s=0.8, wait=True)
        self.assertGreater(time.monotonic() - began, 0.7)                          # the tail was really recorded
        result, _ = rec.take_result()
        self.assertEqual(result.reason, 'mission complete')

    def test_no_camera_pictures_means_no_file(self):
        rec = R.MissionRecorder(self.dir, lambda: None).start()
        time.sleep(1.0)
        self.assertTrue(rec.status()['waiting'])
        rec.stop('nothing came', wait=True)
        self.assertEqual(rec.take_result(), (None, ''))
        self.assertEqual(list(self.dir.iterdir()), [])

    def test_a_broken_camera_or_info_source_never_breaks_the_recorder(self):
        calls = []

        def broken():
            calls.append(1)
            raise RuntimeError('camera exploded')
        rec = R.MissionRecorder(self.dir, broken, info_source=lambda: 1 / 0).start()
        time.sleep(1.0)
        rec.stop('done', wait=True)
        self.assertGreater(len(calls), 2)                                          # it kept trying
        self.assertEqual(rec.take_result(), (None, ''))

    def test_a_worker_that_dies_is_reported_and_nothing_hangs(self):
        rec = R.MissionRecorder(self.dir, self.camera()).start()
        time.sleep(1.5)
        if os.name == 'nt':                                                        # from outside, like a crash
            subprocess.run(['taskkill', '/T', '/F', '/PID', str(rec._proc.pid)], capture_output=True)
        else:
            rec._proc.kill()
        self.assertTrue(rec._done.wait(15))
        result, fault = rec.take_result()
        self.assertIsNone(result)
        self.assertIn('stopped unexpectedly', fault)
        began = time.monotonic()
        rec.stop('after the crash', wait=True)                                     # asking to stop a dead recorder is fine
        self.assertLess(time.monotonic() - began, 2.0)

    def test_the_video_is_saved_when_the_console_goes_away(self):
        rec = R.MissionRecorder(self.dir, self.camera()).start()
        deadline = time.monotonic() + 20
        while rec.status()['seconds'] < 1.0 and time.monotonic() < deadline:
            time.sleep(0.2)
        rec._queue.put(None)                                                       # the console side ends without a stop message:
        self.assertTrue(rec._done.wait(20))                                        # the worker still finishes and saves
        result, fault = rec.take_result()
        self.assertEqual(result.reason, 'the console closed')
        self.assertGreaterEqual(video_info(result.path)['frames'], 8)

    def test_the_worker_runs_at_lower_priority(self):
        real = subprocess.Popen
        seen = []

        def spy(*args, **kwargs):
            seen.append(kwargs)
            return real(*args, **kwargs)
        with mock.patch.object(R.subprocess, 'Popen', spy):
            rec = R.MissionRecorder(self.dir, self.camera()).start()
        rec.stop('test', wait=True)
        self.assertEqual(len(seen), 1)
        if os.name == 'nt':
            self.assertTrue(seen[0]['creationflags'] & subprocess.BELOW_NORMAL_PRIORITY_CLASS)
            self.assertTrue(seen[0]['creationflags'] & subprocess.CREATE_NO_WINDOW)
        else:
            self.assertIn('preexec_fn', seen[0])

    def test_start_refuses_when_it_cannot_work(self):
        small = SimpleNamespace(total=1, used=1, free=200 * 1024 ** 2)
        with mock.patch.object(R.shutil, 'disk_usage', return_value=small):
            with self.assertRaises(R.RecorderError) as caught:
                R.MissionRecorder(self.dir, self.camera()).start()
        self.assertIn('free', str(caught.exception))
        with mock.patch.object(R.importlib.util, 'find_spec', return_value=None):
            with self.assertRaises(R.RecorderError) as caught:
                R.MissionRecorder(self.dir, self.camera()).start()
        self.assertIn('OpenCV', str(caught.exception))
        blocker = self.dir / 'file.txt'
        blocker.write_text('x')
        with self.assertRaises(R.RecorderError):
            R.MissionRecorder(blocker, self.camera()).start()
        with mock.patch.object(R.subprocess, 'Popen', side_effect=OSError('no such program')):
            with self.assertRaises(R.RecorderError) as caught:
                R.MissionRecorder(self.dir, self.camera()).start()
        self.assertIn('could not be started', str(caught.exception))


if __name__ == '__main__':
    unittest.main()
