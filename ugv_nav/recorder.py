"""Record the robot's camera view of a search mission and save it as a video file on this computer.

What is recorded: the camera frames the console already receives from the robot (the same frames YOLO looks
at), so nothing extra is asked of the robot or the Wi-Fi. The robot itself stores nothing.

Why a separate process: decoding, drawing and encoding video inside the program that drives the robot makes
that program stutter (measured: the odometry went stale for 0.5 s and missions paused). So the console only
forwards the pictures it already has to a small worker process (``python -m ugv_nav.recorder --worker``) that
runs at lower priority and does all the video work. The worker needs no clock of its own: every picture arrives
with the time it was taken, and the video is built from those times.

* One frame is written every 1/``fps`` s of that time, so the video is exactly as long as the mission, whatever
  the camera frame rate: when no new picture has arrived the last one is repeated (and the picture says how old
  it is), when pictures arrive faster the extra ones are skipped.
* Optional overlay (default on): time since the mission started, what the robot is doing, survivors found /
  secured, the latest mission event for a few seconds, the YOLO boxes and the wall-clock time. Switched off, the
  footage is the plain camera picture.
* MP4 (``mp4v``) when this computer's OpenCV can write it, otherwise Motion-JPEG AVI. While recording the file
  is called ``..._recording.mp4``; it gets its final name only when the video is complete. If the console
  closes or crashes the worker still finishes and saves the video.
* Safeguards: refuses to start with less than 1 GB free, stops (and keeps what it has) when the disk gets
  below 300 MB free or after 60 minutes, and any problem in the recorder is reported but never touches the
  mission or the robot.
"""
from __future__ import annotations

import importlib.util
import os
import pickle
import queue
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FPS = 10
MIN_FREE_START = 1024 ** 3          # do not start with less than this free
MIN_FREE_RUN = 300 * 1024 ** 2      # stop and keep the video when the disk drops below this
MAX_SECONDS = 60 * 60               # one video never gets longer than this
EVENT_SHOW_S = 6.0                  # the latest mission event stays on the picture this long
STALE_AFTER_S = 1.5                 # the picture says so when the newest camera frame is older than this
DISK_CHECK_FRAMES = 50              # the free disk space is looked at every this many frames (5 s of video)
MAX_GAP_S = 60.0                    # a hold of the same picture longer than this is not written out (clock jump)
MAX_BACKLOG = 600                   # pictures waiting for the worker (60 s); more means it cannot keep up
CODECS = (('mp4', 'mp4v'), ('avi', 'MJPG'))      # tried in this order
ROOT = Path(__file__).resolve().parents[1]

GREEN, RED, GREY = (134, 212, 47), (100, 89, 255), (170, 170, 170)    # BGR, same colours as the live view


class RecorderError(Exception):
    """The recording cannot start (message is meant for the operator)."""


@dataclass
class Recording:
    path: Path
    frames: int
    seconds: float
    size_bytes: int
    reason: str
    started: str
    codec: str

    @property
    def name(self):
        return self.path.name

    def as_dict(self):
        return dict(path=str(self.path), name=self.name, frames=self.frames, seconds=round(self.seconds, 1),
                    size_mb=round(self.size_bytes / 1024 ** 2, 1), reason=self.reason, started=self.started,
                    codec=self.codec)

    def to_plain(self):
        return (str(self.path), self.frames, self.seconds, self.size_bytes, self.reason, self.started, self.codec)

    @classmethod
    def from_plain(cls, plain):
        path, frames, seconds, size, reason, started, codec = plain
        return cls(Path(path), frames, seconds, size, reason, started, codec)


def default_folder():
    """Videos\\UGV missions in the user's home (not the Desktop: that may be synced to the cloud)."""
    return Path.home() / 'Videos' / 'UGV missions'


def check_folder(folder):
    """The folder as a Path, created and proven writable; RecorderError otherwise."""
    folder = Path(folder).expanduser()
    try:
        folder.mkdir(parents=True, exist_ok=True)
        probe = folder / '.ugv_write_test'
        probe.write_bytes(b'x')
        probe.unlink()
    except (OSError, ValueError) as exc:
        raise RecorderError('the folder "%s" cannot be used (%s)' % (folder, getattr(exc, 'strerror', None) or exc))
    return folder


def _ascii(text):
    """OpenCV's drawing fonts know plain ASCII only."""
    return ''.join(ch if 32 <= ord(ch) < 127 else '-' for ch in str(text))


def _clock_text(seconds):
    seconds = int(max(0, seconds))
    return '%02d:%02d' % (seconds // 60, seconds % 60)


# ============================================================================================ the video file
class VideoCore:
    """The video file itself, driven only by the times it is told: no threads, no clock of its own.

    ``feed(t, frame, info)``: at time ``t`` (seconds) the newest camera frame was ``frame`` = (id, JPEG bytes,
    boxes) -- or None when there is nothing newer -- and the mission said ``info``. ``finish(t_end, reason)`` ends
    the video at time ``t_end``. The picture may also be given already decoded (a BGR array), and ``prefix`` is
    the first word of the file name: the drone's footage (uav/video.py) is written by this same class."""

    def __init__(self, folder, label='', fps=FPS, overlay=True, stamp=None, started_text=None, prefix='mission'):
        self.folder = Path(folder)
        self.label = ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in str(label))
        self.prefix = ''.join(ch for ch in str(prefix) if ch.isalnum() or ch in '-_') or 'mission'
        self.fps = min(60.0, max(1.0, float(fps)))
        self.overlay = bool(overlay)
        self.stamp = stamp or time.strftime('%Y-%m-%d_%H-%M-%S')
        self.started_text = started_text or time.strftime('%Y-%m-%d %H:%M:%S')
        self.frames = 0
        self.bytes = 0
        self.fault = ''                   # why the video stopped by itself
        self.stop_reason = ''
        self.image, self.boxes, self.info = None, [], {}
        self.last_new = None              # time of the newest camera picture
        self.next_tick = None             # time of the next video frame
        self._writer = None
        self._size = None
        self._temp = None
        self._codec = ''
        self._ended = False

    @property
    def seconds(self):
        return self.frames / self.fps

    @property
    def ended(self):
        return self._ended

    def feed(self, t, frame=None, info=None):
        """True when the video has ended by itself (disk almost full, maximum length)."""
        if self._ended:
            return True
        if self.image is None:
            if info is not None:
                self.info = info
            if frame is None or not self._take(frame, t):
                return False
            self._open_writer(self.image.shape[1], self.image.shape[0])      # the video starts with the first picture
            self.next_tick = t
            return False
        self._write_until(t)
        if self._ended:
            return True
        if info is not None:
            self.info = info
        if frame is not None:
            self._take(frame, t)
        return False

    def finish(self, t_end, reason='finished'):
        """Write up to ``t_end``, close and name the file. (Recording or None, fault text)."""
        if not self._ended and self.image is not None:
            self._write_until(t_end)
        self._ended = True
        result = None
        try:
            if self._writer is not None:
                self._writer.release()
            if self._temp is not None:
                if self.frames == 0:
                    self._temp.unlink()
                else:
                    final = self._temp.with_name(self._temp.name.replace('_recording.', '.'))
                    for attempt in range(5):                   # (an antivirus or a cloud client may hold the file a moment)
                        try:
                            os.replace(self._temp, final)
                            break
                        except OSError:
                            if attempt == 4:
                                final = self._temp
                            else:
                                time.sleep(0.4)
                    self.bytes = final.stat().st_size
                    result = Recording(final, self.frames, self.seconds, self.bytes,
                                       self.stop_reason or reason or 'finished', self.started_text, self._codec)
        except Exception as exc:
            self.fault = self.fault or 'the video could not be saved: %s' % exc
        return result, self.fault

    # ------------------------------------------------------------------ internals
    def _take(self, frame, t):
        import cv2
        data = frame[1]
        if isinstance(data, np.ndarray):                       # already a picture
            decoded = data if data.dtype == np.uint8 and data.ndim == 3 and data.shape[2] == 3 else None
        else:
            decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            return False
        self.image, self.boxes, self.last_new = decoded, (frame[2] if len(frame) > 2 else []), t
        return True

    def _write_until(self, t):
        period = 1.0 / self.fps
        if t - self.next_tick > MAX_GAP_S:                     # a clock jump: do not write minutes of the same picture
            self.next_tick = t - MAX_GAP_S
        while self.next_tick < t - 1e-9 and not self._ended:
            self._write(self.next_tick)
            self.next_tick += period

    def _write(self, tick):
        import cv2
        if self.overlay:
            picture = self._compose(self.image, self.boxes, tick - self.last_new, self.info, tick)
        else:
            picture = self.image
        if (picture.shape[1], picture.shape[0]) != self._size:
            picture = cv2.resize(picture, self._size)
        self._writer.write(picture)
        self.frames += 1
        if self.frames % max(1, int(self.fps)) == 0:
            try:
                self.bytes = self._temp.stat().st_size
            except OSError:
                pass
        if self.frames % DISK_CHECK_FRAMES == 0:
            self._guards()

    def _guards(self):
        if self.seconds >= MAX_SECONDS:
            self.stop_reason = 'maximum length reached'
            self.fault = 'the video reached its maximum length of %d minutes' % (MAX_SECONDS // 60)
            self._ended = True
        elif shutil.disk_usage(self.folder).free < MIN_FREE_RUN:
            self.stop_reason = 'disk almost full'
            self.fault = 'the disk is almost full: the video was saved up to here'
            self._ended = True

    def _open_writer(self, width, height):
        import cv2
        for extension, code in CODECS:
            path = self.folder / ('%s_%s%s_recording.%s' % (self.prefix, self.stamp,
                                                            '_' + self.label if self.label else '', extension))
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*code), self.fps, (width, height))
            if writer.isOpened():
                self._writer, self._size, self._temp, self._codec = writer, (width, height), path, code
                return
            writer.release()
            try:
                path.unlink()
            except OSError:
                pass
        raise RecorderError('this computer\'s OpenCV cannot write MP4 or AVI video')

    # ------------------------------------------------------------------ the picture
    def _compose(self, image, boxes, age, info, tick_time):
        import cv2
        out = image.copy()
        h, w = out.shape[:2]
        s = max(0.6, h / 480.0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for b in boxes or []:                                  # YOLO boxes, same colours as the live view
            try:
                x1, y1, x2, y2 = (int(round(v)) for v in b['box'])
            except (KeyError, TypeError, ValueError):
                continue
            green = str(b.get('cls', '')).startswith('green')
            used = bool(b.get('used'))
            colour = (GREEN if green else RED) if used else GREY
            cv2.rectangle(out, (x1, y1), (x2, y2), colour, max(1, int(round((2 if used else 1) * s))), cv2.LINE_AA)
            label = '%s %.2f' % ('green' if green else 'red', float(b.get('conf', 0.0)))
            if b.get('range') is not None:
                label += ' %.2fm' % float(b['range'])
            if b.get('note') and not used:
                label += ' ' + _ascii(b['note'])
            (tw, th), base = cv2.getTextSize(label, font, 0.45 * s, 1)
            ly = y1 - 4 if y1 - th - 8 > 0 else y2 + th + 6
            cv2.rectangle(out, (x1, ly - th - 3), (x1 + tw + 6, ly + base), colour, -1)
            cv2.putText(out, label, (x1 + 3, ly), font, 0.45 * s, (10, 20, 10), 1, cv2.LINE_AA)
        bar = int(round(24 * s))
        out[:bar] = (out[:bar] * 0.4).astype(np.uint8)         # darkened strips behind the text
        out[h - bar:] = (out[h - bar:] * 0.4).astype(np.uint8)
        scale, base_y = 0.5 * s, int(round(16 * s))
        low_y = h - int(round(7 * s))

        def width(text):
            return cv2.getTextSize(text, font, scale, 1)[0][0]

        def fit(text, room):                                   # never let two texts run into each other
            if width(text) <= room:
                return text
            while text and width(text + '..') > room:
                text = text[:-1]
            return text + '..' if text else ''
        if info:
            left = '%s  %s' % (_clock_text(info.get('elapsed', 0.0)), _ascii(info.get('phase', '')))
            right = '%d found | %d secured' % (info.get('found', 0), info.get('secured', 0))
        else:
            left, right = _clock_text(self.seconds), ''
        left = fit(left, w - width(right) - 28)
        cv2.putText(out, left, (8, base_y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
        if right:
            cv2.putText(out, right, (w - width(right) - 8, base_y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
        stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(tick_time))
        cv2.putText(out, stamp, (w - width(stamp) - 8, low_y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
        note, colour = '', (255, 255, 255)
        event = info.get('event') if info else None
        if age > STALE_AFTER_S:
            note, colour = 'camera frame is %d s old' % int(age), (60, 170, 255)
        elif event and 0.0 <= info.get('elapsed', 0.0) - event[0] <= EVENT_SHOW_S:
            note = _ascii(event[1])
        if note:
            cv2.putText(out, fit(note, w - width(stamp) - 32), (8, low_y), font, scale, colour, 1, cv2.LINE_AA)
        return out


# ============================================================================================ the console side
def _plain_boxes(boxes):
    """Only what the video needs, as plain Python values (small and safe to send to the worker)."""
    out = []
    for b in boxes or []:
        try:
            out.append(dict(box=[float(v) for v in b['box']], cls=str(b.get('cls', '')), conf=float(b.get('conf', 0.0)),
                            range=None if b.get('range') is None else float(b['range']), used=bool(b.get('used')),
                            note=str(b.get('note') or '')))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _plain_info(info):
    if not info:
        return None
    event = info.get('event')
    try:
        return dict(phase=str(info.get('phase', '')), elapsed=float(info.get('elapsed', 0.0)),
                    found=int(info.get('found', 0)), secured=int(info.get('secured', 0)),
                    event=None if not event else (float(event[0]), str(event[1])))
    except (TypeError, ValueError, IndexError):
        return None


class MissionRecorder:
    """Films a mission. ``frame_source()`` -> (frame id, JPEG bytes, width, height, boxes) or None, ``info_source()`` ->
    dict(phase, elapsed, found, secured, event=(seconds since mission start, text)). Both are called about ten times
    a second from a light thread; all video work happens in a separate worker process."""

    def __init__(self, folder, frame_source, info_source=None, label='', fps=FPS, overlay=True,
                 clock=time.monotonic):
        self.folder = Path(folder)
        self.frame_source = frame_source
        self.info_source = info_source
        self.label = str(label)
        self.fps = min(60.0, max(1.0, float(fps)))
        self.overlay = bool(overlay)
        self.clock = clock
        self.frames = 0                   # reported by the worker about once a second
        self.fault = ''                   # why the recording stopped by itself / could not be saved
        self._bytes = 0
        self._proc = None
        self._queue = queue.Queue(maxsize=MAX_BACKLOG)
        self._stop_at = None
        self._reason = ''
        self._wake = threading.Event()
        self._done = threading.Event()
        self._result = None
        self._collected = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ control (any thread)
    def start(self):
        if importlib.util.find_spec('cv2') is None:
            raise RecorderError('OpenCV is not installed (pip install -r requirements-yolo.txt)')
        self.folder = check_folder(self.folder)
        free = shutil.disk_usage(self.folder).free
        if free < MIN_FREE_START:
            raise RecorderError('only %.1f GB are free on that disk (at least %.0f GB are needed)'
                                % (free / 1024 ** 3, MIN_FREE_START / 1024 ** 3))
        self._spawn()
        self._queue.put(('start', dict(folder=str(self.folder), label=self.label, fps=self.fps, overlay=self.overlay,
                                       stamp=time.strftime('%Y-%m-%d_%H-%M-%S'),
                                       started_text=time.strftime('%Y-%m-%d %H:%M:%S'))))
        for target, name in ((self._send_loop, 'recorder-send'), (self._read_loop, 'recorder-read'),
                             (self._poll_loop, 'recorder-poll')):
            threading.Thread(target=target, name=name, daemon=True).start()
        return self

    def stop(self, reason='stopped', tail_s=0.0, wait=False):
        """Finish the video: keep recording ``tail_s`` more seconds, then save. ``wait``: return only when saved."""
        with self._lock:
            if self._stop_at is None:
                self._stop_at = self.clock() + max(0.0, tail_s)
                self._reason = reason
                if tail_s <= 0:
                    self._wake.set()
        if wait and not self._done.wait(30.0):
            self._kill('the video worker did not finish in time')

    @property
    def stopping(self):
        return self._stop_at is not None

    @property
    def done(self):
        return self._done.is_set()

    @property
    def seconds(self):
        return self.frames / self.fps

    def status(self):
        return dict(frames=self.frames, seconds=self.seconds, mb=round(self._bytes / 1024 ** 2, 1),
                    stopping=self.stopping, done=self.done, waiting=self.frames == 0 and not self.done)

    def take_result(self):
        """(Recording or None, fault text), once, after the recording is done; (None, '') before that or after."""
        if not self.done or self._collected:
            return None, ''
        self._collected = True
        return self._result, self.fault

    # ------------------------------------------------------------------ the worker process
    def _spawn(self):
        kwargs = dict(stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=str(ROOT))
        if os.name == 'nt':                # lower priority, and no console window popping up
            kwargs['creationflags'] = (getattr(subprocess, 'BELOW_NORMAL_PRIORITY_CLASS', 0)
                                       | getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        else:
            kwargs['preexec_fn'] = lambda: os.nice(10)
        try:
            self._proc = subprocess.Popen([sys.executable, '-u', '-m', 'ugv_nav.recorder', '--worker'], **kwargs)
        except (OSError, ValueError) as exc:
            raise RecorderError('the video worker could not be started (%s)' % exc)

    def _kill(self, why):
        """Stop the worker for good. (A virtual environment's python.exe on Windows is a small launcher that runs
        the real interpreter as its child, so the whole process tree has to go.)"""
        self.fault = self.fault or why
        try:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(self._proc.pid)], capture_output=True, timeout=10)
            self._proc.kill()
        except Exception:
            pass

    def _send_loop(self):
        stdin = self._proc.stdin
        try:
            while True:
                message = self._queue.get()
                if message is None:
                    break
                pickle.dump(message, stdin, protocol=4)
                stdin.flush()
        except (OSError, ValueError):
            pass                             # the worker is gone: the reader reports it
        finally:
            try:
                stdin.close()
            except Exception:
                pass

    def _read_loop(self):
        out = self._proc.stdout
        try:
            while True:
                message = pickle.load(out)
                if message[0] == 'status':
                    self.frames, self._bytes = message[1], message[2]
                elif message[0] == 'done':
                    result, fault = message[1], message[2]
                    self._result = None if result is None else Recording.from_plain(result)
                    if result is not None:
                        self.frames, self._bytes = self._result.frames, self._result.size_bytes
                    self.fault = self.fault or fault
                    break
        except (EOFError, OSError, ValueError, pickle.UnpicklingError, IndexError):
            if self._result is None and not self.fault:
                self.fault = 'the video worker stopped unexpectedly: nothing was saved'
        try:
            self._proc.wait(timeout=5)
        except Exception:
            self._kill('')
        self._done.set()
        self._wake.set()

    def _enqueue(self, message):
        try:
            self._queue.put(message, timeout=2.0)
            return True
        except queue.Full:
            self._kill('the video worker cannot keep up')
            return False

    def _poll_loop(self):
        period = 1.0 / self.fps
        last_id = None
        next_at = self.clock()
        try:
            while not self._done.is_set():
                now = self.clock()
                with self._lock:
                    stop_at, reason = self._stop_at, self._reason
                if stop_at is not None and now >= stop_at:
                    self._enqueue(('stop', time.time(), reason or 'stopped'))
                    return
                payload = None
                try:
                    frame = self.frame_source()
                except Exception:
                    frame = None
                if frame is not None and frame[0] != last_id:
                    last_id = frame[0]
                    payload = (frame[0], bytes(frame[1]), _plain_boxes(frame[4] if len(frame) > 4 else []))
                try:
                    info = _plain_info(self.info_source() if self.info_source is not None else None)
                except Exception:
                    info = None
                if not self._enqueue(('tick', time.time(), payload, info)):
                    return
                next_at += period
                if next_at < now:
                    next_at = now + period       # never try to catch up on missed polls
                self._wake.wait(max(0.0, min(next_at - self.clock(), period)))
        finally:
            try:
                self._queue.put(None, timeout=2.0)
            except queue.Full:
                self._kill('the video worker cannot keep up')


# ============================================================================================ the worker process
def worker_main():
    """Runs in its own process: reads pickled messages from stdin, writes the video, reports on stdout."""
    out = os.fdopen(os.dup(1), 'wb')                           # the real stdout: only protocol messages go there
    os.dup2(os.open(os.devnull, os.O_WRONLY), 1)               # anything a library prints must not corrupt it

    def send(message):
        pickle.dump(message, out, protocol=4)
        out.flush()

    def done(core, t_end, reason):
        result, fault = core.finish(t_end, reason)
        send(('done', None if result is None else result.to_plain(), fault))

    source = sys.stdin.buffer
    try:
        import cv2                                              # noqa: F401  (a clear message instead of a crash)
        first = pickle.load(source)
        core = VideoCore(**first[1])
        reported = 0
        while True:
            try:
                message = pickle.load(source)
            except EOFError:                                    # the console went away: keep what was filmed
                done(core, time.time(), 'the console closed')
                return 0
            if message[0] == 'tick':
                ended = core.feed(message[1], message[2], message[3])
                if core.frames - reported >= core.fps or ended:
                    reported = core.frames
                    send(('status', core.frames, core.bytes))
                if ended:
                    done(core, message[1], '')
                    return 0
            elif message[0] == 'stop':
                done(core, message[1], message[2])
                return 0
    except Exception as exc:
        try:
            send(('done', None, 'the video recording failed: %s' % exc))
        except Exception:
            pass
        return 1


if __name__ == '__main__':
    sys.exit(worker_main() if '--worker' in sys.argv else 0)
