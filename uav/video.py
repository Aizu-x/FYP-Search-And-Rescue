"""The drone's camera: receive its video, show it, record it, take photos.

The Tello sends H.264 video as UDP datagrams to port 11111 of this computer. All the work on it happens in a
separate worker process (``python -m uav.video --worker``), for the reason the mission recorder has one: decoding
and encoding video inside the console program makes the console stutter. The worker

  * receives the datagrams itself and keeps only those that come from the drone's address,
  * hands them to OpenCV's FFmpeg decoder through a second UDP port on this computer (127.0.0.1), with the
    settings that were measured to keep the delay low (no input buffering, one decoder thread: about one picture
    of delay instead of a full second),
  * sends the console small JPEG pictures for the live preview (about 15 a second),
  * on request writes the footage to a video file in full size and without anything drawn on it (the file is made
    by the same code as the mission videos: ``..._recording.mp4`` while it is being written, renamed when
    complete, as long as the flight, MP4 or else AVI, disk guards), and saves single photos as JPEG.

If the console closes or crashes the worker notices (its input closes), finishes the video file and ends. If the
drone was in the air at that moment the worker also sends it "land" (three times): the console can no longer do
that, and without it the drone would keep its last stick values until its own 15 s safety landing.
"""
from __future__ import annotations

import collections
import importlib.util
import itertools
import os
import pickle
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
VIDEO_PORT = 11111
FOOTAGE_FPS = 30                    # the Tello's camera rate; pictures are repeated or skipped to keep real time
MIN_FREE_START = 1024 ** 3
CAPTURE_OPTIONS = 'fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0|input_format;h264'
CAPTURE_URL = 'udp://127.0.0.1:%d?overrun_nonfatal=1&fifo_size=1000000&buffer_size=4000000'


class VideoError(Exception):
    """The video cannot be received / recorded (the message is meant for the operator)."""


def default_folder():
    """Videos\\UAV footage in the user's home (not the Desktop: that may be synced to the cloud)."""
    return Path.home() / 'Videos' / 'UAV footage'


# ============================================================================================ the console side
class VideoFeed:
    """The console's handle on the worker process. ``start()``, then ``wait_frame()`` for the preview,
    ``record_start()`` / ``record_stop()``, ``photo()``, ``take_events()`` for what was saved, ``stop()``."""

    _ids = itertools.count(1)       # picture numbers never repeat, also not after a reconnect

    def __init__(self, listen_ip, port, source_ip, preview_fps=15.0, preview_width=960, command_port=8889):
        self.listen_ip, self.port, self.source_ip = listen_ip, int(port), source_ip
        self.command_port = int(command_port)      # where the drone takes commands (for the worker's last "land")
        self.preview_fps, self.preview_width = float(preview_fps), int(preview_width)
        self.error = ''                    # why no video can arrive (port in use ...)
        self.dead = False                  # the worker is gone
        self._stopping = False
        self._proc = None
        self._write_lock = threading.Lock()
        self._cond = threading.Condition()
        self._frame = None                 # (id, JPEG bytes, width, height)
        self._frame_time = None
        self._status, self._status_time = {}, None
        self._events = collections.deque(maxlen=100)
        self._ready = threading.Event()
        self._done = threading.Event()

    def start(self, timeout=30.0):
        if importlib.util.find_spec('cv2') is None:
            raise VideoError('OpenCV is not installed (pip install -r requirements-yolo.txt), so the drone\'s '
                             'video cannot be shown')
        kwargs = dict(stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=str(PROJECT),
                      env=dict(os.environ, OPENCV_FFMPEG_CAPTURE_OPTIONS=CAPTURE_OPTIONS, OPENCV_FFMPEG_THREADS='1',
                               OPENCV_LOG_LEVEL='SILENT', OPENCV_FFMPEG_LOGLEVEL='-8'))
        if os.name == 'nt':                # lower priority, and no console window popping up
            kwargs['creationflags'] = (getattr(subprocess, 'BELOW_NORMAL_PRIORITY_CLASS', 0)
                                       | getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        try:
            self._proc = subprocess.Popen([sys.executable, '-u', '-m', 'uav.video', '--worker'], **kwargs)
        except (OSError, ValueError) as exc:
            raise VideoError('the video worker could not be started (%s)' % exc)
        threading.Thread(target=self._read_loop, name='uav-video-read', daemon=True).start()
        self._send(('start', dict(listen_ip=self.listen_ip, port=self.port, source_ip=self.source_ip,
                                  preview_fps=self.preview_fps, preview_width=self.preview_width,
                                  command_port=self.command_port)))
        started = self._ready.wait(timeout)
        if self.error:
            error = self.error
            self.stop(wait=False)
            raise VideoError(error)
        if not started or self.dead:
            self.stop(wait=False)
            raise VideoError('the video worker did not start')
        return self

    # ------------------------------------------------------------------ preview
    def wait_frame(self, after=None, timeout=1.5):
        """The newest picture (id, JPEG bytes, width, height) once its id differs from ``after``; None on timeout."""
        with self._cond:
            if timeout and (self._frame is None or self._frame[0] == after):
                self._cond.wait_for(lambda: (self._frame is not None and self._frame[0] != after) or self.dead,
                                    timeout=timeout)
            frame = self._frame
        return frame if frame is not None and frame[0] != after else None

    def status(self):
        """What the worker last reported, plus how old the newest picture and the report are."""
        now = time.monotonic()
        out = dict(self._status)
        out['frame_age'] = None if self._frame_time is None else now - self._frame_time
        out['report_age'] = None if self._status_time is None else now - self._status_time
        out['dead'], out['port'], out['error'] = self.dead, self.port, self.error
        return out

    # ------------------------------------------------------------------ footage
    def record_start(self, folder, label='', fps=FOOTAGE_FPS):
        self._send(('record', dict(folder=str(folder), label=str(label), fps=float(fps),
                                   stamp=time.strftime('%Y-%m-%d_%H-%M-%S'),
                                   started_text=time.strftime('%Y-%m-%d %H:%M:%S'))))

    def record_stop(self, reason='stopped'):
        self._send(('record_stop', str(reason)))

    def photo(self, folder):
        self._send(('photo', str(folder), time.strftime('%Y-%m-%d_%H-%M-%S')))

    def set_airborne(self, airborne):
        """Tell the worker whether the drone is in the air (it lands the drone if the console dies meanwhile)."""
        self._send(('airborne', bool(airborne)))

    def take_events(self):
        """Messages since the last call: ('record_started', folder), ('record_failed', why),
        ('recorded', Recording or None, fault), ('photo', path or None, why not), ('gone',)."""
        out = []
        while True:
            try:
                out.append(self._events.popleft())
            except IndexError:
                return out

    def stop(self, wait=True, timeout=10.0):
        """End the worker. ``wait``: give it time to finish and name a video that is being recorded."""
        proc = self._proc
        if proc is None:
            return
        self._stopping = True
        self._send(('stop',))
        try:
            proc.stdin.close()
        except (OSError, ValueError):
            pass
        if not (wait and self._done.wait(timeout)):
            self._kill()
        try:
            proc.wait(timeout=3.0)
        except Exception:
            self._kill()
        self.dead = True
        with self._cond:
            self._cond.notify_all()

    # ------------------------------------------------------------------ internals
    def _kill(self):
        """(A virtual environment's python.exe on Windows is a launcher that runs the real interpreter as its
        child, so the whole process tree has to go.)"""
        try:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(self._proc.pid)], capture_output=True, timeout=10)
            self._proc.kill()
        except Exception:
            pass

    def _send(self, message):
        with self._write_lock:
            try:
                pickle.dump(message, self._proc.stdin, protocol=4)
                self._proc.stdin.flush()
                return True
            except (OSError, ValueError, AttributeError):
                return False

    def _read_loop(self):
        from ugv_nav.recorder import Recording
        out = self._proc.stdout
        finished = False
        try:
            while True:
                message = pickle.load(out)
                kind = message[0]
                if kind == 'frame':
                    with self._cond:
                        self._frame = (next(VideoFeed._ids), message[1], message[2], message[3])
                        self._frame_time = time.monotonic()
                        self._cond.notify_all()
                elif kind == 'status':
                    self._status, self._status_time = message[1], time.monotonic()
                elif kind == 'ready':
                    self.port, self.error = message[1], message[2]
                    self._ready.set()
                elif kind == 'recorded':
                    result = None if message[1] is None else Recording.from_plain(message[1])
                    self._events.append(('recorded', result, message[2]))
                elif kind in ('record_started', 'record_failed', 'photo'):
                    self._events.append(tuple(message))
                elif kind == 'done':
                    finished = True
                    break
        except (EOFError, OSError, ValueError, pickle.UnpicklingError, IndexError, TypeError):
            pass
        if not finished and not self._stopping:    # the worker crashed or was killed
            self._events.append(('gone',))
        self.dead = True
        self._done.set()
        self._ready.set()
        with self._cond:
            self._cond.notify_all()


# ============================================================================================ the worker process
class _Worker:
    def __init__(self, cfg, send):
        self.listen_ip, self.port, self.source_ip = cfg['listen_ip'], int(cfg['port']), cfg['source_ip']
        self.preview_period = 1.0 / max(1.0, float(cfg.get('preview_fps', 15.0)))
        self.preview_width = int(cfg.get('preview_width', 960))
        self.command_address = (self.source_ip, int(cfg.get('command_port', 8889)))
        self.airborne = False                  # told by the console
        self.send = send
        self.stop = threading.Event()
        self.lock = threading.Lock()           # the newest picture and the video file
        self.core = None
        self.picture = None
        self.picture_time = None
        self.packets = self.bytes = self.rejected = 0
        self.last_packet = None
        self.frames = 0
        self.size = (0, 0)
        self.opens = 0
        self.sock = self.forward = None
        self.forward_port = 0

    def start(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            except OSError:
                pass
            sock.bind((self.listen_ip, self.port))
            sock.settimeout(0.25)
        except OSError as exc:
            self.send(('ready', self.port, 'port %d of this computer is used by another program (another drone app, '
                                           'or a second console?), so no video can arrive (%s)'
                       % (self.port, exc.strerror or exc)))
            return False
        self.sock, self.port = sock, sock.getsockname()[1]
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)       # a free port for the hand-over to the decoder
        probe.bind(('127.0.0.1', 0))
        self.forward_port = probe.getsockname()[1]
        probe.close()
        self.forward = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for target, name in ((self._receive, 'receive'), (self._decode, 'decode'), (self._report, 'report')):
            threading.Thread(target=target, name=name, daemon=True).start()
        self.send(('ready', self.port, ''))
        return True

    # ------------------------------------------------------------------ threads
    def _receive(self):
        sock, forward, target, source = self.sock, self.forward, ('127.0.0.1', self.forward_port), self.source_ip
        while not self.stop.is_set():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self.stop.wait(0.05):
                    break
                continue
            if address[0] != source:               # not from the drone: dropped
                self.rejected += 1
                continue
            self.packets += 1
            self.bytes += len(data)
            self.last_packet = time.monotonic()
            try:
                forward.sendto(data, target)
            except OSError:
                pass

    def _decode(self):
        import cv2
        url = CAPTURE_URL % self.forward_port
        settings = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 4000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2500]
        capture, next_preview = None, 0.0
        while not self.stop.is_set():
            if capture is None:                    # waits (inside FFmpeg) until video arrives; tried again and again
                self.opens += 1
                capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG, settings)
                if not capture.isOpened():
                    capture.release()
                    capture = None
                    self.stop.wait(0.05)
                    continue
            ok, picture = capture.read()
            if not ok or picture is None:          # the video stopped: start again from a clean decoder
                capture.release()
                capture = None
                continue
            now, wall = time.monotonic(), time.time()
            height, width = picture.shape[:2]
            ended = False
            with self.lock:
                self.frames += 1
                self.picture, self.picture_time, self.size = picture, now, (width, height)
                core = self.core
                if core is not None:
                    try:
                        ended = core.feed(wall, (self.frames, picture, []), None)
                    except Exception as exc:
                        core.fault = core.fault or 'the video file could not be written: %s' % exc
                        ended = True
                    if ended:
                        self.core = None
            if ended:
                self._finish(core, wall, '')
            if now >= next_preview:
                next_preview = now + self.preview_period * 0.9
                if width > self.preview_width:
                    scale = self.preview_width / float(width)
                    picture = cv2.resize(picture, (self.preview_width, int(round(height * scale))),
                                         interpolation=cv2.INTER_AREA)
                good, jpeg = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if good:
                    self.send(('frame', jpeg.tobytes(), picture.shape[1], picture.shape[0]))
        if capture is not None:
            capture.release()

    def _report(self):
        last_t, last_frames, last_bytes = time.monotonic(), 0, 0
        while not self.stop.wait(0.5):
            now = time.monotonic()
            dt = max(now - last_t, 1e-3)
            fps, kbps = (self.frames - last_frames) / dt, (self.bytes - last_bytes) * 8 / 1000.0 / dt
            last_t, last_frames, last_bytes = now, self.frames, self.bytes
            core = self.core
            recording = None if core is None else dict(frames=core.frames, seconds=core.seconds, bytes=core.bytes)
            self.send(('status', dict(
                packets=self.packets, rejected=self.rejected, frames=self.frames, fps=round(fps, 1),
                kbps=round(kbps), width=self.size[0], height=self.size[1],
                packet_age=None if self.last_packet is None else round(now - self.last_packet, 2),
                picture_age=None if self.picture_time is None else round(now - self.picture_time, 2),
                recording=recording)))

    # ------------------------------------------------------------------ footage
    def record(self, cfg):
        from ugv_nav.recorder import RecorderError, VideoCore, check_folder
        with self.lock:
            if self.core is not None:
                return self.send(('record_failed', 'a video is already being recorded'))
            try:
                folder = check_folder(cfg['folder'])
                free = shutil.disk_usage(folder).free
                if free < MIN_FREE_START:
                    raise RecorderError('only %.1f GB are free on that disk (at least %.0f GB are needed)'
                                        % (free / 1024 ** 3, MIN_FREE_START / 1024 ** 3))
                self.core = VideoCore(folder, cfg.get('label', ''), fps=cfg.get('fps', FOOTAGE_FPS), overlay=False,
                                      stamp=cfg.get('stamp'), started_text=cfg.get('started_text'), prefix='flight')
            except RecorderError as exc:
                return self.send(('record_failed', str(exc)))
        return self.send(('record_started', str(folder)))

    def record_stop(self, reason):
        with self.lock:
            core, self.core = self.core, None
        if core is None:
            return self.send(('recorded', None, 'no video was being recorded'))
        return self._finish(core, time.time(), reason)

    def _finish(self, core, t_end, reason):
        result, fault = core.finish(t_end, reason)
        self.send(('recorded', None if result is None else result.to_plain(), fault))

    def photo(self, folder, stamp):
        import cv2
        from ugv_nav.recorder import RecorderError, check_folder
        with self.lock:
            picture, taken = self.picture, self.picture_time
        if picture is None or taken is None or time.monotonic() - taken > 2.0:
            return self.send(('photo', None, 'there is no live picture from the drone right now'))
        try:
            folder = check_folder(folder)
            path = folder / ('photo_%s.jpg' % stamp)
            number = 1
            while path.exists():
                number += 1
                path = folder / ('photo_%s_%d.jpg' % (stamp, number))
            good, jpeg = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not good:
                raise OSError('the picture could not be encoded')
            path.write_bytes(jpeg.tobytes())
        except (RecorderError, OSError) as exc:
            return self.send(('photo', None, str(exc)))
        return self.send(('photo', str(path), ''))

    def land_drone(self):
        """The console is gone while the drone is in the air: tell the drone to land (best effort)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((self.listen_ip, 0))
                for _ in range(3):
                    sock.sendto(b'land', self.command_address)
                    time.sleep(0.05)
        except OSError:
            pass

    def shutdown(self, reason):
        self.stop.set()
        with self.lock:
            core, self.core = self.core, None
        if core is not None:
            self._finish(core, time.time(), reason)
        for sock in (self.sock, self.forward):
            try:
                if sock is not None:
                    sock.close()
            except OSError:
                pass


def worker_main():
    """Runs in its own process: pickled messages in on stdin, pickled messages out on stdout."""
    out = os.fdopen(os.dup(1), 'wb')                           # the real stdout: only protocol messages go there
    os.dup2(os.open(os.devnull, os.O_WRONLY), 1)               # anything a library prints must not corrupt it
    write_lock = threading.Lock()

    def send(message):
        with write_lock:
            try:
                pickle.dump(message, out, protocol=4)
                out.flush()
            except (OSError, ValueError):
                pass

    source = sys.stdin.buffer
    worker, reason, console_gone = None, 'the console closed', False
    try:
        import cv2                                              # noqa: F401  (a clear message instead of a crash)
        first = pickle.load(source)
        worker = _Worker(first[1], send)
        if not worker.start():
            return 1
        while True:
            try:
                message = pickle.load(source)
            except Exception:                                   # the console went away (or its last message was
                console_gone = True                             # cut off): keep what was filmed
                break
            try:
                kind = message[0]
                if kind == 'record':
                    worker.record(message[1])
                elif kind == 'record_stop':
                    worker.record_stop(message[1])
                elif kind == 'photo':
                    worker.photo(message[1], message[2])
                elif kind == 'airborne':
                    worker.airborne = bool(message[1])
                elif kind == 'stop':
                    reason = 'the drone was disconnected'
                    break
            except Exception as exc:                            # one bad request must not end the live picture
                send(('record_failed', 'the video worker could not do that (%s)' % exc))
    except Exception as exc:
        send(('ready', 0, 'the video worker failed: %s' % exc))
    finally:
        if worker is not None:
            if console_gone and worker.airborne:
                worker.land_drone()
            worker.shutdown(reason)
        send(('done',))
    return 0


if __name__ == '__main__':
    sys.exit(worker_main() if '--worker' in sys.argv else 0)
