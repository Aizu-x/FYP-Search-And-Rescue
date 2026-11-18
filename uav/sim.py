"""A simulated Tello on this computer, for practice and for the tests.

It speaks the same UDP text protocol as the real drone, so ``uav/tello.py`` and the video receiver are used
exactly as with the real one: it answers "command", "takeoff", "land", "emergency", "streamon", "streamoff",
"rc a b c d" and the read commands, sends its status ten times a second and, after "streamon", real H.264 video
of a drawn room (with the UGV arena on the floor) as seen from the drone's camera.

From the SDK document: the command names and answers, the status text, landing by itself after 15 s without
any command. Everything about HOW it flies is made up (speeds, how quickly it reacts, battery drain, take-off
height): it is for learning the controls and for testing the console, not a model of the real aircraft.

    python -m uav.sim            run it as its own process (the console does this for "Simulated drone")
"""
from __future__ import annotations

import argparse
import collections
import importlib.util
import math
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .h264 import IpcmEncoder

PROJECT = Path(__file__).resolve().parents[1]
CHUNK = 1460                       # the real drone sends its video in datagrams of at most this size
ROOM = 4.0                         # the drawn room: 8 x 8 m, the drone starts in the middle facing +x
CEILING = 3.0
ARENA = (0.9, -1.38, 3.66, 1.38)   # the UGV arena on the floor (x0, y0, x1, y1): 2.76 m square
HFOV_DEG, PITCH_DOWN_DEG = 70.0, 10.0


# ---------------------------------------------------------------------------------------------- the drawn room
def _box(x0, y0, z0, x1, y1, z1, colour):
    """An axis-aligned box: (centre, [(corner points, outward normal, colour)])."""
    base = np.array(colour, float)

    def face(points, normal, shade):
        return np.array(points, float), np.array(normal, float), tuple(int(v) for v in base * shade)
    faces = [face([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1), 1.0),
             face([(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)], (0, 0, -1), 0.4),
             face([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0), 0.82),
             face([(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)], (-1, 0, 0), 0.70),
             face([(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)], (0, 1, 0), 0.90),
             face([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0), 0.62)]
    return np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0]), faces


def _clip_near(points, near=0.05):
    """Cut a polygon (camera coordinates: right, down, depth) at the plane depth = near."""
    out, n = [], len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        a_in, b_in = a[2] >= near, b[2] >= near
        if a_in:
            out.append(a)
        if a_in != b_in:
            t = (near - a[2]) / (b[2] - a[2])
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), near))
    return out


class Scene:
    """What the simulated camera looks at. Colours are BGR."""

    def __init__(self):
        r, top = ROOM, CEILING
        ax0, ay0, ax1, ay1 = ARENA

        def quad(points, colour):
            return np.array(points, float), colour
        self.flat = [                                                  # drawn first, in this order
            quad([(-r, -r, top), (r, -r, top), (r, r, top), (-r, r, top)], (214, 214, 210)),      # ceiling
            quad([(-r, -r, 0), (r, -r, 0), (r, r, 0), (-r, r, 0)], (104, 100, 96)),               # floor
            quad([(r, -r, 0), (r, r, 0), (r, r, top), (r, -r, top)], (196, 178, 150)),            # wall ahead
            quad([(-r, -r, 0), (-r, r, 0), (-r, r, top), (-r, -r, top)], (150, 176, 200)),        # wall behind
            quad([(-r, r, 0), (r, r, 0), (r, r, top), (-r, r, top)], (160, 190, 165)),            # wall on the left
            quad([(-r, -r, 0), (r, -r, 0), (r, -r, top), (-r, -r, top)], (170, 165, 200)),        # wall on the right
            quad([(r, -1.2, 1.1), (r, 1.2, 1.1), (r, 1.2, 2.3), (r, -1.2, 2.3)], (238, 222, 180)),     # window ahead
            quad([(-r, -0.5, 0), (-r, 0.5, 0), (-r, 0.5, 2.05), (-r, -0.5, 2.05)], (60, 85, 120)),     # door behind
            quad([(-1.5, r, 0.9), (0.5, r, 0.9), (0.5, r, 2.0), (-1.5, r, 2.0)], (70, 72, 70)),        # board, left wall
            quad([(1.0, -r, 0.8), (2.6, -r, 0.8), (2.6, -r, 2.2), (1.0, -r, 2.2)], (80, 120, 190)),    # poster, right wall
        ]
        self.lines = []                                                # 1 m grid on the floor
        for k in range(-int(r), int(r) + 1):
            self.lines.append(((float(k), -r, 0.0), (float(k), r, 0.0)))
            self.lines.append(((-r, float(k), 0.0), (r, float(k), 0.0)))
        self.paint = [                                                 # painted on the floor, drawn after the grid
            quad([(ax0, ay0, 0), (ax1, ay0, 0), (ax1, ay1, 0), (ax0, ay1, 0)], (178, 182, 184)),     # arena floor
            quad([(-0.3, -0.3, 0), (0.3, -0.3, 0), (0.3, 0.3, 0), (-0.3, 0.3, 0)], (150, 95, 35)),   # landing pad
            quad([(-0.18, -0.2, 0), (-0.10, -0.2, 0), (-0.10, 0.2, 0), (-0.18, 0.2, 0)], (245, 245, 245)),
            quad([(0.10, -0.2, 0), (0.18, -0.2, 0), (0.18, 0.2, 0), (0.10, 0.2, 0)], (245, 245, 245)),
            quad([(-0.10, -0.04, 0), (0.10, -0.04, 0), (0.10, 0.04, 0), (-0.10, 0.04, 0)], (245, 245, 245)),
        ]
        wall, t, h, parts = (232, 232, 232), 0.03, 0.10, 4
        self.boxes = []
        for k in range(parts):                                         # the low arena walls, in short pieces
            x_a, x_b = ax0 + (ax1 - ax0) * k / parts, ax0 + (ax1 - ax0) * (k + 1) / parts
            y_a, y_b = ay0 + (ay1 - ay0) * k / parts, ay0 + (ay1 - ay0) * (k + 1) / parts
            self.boxes += [_box(x_a, ay0 - t, 0, x_b, ay0, h, wall), _box(x_a, ay1, 0, x_b, ay1 + t, h, wall),
                           _box(ax0 - t, y_a, 0, ax0, y_b, h, wall), _box(ax1, y_a, 0, ax1 + t, y_b, h, wall)]
        green, red = (70, 200, 60), (50, 50, 215)
        for x, y, colour in ((1.6, 0.6, green), (3.0, -0.8, green), (2.3, -0.2, red), (2.9, 0.9, red)):
            self.boxes.append(_box(x - 0.05, y - 0.05, 0, x + 0.05, y + 0.05, 0.10, colour))   # 10 cm blocks
        self.boxes += [_box(-2.65, 2.35, 0, -2.35, 2.65, 1.8, (40, 140, 240)),                 # orange pillar
                       _box(2.5, -3.1, 0, 3.1, -2.5, 0.6, (190, 120, 50)),                     # blue crate
                       _box(-2.5, -2.3, 0, -1.5, -1.7, 0.7, (70, 200, 220))]                   # yellow table

    def render(self, x, y, z, yaw, width, height, texts=()):
        import cv2
        f = 0.5 * width / math.tan(math.radians(HFOV_DEG / 2.0))
        cx, cy = width / 2.0, height / 2.0
        pitch = math.radians(PITCH_DOWN_DEG)
        c_yaw, s_yaw, c_p, s_p = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        eye = np.array([x, y, z + 0.04])
        basis = np.array([[s_yaw, -c_yaw, 0.0],                        # camera right
                          [-s_p * c_yaw, -s_p * s_yaw, -c_p],          # camera down
                          [c_p * c_yaw, c_p * s_yaw, -s_p]])           # camera forward (depth)
        image = np.empty((height, width, 3), np.uint8)
        image[:] = (28, 28, 28)

        def pixel(p):
            u, v = cx + f * p[0] / p[2], cy + f * p[1] / p[2]
            return int(max(-30000.0, min(30000.0, u))), int(max(-30000.0, min(30000.0, v)))

        def fill(points, colour):
            polygon = _clip_near(((points - eye) @ basis.T).tolist())
            if len(polygon) >= 3:
                cv2.fillPoly(image, [np.array([pixel(p) for p in polygon], np.int32)], colour)

        for points, colour in self.flat:
            fill(points, colour)
        for a, b in self.lines:
            segment = _clip_near(((np.array([a, b]) - eye) @ basis.T).tolist())
            if len(segment) >= 2:
                cv2.line(image, pixel(segment[0]), pixel(segment[1]), (86, 82, 79), 1, cv2.LINE_AA)
        for points, colour in self.paint:
            fill(points, colour)
        for centre, faces in sorted(self.boxes, key=lambda box: -float(np.sum((box[0] - eye) ** 2))):
            for points, normal, colour in faces:
                if float(np.dot(points[0] - eye, normal)) < 0.0:       # only faces turned towards the camera
                    fill(points, colour)
        for row, text in enumerate(texts):
            cv2.putText(image, text, (10, 22 + 20 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 3, cv2.LINE_AA)
            cv2.putText(image, text, (10, 22 + 20 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return image


# ---------------------------------------------------------------------------------------------- the drone
class FakeTello:
    def __init__(self, host='127.0.0.1', cmd_port=0, state_port=8890, video_port=11111, video=True,
                 size=(640, 480), fps=15, battery=87.0, takeoff_s=2.5, takeoff_height=0.8, land_speed=0.5,
                 max_speed=1.2, max_climb=0.7, max_turn_deg=100.0, failsafe_s=15.0, drain_per_min=4.0,
                 clock=time.monotonic):
        self.host, self.cmd_port = host, cmd_port
        self.state_port, self.video_port = state_port, video_port
        self.video, self.size, self.fps = bool(video), size, float(fps)
        self.battery = float(battery)
        self.takeoff_s, self.takeoff_height, self.land_speed = takeoff_s, takeoff_height, land_speed
        self.max_speed, self.max_climb, self.max_turn_deg = max_speed, max_climb, max_turn_deg
        self.failsafe_s, self.drain_per_min = failsafe_s, drain_per_min
        self.clock = clock
        # test hooks
        self.answer_rc = False             # the SDK document lists an answer to "rc"; real drones seem to send none
        self.drop_answers = set()          # commands whose answer gets "lost on the way"
        self.lose = collections.Counter()  # command -> how many of them get lost before they arrive
        self.mute_status = False           # no status datagrams (as if the port were blocked)
        self.refuse_takeoff = False
        # state
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0                     # radians, counter-clockwise seen from above; 0 = facing +x
        self.v = [0.0, 0.0, 0.0, 0.0]      # forward, right, up (m/s), turn (deg/s, clockwise)
        self.rc = (0, 0, 0, 0)
        self.phase = 'ground'              # ground | takeoff | air | landing
        self.sdk = self.streaming = False
        self.client = None                 # who said "command": (address, port)
        self.motor_s = 0.0
        self.landed_by_itself = False
        self.commands = collections.deque(maxlen=2000)
        self.frames_sent = 0
        self._land_asked = False
        self._phase_t = 0.0
        self._last_command = clock()
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._threads = []
        self._sock = self._out = None
        self._scene = self._encoder = None

    # ------------------------------------------------------------------ running
    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.cmd_port))
        self._sock.settimeout(0.2)
        self.cmd_port = self._sock.getsockname()[1]
        self._out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._out.bind((self.host, 0))
        try:
            self._out.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        except OSError:
            pass
        if self.video and importlib.util.find_spec('cv2') is None:     # no OpenCV: it still flies, without a camera
            self.video = False
        if self.video:
            self._scene, self._encoder = Scene(), IpcmEncoder(*self.size)
        for target, name in ((self._command_loop, 'fake-tello-commands'), (self._tick_loop, 'fake-tello-flight')):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def stop(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        for sock in (self._sock, self._out):
            try:
                if sock is not None:
                    sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------ commands
    def _reply(self, text, to=''):
        if to in self.drop_answers or self.client is None:
            return
        try:
            self._sock.sendto(text.encode('ascii'), self.client)
        except OSError:
            pass

    def _command_loop(self):
        while not self._stop.is_set():
            try:
                data, address = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.wait(0.05):
                    break
                continue
            text = data[:200].decode('ascii', 'replace').strip()
            with self._lock:
                if self.lose[text] > 0:
                    self.lose[text] -= 1
                    continue
                self._handle(text, address, self.clock())

    def _handle(self, text, address, now):
        self.commands.append(text)
        if text == 'command':
            self.sdk, self.client, self._last_command = True, address, now
            return self._reply('ok', 'command')
        if not self.sdk:                                   # the real drone ignores everything before "command"
            return None
        self._last_command = now
        if address != self.client:                         # another program on this computer: only "land" is taken
            if text == 'land' and self.phase == 'air':     # (the video worker, when the console has died)
                self.phase, self._land_asked = 'landing', False
            return None
        if text == 'takeoff':
            if self.phase != 'ground' or self.refuse_takeoff or self.battery < 10:
                return self._reply('error', 'takeoff')
            self.phase, self._phase_t, self.landed_by_itself = 'takeoff', now, False
        elif text == 'land':
            if self.phase != 'air':                        # on the ground, or busy taking off / landing
                return self._reply('error', 'land')
            self.phase, self._land_asked = 'landing', True
        elif text == 'emergency':
            self.phase, self.z, self.v, self.rc = 'ground', 0.0, [0.0] * 4, (0, 0, 0, 0)
            self._reply('ok', 'emergency')
        elif text in ('streamon', 'streamoff'):
            self.streaming = text == 'streamon'
            self._reply('ok', text)
        elif text.startswith('rc '):
            try:
                values = tuple(int(part) for part in text.split()[1:])
            except ValueError:
                values = ()
            if len(values) != 4 or any(abs(v) > 100 for v in values):
                return self._reply('error', 'rc')
            if self.phase == 'air':
                self.rc = values
            if self.answer_rc:
                self._reply('ok', 'rc')
        elif text.endswith('?'):
            answers = {'battery?': '%d' % self.battery, 'height?': '%ddm' % round(self.z * 10),
                       'tof?': '%dmm' % max(100, self.z * 1000), 'time?': '%ds' % self.motor_s, 'wifi?': '90',
                       'temp?': '60~62C', 'speed?': '100.0', 'baro?': '%.2f' % (100.0 + self.z),
                       'attitude?': 'pitch:0;roll:0;yaw:%d;' % self.yaw_deg()}
            self._reply(answers.get(text, 'error'), text)
        else:
            self._reply('unknown command: %s' % text[:30], text)
        return None

    # ------------------------------------------------------------------ flight
    def yaw_deg(self):
        """Heading as the status reports it: degrees, clockwise positive, -180..180, 0 = heading at start."""
        return int(round((math.degrees(-self.yaw) + 180.0) % 360.0 - 180.0))

    def step(self, dt, now):
        phase = self.phase
        if phase == 'takeoff':
            self.motor_s += dt
            k = min(1.0, (now - self._phase_t) / max(self.takeoff_s, 1e-3))
            self.z = self.takeoff_height * k
            if k >= 1.0:
                self.phase, self.rc, self._last_command = 'air', (0, 0, 0, 0), now
                self._reply('ok', 'takeoff')
        elif phase == 'air':
            self.motor_s += dt
            a, b, c, d = self.rc
            target = (b / 100.0 * self.max_speed, a / 100.0 * self.max_speed, c / 100.0 * self.max_climb,
                      d / 100.0 * self.max_turn_deg)
            blend = 1.0 - math.exp(-dt / 0.25)
            self.v = [v + (t - v) * blend for v, t in zip(self.v, target)]
            forward, right, up, turn = self.v
            self.yaw -= math.radians(turn) * dt            # clockwise = towards the right
            c_yaw, s_yaw = math.cos(self.yaw), math.sin(self.yaw)
            limit = ROOM - 0.35
            self.x = min(limit, max(-limit, self.x + (forward * c_yaw + right * s_yaw) * dt))
            self.y = min(limit, max(-limit, self.y + (forward * s_yaw - right * c_yaw) * dt))
            self.z = min(CEILING - 0.25, max(0.2, self.z + up * dt))
            if now - self._last_command > self.failsafe_s:              # the Tello safety feature
                self.phase, self._land_asked, self.landed_by_itself = 'landing', False, True
        elif phase == 'landing':
            self.motor_s += dt
            self.v = [0.0] * 4
            self.z -= self.land_speed * dt
            if self.z <= 0.0:
                self.z, self.phase, self.rc = 0.0, 'ground', (0, 0, 0, 0)
                if self._land_asked:
                    self._reply('ok', 'land')
        drain = self.drain_per_min * (1.0 if phase != 'ground' else 0.1)
        self.battery = max(0.0, self.battery - drain * dt / 60.0)

    def state_text(self):
        forward, right, up, _ = self.v
        return ('pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy:%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f;'
                'time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n'
                % (round(-6 * forward), round(6 * right), self.yaw_deg(), round(forward * 10), round(right * 10),
                   round(-up * 10), 60, 62, max(10, round(self.z * 100)), int(round(self.z * 10)) * 10,
                   int(self.battery), 100.0 + self.z, int(self.motor_s), 0.0, 0.0, -1000.0))

    def frame(self):
        """One camera picture (BGR) from where the drone is now."""
        with self._lock:
            pose = (self.x, self.y, self.z, self.yaw)
        stamp = time.time()
        clock = time.strftime('%H:%M:%S', time.localtime(stamp)) + '.%d' % (int(stamp * 10) % 10)
        return self._scene.render(*pose, self.size[0], self.size[1], ('SIMULATED DRONE', clock))

    def _tick_loop(self):
        last = self.clock()
        next_status = next_frame = last
        while not self._stop.wait(0.02):
            now = self.clock()
            with self._lock:
                self.step(min(now - last, 0.1), now)
                client, text = self.client, self.state_text()
                sending = self.sdk and client is not None
                streaming = sending and self.streaming and self.video
            last = now
            try:
                if sending and now >= next_status:
                    next_status = max(next_status + 0.1, now)
                    if not self.mute_status and self.state_port:
                        self._out.sendto(text.encode('ascii'), (client[0], self.state_port))
                if streaming and now >= next_frame and self.video_port:
                    next_frame = max(next_frame + 1.0 / self.fps, now)
                    data = self._encoder.encode(self.frame())
                    target = (client[0], self.video_port)
                    for start in range(0, len(data), CHUNK):
                        self._out.sendto(data[start:start + CHUNK], target)
                    self.frames_sent += 1
            except OSError:
                pass


# ---------------------------------------------------------------------------------------------- as a process
class SimProcess:
    """The simulated drone in its own process, so that drawing and sending its video never slows the console."""

    def __init__(self):
        self.cmd_port = 0
        self._proc = None

    def start(self, timeout=20.0):
        kwargs = dict(stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=str(PROJECT))
        if os.name == 'nt':
            kwargs['creationflags'] = (getattr(subprocess, 'BELOW_NORMAL_PRIORITY_CLASS', 0)
                                       | getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self._proc = subprocess.Popen([sys.executable, '-u', '-m', 'uav.sim'], **kwargs)
        line = []

        def read():
            try:
                line.append(self._proc.stdout.readline())
            except (OSError, ValueError):
                pass
        reader = threading.Thread(target=read, name='sim-ready', daemon=True)
        reader.start()
        reader.join(timeout)
        try:
            word, port = line[0].decode('ascii', 'replace').split()
            if word != 'ready':
                raise ValueError
            self.cmd_port = int(port)
        except (IndexError, ValueError):
            self.stop()
            raise RuntimeError('The simulated drone did not start.')
        return self

    def set_ports(self, state_port, video_port):
        """Tell the simulated drone where this computer listens for its status and its video."""
        try:
            self._proc.stdin.write(('ports %d %d\n' % (state_port, video_port)).encode('ascii'))
            self._proc.stdin.flush()
        except (OSError, ValueError):
            raise RuntimeError('The simulated drone is not running any more.')

    def alive(self):
        return self._proc is not None and self._proc.poll() is None

    def stop(self):
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.stdin.close()                         # the process ends when its input closes
        except (OSError, ValueError):
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:                                       # (a virtual environment's python.exe is a launcher with a child)
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/T', '/F', '/PID', str(proc.pid)], capture_output=True, timeout=10)
                proc.kill()
            except Exception:
                pass
        try:
            proc.stdout.close()
        except (OSError, ValueError):
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description='A simulated Tello drone on 127.0.0.1.')
    parser.add_argument('--cmd-port', type=int, default=0, help='0 = any free port (it is printed)')
    parser.add_argument('--state-port', type=int, default=0)
    parser.add_argument('--video-port', type=int, default=0)
    parser.add_argument('--fps', type=float, default=15.0)
    args = parser.parse_args(argv)
    drone = FakeTello(cmd_port=args.cmd_port, state_port=args.state_port, video_port=args.video_port,
                      fps=args.fps).start()
    print('ready %d' % drone.cmd_port, flush=True)
    try:
        for line in sys.stdin:                         # "ports <status> <video>"; the end of input = stop
            parts = line.split()
            if len(parts) == 3 and parts[0] == 'ports':
                try:
                    state_port, video_port = int(parts[1]), int(parts[2])
                except ValueError:
                    continue
                if 0 <= state_port < 65536 and 0 <= video_port < 65536:
                    drone.state_port, drone.video_port = state_port, video_port
    except (KeyboardInterrupt, OSError):
        pass
    drone.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
