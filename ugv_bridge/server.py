"""UGV-side bridge server: authenticated TCP <-> existing HiWonder motor controller.

Responsibilities (all enforced here, independent of the edge laptop):

* Mutual HMAC authentication; exactly one client at a time.
* Starts DISARMED. Motion commands are rejected until the authenticated client
  sends ``arm`` (issued only after explicit operator authorization on the edge).
* Disarms on: ``stop``/``disarm``, client disconnect, heartbeat loss, command
  starvation, malformed/unauthenticated frames, out-of-limit commands.
  A reconnecting client always starts disarmed.
* Velocity validation: finite numbers, absolute rejection threshold, clamping
  to configured limits, acceleration limiting, command expiry in the UGV clock.
* Independent watchdog/publisher thread: it is the ONLY writer to the motor
  topic. It publishes the latest valid command while fresh and zero otherwise,
  so the robot stops even if the edge laptop crashes or Wi-Fi drops.
* Local LiDAR stop zone and stale-odometry/stale-LiDAR interlocks.
* Telemetry: odometry, IMU, LiDAR summary/points, armed state, faults.
"""
from __future__ import annotations

import base64
import logging
import math
import select
import socket
import threading
import time

from . import protocol
from .protocol import ProtocolError, Session, finite_number
import math as _m
from .safety import clamp, obstacle_blocking, scan_points, slew, stopping_distance

log = logging.getLogger('ugv_bridge')

REQUIRED_MEASURED = ('half_length_m', 'half_width_m')

DEFAULTS = dict(
    bind_host='0.0.0.0', port=8765, key_file='mentorpi.key',
    max_vx=0.20, max_vy=0.0, max_wz=0.8,          # m/s, m/s, rad/s (vy=0: no lateral motion yet)
    reject_factor=1.5,                            # |cmd| > factor*max -> reject + disarm
    max_accel_v=0.4, max_accel_w=2.0,             # m/s^2, rad/s^2
    cmd_max_ttl=0.5,                              # longest expiry a command may request (s)
    cmd_hold=0.3,                                 # a single command is never applied longer than this (s)
    disarm_after_no_cmd=1.0,                      # armed but starved of commands -> disarm (s)
    heartbeat_timeout=1.0,                        # no frames at all -> drop client + disarm (s)
    publish_rate_hz=20.0, telemetry_rate_hz=10.0, zero_burst_s=0.5,
    odom_timeout=0.5, scan_timeout=0.5, require_lidar=True,
    half_length_m=None, half_width_m=None,        # MUST be measured; bridge refuses to start without them
    stop_distance_m=0.08, reaction_time_s=0.3, brake_decel=0.3,   # stop zone = margin + v*t + v^2/2a
    rotate_clearance_m=0.02,
    lidar_mount=[0.0, 0.0, 0.0],                  # x, y, yaw of LiDAR in base frame -- VERIFY
    lidar_min_valid_m=0.05, telemetry_scan_points=180, telemetry_scan_range_m=3.0,
    camera_max_rate_hz=10.0, camera_max_age_s=0.5,  # camera frames are sent only when the edge asks
)


class BridgeConfig(dict):
    def __init__(self, values=None):
        merged = dict(DEFAULTS)
        merged.update({k: v for k, v in (values or {}).items() if not k.startswith('_')})
        unknown = set(merged) - set(DEFAULTS) - {'backend', 'topics'}
        if unknown:
            raise ValueError('Unknown bridge config keys: %s' % sorted(unknown))
        super().__init__(merged)
        for key in REQUIRED_MEASURED:
            v = self[key]
            if v is None or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise ValueError('Config %s must be the MEASURED robot dimension in meters (got %r).' % (key, v))
        for key in ('max_vx', 'max_vy', 'max_wz', 'cmd_max_ttl', 'cmd_hold', 'stop_distance_m'):
            if not math.isfinite(self[key]) or self[key] < 0:
                raise ValueError('%s must be finite and nonnegative.' % key)
        if self['max_vx'] > 0.5 or self['max_vy'] > 0.5 or self['max_wz'] > 2.0:
            raise ValueError('Configured speed limits exceed the bridge hard ceiling (0.5 m/s, 2 rad/s).')
        if len(self['lidar_mount']) != 3:
            raise ValueError('lidar_mount must be [x, y, yaw].')

    def __getattr__(self, item):
        try:
            return self[item]
        except KeyError as exc:
            raise AttributeError(item) from exc


class BridgeServer:
    def __init__(self, config: BridgeConfig, backend, psk: bytes, clock=time.monotonic):
        self.cfg = config
        self.backend = backend
        self.psk = psk
        self.clock = clock
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self.armed = False
        self.target = (0.0, 0.0, 0.0)
        self.target_deadline = 0.0
        self.last_valid_cmd = 0.0
        self.output = (0.0, 0.0, 0.0)
        self.zero_until = 0.0
        self.safety_reason = None
        self.client = None             # (socket, session, address)
        self.last_rx = 0.0
        self.counters = {}
        self.events = []
        self._listener = None
        self.port = None
        self._threads = []

    # ---------------------------------------------------------- lifecycle
    def start(self):
        self.backend.start()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self.cfg.bind_host, self.cfg.port))
        self._listener.listen(2)
        self._listener.settimeout(0.2)
        self.port = self._listener.getsockname()[1]
        for target, name in ((self._accept_loop, 'accept'), (self._publish_loop, 'watchdog')):
            t = threading.Thread(target=target, name='bridge-' + name, daemon=True)
            t.start(); self._threads.append(t)
        self._event('bridge started on %s:%d (backend=%s), DISARMED' % (self.cfg.bind_host, self.port, self.backend.name))
        return self

    def shutdown(self):
        with self._lock:
            self._disarm('bridge shutdown')
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        try:
            self.backend.send_velocity(0.0, 0.0, 0.0)
        finally:
            self.backend.shutdown()
            if self._listener:
                self._listener.close()

    # ---------------------------------------------------------- bookkeeping
    def _event(self, text):
        log.warning(text)
        with self._lock:
            self.events.append((round(self.clock(), 3), text))
            del self.events[:-50]

    def _count(self, reason):
        with self._lock:
            self.counters[reason] = self.counters.get(reason, 0) + 1

    def _disarm(self, reason):
        with self._lock:
            if self.armed:
                self._event('DISARMED: %s' % reason)
            self.armed = False
            self.target = (0.0, 0.0, 0.0)
            self.target_deadline = 0.0
            self.output = (0.0, 0.0, 0.0)  # braking is immediate
            self.zero_until = self.clock() + self.cfg.zero_burst_s

    # ---------------------------------------------------------- network
    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                busy = self.client is not None
            if busy:
                self._event('rejected second connection from %s:%d (busy)' % addr)
                conn.close()
                continue
            t = threading.Thread(target=self._serve_client, args=(conn, addr), name='bridge-client', daemon=True)
            t.start()

    def _serve_client(self, conn, addr):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            conn.settimeout(2.0)
            frame, server_nonce = protocol.make_challenge(self.psk, self.clock())
            conn.sendall(frame)
            session_key, client_id = protocol.verify_auth(self.psk, protocol.read_frame(conn), server_nonce)
        except (ProtocolError, OSError, ConnectionError, ValueError) as exc:
            self._count('auth_failed')
            self._event('authentication failed from %s:%d: %s' % (addr[0], addr[1], exc))
            conn.close()
            return
        session = Session(session_key)
        send_lock = threading.Lock()
        with self._lock:
            if self.client is not None:
                conn.close(); return
            self.client = (conn, session, addr, send_lock)
            self.last_rx = self.clock()
            self._disarm('new connection')  # a (re)connected client always starts disarmed
        self._event('client %r connected from %s:%d (disarmed)' % (client_id, addr[0], addr[1]))
        try:
            self._send(conn, session, send_lock, {'type': 'auth_ok', 'server_time': self.clock(), 'limits': self._limits()})
            self._client_loop(conn, session, send_lock)
        except (ProtocolError, OSError, ConnectionError, ValueError) as exc:
            self._count('connection_error')
            self._event('client connection closed: %s' % exc)
        finally:
            with self._lock:
                self.client = None
                self._disarm('client disconnected')
            try:
                conn.close()
            except OSError:
                pass

    def _send(self, conn, session, send_lock, body):
        with send_lock:
            conn.sendall(session.seal(body))

    def _client_loop(self, conn, session, send_lock):
        reader = protocol.FrameReader()
        conn.settimeout(2.0)   # blocking sends with a timeout (large camera frames); reads are select()-gated
        period = 1.0 / self.cfg.telemetry_rate_hz
        next_telemetry = self.clock()
        self.camera_rate = 0.0     # frames/s requested by this client (0 = off)
        self.camera_info_sent = False
        next_frame = 0.0
        last_frame_time = None
        while not self._stop.is_set():
            with self._lock:
                if self.client is None or self.client[0] is not conn:
                    raise ConnectionError('dropped by watchdog')
            ready, _, _ = select.select([conn], [], [], 0.02)
            if ready:
                chunk = conn.recv(65536)
                if not chunk:
                    raise ConnectionError('peer closed')
                for raw in reader.feed(chunk):
                    try:
                        body = session.open(raw)
                    except ProtocolError as exc:
                        self._count('malformed')
                        raise ProtocolError('rejected frame: %s' % exc)
                    with self._lock:
                        self.last_rx = self.clock()
                    reply = self._handle(body)
                    if reply:
                        self._send(conn, session, send_lock, reply)
            now = self.clock()
            if now >= next_telemetry:
                next_telemetry = now + period
                self._send(conn, session, send_lock, self.telemetry())
            if self.camera_rate > 0 and now >= next_frame:
                next_frame = now + 1.0 / self.camera_rate
                if not self.camera_info_sent:
                    info = self.backend.camera_info()
                    if info is not None:
                        self._send(conn, session, send_lock, dict(info, type='camera_info'))
                        self.camera_info_sent = True
                image = self.backend.latest_image()
                if image is not None:
                    data, fmt, stamp = image
                    age = self.clock() - stamp
                    if stamp != last_frame_time and age <= self.cfg.camera_max_age_s and data:
                        last_frame_time = stamp
                        started = self.clock()
                        self._send(conn, session, send_lock, {
                            'type': 'image', 'format': str(fmt)[:40], 'age': age,
                            'data': base64.b64encode(bytes(data)).decode('ascii')})
                        if self.clock() - started > 0.5 / self.camera_rate:   # Wi-Fi too slow: back off
                            next_frame = self.clock() + 2.0 / self.camera_rate

    def _limits(self):
        c = self.cfg
        return dict(max_vx=c.max_vx, max_vy=c.max_vy, max_wz=c.max_wz, cmd_max_ttl=c.cmd_max_ttl,
                    cmd_hold=c.cmd_hold, disarm_after_no_cmd=c.disarm_after_no_cmd,
                    heartbeat_timeout=c.heartbeat_timeout)

    # ---------------------------------------------------------- command handling
    def _handle(self, body):
        kind = body['type']
        now = self.clock()
        if kind == 'camera':
            rate = finite_number(body, 'rate')
            self.camera_rate = max(0.0, min(rate, self.cfg.camera_max_rate_hz))
            self.camera_info_sent = False
            return {'type': 'ack', 'for': 'camera', 'ok': True, 'rate': self.camera_rate, 'ref': body['seq']}
        if kind == 'ping':
            return {'type': 'pong', 'client_time': finite_number(body, 'client_time'), 'server_time': now}
        if kind in ('stop', 'disarm'):
            self._disarm('%s requested by edge' % kind)
            return {'type': 'ack', 'for': kind, 'ok': True, 'ref': body['seq']}
        if kind == 'arm':
            ok, reason = self._can_arm(now)
            with self._lock:
                if ok:
                    self.armed = True
                    self.target = (0.0, 0.0, 0.0)
                    self.last_valid_cmd = now
                    self.safety_reason = None
                    self._event('ARMED by authenticated edge request')
            return {'type': 'ack', 'for': 'arm', 'ok': ok, 'reason': reason, 'ref': body['seq']}
        if kind == 'cmd_vel':
            ok, reason = self._accept_cmd(body, now)
            if not ok:
                self._count('cmd_' + reason.split(':')[0])
                return {'type': 'reject', 'for': 'cmd_vel', 'reason': reason, 'ref': body['seq']}
            return None  # accepted commands are acknowledged via telemetry, to save bandwidth
        self._count('unknown_type')
        return {'type': 'reject', 'for': kind[:32], 'reason': 'unknown message type', 'ref': body['seq']}

    def _can_arm(self, now):
        snap = self.backend.latest()
        if snap.odom_time is None or now - snap.odom_time > self.cfg.odom_timeout:
            return False, 'odometry missing or stale'
        if self.cfg.require_lidar and (snap.scan_time is None or now - snap.scan_time > self.cfg.scan_timeout):
            return False, 'LiDAR missing or stale'
        return True, ''

    def _accept_cmd(self, body, now):
        try:
            vx, vy, wz = (finite_number(body, k) for k in ('vx', 'vy', 'wz'))
            expires = finite_number(body, 'expires')
        except ProtocolError as exc:
            return False, 'malformed: %s' % exc
        c = self.cfg
        with self._lock:
            if not self.armed:
                return False, 'not_armed'
            if now > expires:
                return False, 'stale: expired %.3f s ago' % (now - expires)
            if expires - now > c.cmd_max_ttl + 0.05:
                return False, 'bad_expiry: too far in the future (clock sync?)'
            f = c.reject_factor
            if abs(vx) > f * c.max_vx + 1e-9 or abs(vy) > f * c.max_vy + 1e-9 or abs(wz) > f * c.max_wz + 1e-9:
                self._disarm('out-of-limit command (%.3f, %.3f, %.3f)' % (vx, vy, wz))
                return False, 'over_limit: disarmed'
            self.target = (clamp(vx, c.max_vx), clamp(vy, c.max_vy), clamp(wz, c.max_wz))
            self.target_deadline = min(expires, now + c.cmd_hold)
            self.last_valid_cmd = now
        return True, ''

    # ---------------------------------------------------------- watchdog / sole motor writer
    def _publish_loop(self):
        period = 1.0 / self.cfg.publish_rate_hz
        last = self.clock()
        while not self._stop.is_set():
            time.sleep(period)
            now = self.clock()
            try:
                self.step(now, now - last)
            except Exception as exc:  # never let the watchdog die silently
                log.exception('watchdog step failed')
                self._event('watchdog error: %s' % exc)
                try:
                    self.backend.send_velocity(0.0, 0.0, 0.0)
                except Exception:
                    pass
            last = now

    def step(self, now, dt):
        """One watchdog cycle. Public so tests can drive it with a fake clock."""
        c = self.cfg
        with self._lock:
            if self.client is not None and now - self.last_rx > c.heartbeat_timeout:
                self._event('heartbeat lost (%.2f s): dropping client' % (now - self.last_rx))
                try:
                    self.client[0].shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self.client = None
                self._disarm('heartbeat lost')
            if self.armed and now - self.last_valid_cmd > c.disarm_after_no_cmd:
                self._disarm('no valid command for %.2f s' % (now - self.last_valid_cmd))
            desired = self.target if (self.armed and now <= self.target_deadline) else (0.0, 0.0, 0.0)
            reason = None
            if any(desired):
                snap = self.backend.latest()
                reason = self._motion_interlock(snap, now, desired)
                if reason:
                    desired = (0.0, 0.0, 0.0)
            if reason != self.safety_reason and reason:
                self._event('safety stop: %s' % reason)
            self.safety_reason = reason
            ox, oy, ow = self.output
            dv, dw = c.max_accel_v * dt, c.max_accel_w * dt
            new = (slew(ox, desired[0], dv), slew(oy, desired[1], dv), slew(ow, desired[2], dw))
            if any(self.output) and not any(new):
                self.zero_until = now + c.zero_burst_s
            self.output = new
            publish = self.armed or any(new) or now < self.zero_until
        if publish:
            self.backend.send_velocity(*new)

    def _motion_interlock(self, snap, now, cmd):
        c = self.cfg
        if snap.odom_time is None or now - snap.odom_time > c.odom_timeout:
            return 'odometry stale'
        if snap.scan_time is None or now - snap.scan_time > c.scan_timeout:
            return 'LiDAR stale' if c.require_lidar else None
        points = scan_points(snap.scan, c.lidar_mount, max_range=1.0, min_valid=c.lidar_min_valid_m)
        zone = stopping_distance(_m.hypot(cmd[0], cmd[1]), c.stop_distance_m, c.reaction_time_s, c.brake_decel)
        return obstacle_blocking(points, cmd[0], cmd[1], cmd[2], c.half_length_m, c.half_width_m,
                                 zone, c.rotate_clearance_m)

    # ---------------------------------------------------------- telemetry
    def telemetry(self):
        snap = self.backend.latest()
        now = self.clock()
        c = self.cfg
        scan = None
        if snap.scan is not None:
            pts = scan_points(snap.scan, c.lidar_mount, max_range=c.telemetry_scan_range_m, min_valid=c.lidar_min_valid_m)
            step = max(1, int(math.ceil(len(pts) / float(c.telemetry_scan_points)))) if pts else 1
            scan = dict(age=now - snap.scan_time, points=[[round(x, 3), round(y, 3)] for x, y in pts[::step]])
        with self._lock:
            return {
                'type': 'telemetry', 'server_time': now, 'armed': self.armed,
                'output': list(self.output), 'target': list(self.target),
                'safety_stop': self.safety_reason,
                'odom': None if snap.odom is None else dict(snap.odom, age=now - snap.odom_time),
                'imu': None if snap.imu is None else dict(snap.imu, age=now - snap.imu_time),
                'scan': scan, 'counters': dict(self.counters),
                'events': [e for e in self.events[-5:]],
            }
