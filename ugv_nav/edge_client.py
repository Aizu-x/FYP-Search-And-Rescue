"""Edge-side client for the MentorPi bridge (standard library only).

* Background thread: connect, mutual authentication, receive telemetry, send
  pings every ``ping_period`` (heartbeat + clock synchronisation), reconnect
  with exponential back-off.
* Commands carry an expiry time in the UGV clock (offset estimated from the
  lowest-RTT ping), so frames delayed by a Wi-Fi stall are rejected as stale
  by the UGV instead of being executed late.
* The client NEVER arms automatically. After any reconnect the UGV is
  disarmed and ``arm()`` must be called again by the operator flow.
"""
from __future__ import annotations

import base64
import collections
import socket
import threading
import time

from ugv_bridge import protocol
from ugv_bridge.protocol import ProtocolError, Session

DISCONNECTED, CONNECTING, CONNECTED = 'DISCONNECTED', 'CONNECTING', 'CONNECTED'


class EdgeClient:
    def __init__(self, host, port, key: bytes, client_id='edge-laptop', ping_period=0.2,
                 link_timeout=0.6, connect_timeout=3.0, clock=time.monotonic):
        self.host, self.port, self.key, self.client_id = host, int(port), key, client_id
        self.ping_period, self.link_timeout, self.connect_timeout = ping_period, link_timeout, connect_timeout
        self.clock = clock
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sock = None
        self._session = None
        self.state = DISCONNECTED
        self.last_error = ''
        self.telemetry = None
        self.telemetry_time = None
        self.offset = None          # server_time - local_time
        self._sync = collections.deque(maxlen=20)
        self.rtt = None
        self.connections = 0
        self.limits = {}
        self._acks = {}
        self._ack_event = threading.Condition(self._lock)
        self.rejects = collections.deque(maxlen=20)
        self.camera_rate = 0.0          # requested frames/s (re-sent after every reconnect)
        self.image = None               # (encoded bytes, format, local receive time, age at UGV)
        self.image_count = 0
        self.camera_info = None

    # ---------------------------------------------------------- lifecycle
    def start(self):
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='edge-client', daemon=True)
        self._thread.start()
        return self

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        self._stop.set()
        self._drop('closed by operator')
        if self._thread:
            self._thread.join(timeout=3)

    # ---------------------------------------------------------- status
    def link_ok(self):
        with self._lock:
            return (self.state == CONNECTED and self.telemetry_time is not None
                    and self.clock() - self.telemetry_time <= self.link_timeout and self.offset is not None)

    def snapshot(self):
        with self._lock:
            age = None if self.telemetry_time is None else self.clock() - self.telemetry_time
            return dict(state=self.state, link_ok=self.state == CONNECTED and age is not None and age <= self.link_timeout
                        and self.offset is not None, telemetry=self.telemetry, telemetry_age=age, rtt=self.rtt,
                        error=self.last_error, connections=self.connections, rejects=list(self.rejects))

    def server_now(self):
        with self._lock:
            return None if self.offset is None else self.clock() + self.offset

    # ---------------------------------------------------------- commands
    def _send(self, body):
        with self._lock:
            sock, session = self._sock, self._session
        if sock is None or session is None:
            return None
        try:
            with self._send_lock:
                sock.sendall(session.seal(body))
                return session.send_seq
        except OSError as exc:
            self._drop('send failed: %s' % exc)
            return None

    def send_velocity(self, vx, vy, wz, ttl=0.3):
        """Returns False (and sends nothing) if the link or clock sync is not healthy."""
        if not self.link_ok():
            return False
        now = self.server_now()
        return self._send({'type': 'cmd_vel', 'vx': float(vx), 'vy': float(vy), 'wz': float(wz),
                           'expires': now + float(ttl)}) is not None

    def stop(self):
        """Best-effort STOP (also disarms). The UGV watchdog stops the robot regardless."""
        return self._send({'type': 'stop'}) is not None

    def disarm(self):
        return self._send({'type': 'disarm'}) is not None

    def request_camera(self, rate_hz):
        """Ask the UGV to stream camera frames (0 = stop). Remembered across reconnects."""
        self.camera_rate = max(0.0, float(rate_hz))
        return self._send({'type': 'camera', 'rate': self.camera_rate}) is not None

    def latest_image(self):
        with self._lock:
            return self.image

    def arm(self, timeout=1.5):
        """Request arming; returns (ok, reason). Call only after operator authorization."""
        if not self.link_ok():
            return False, 'link not healthy'
        ref = self._send({'type': 'arm'})
        if ref is None:
            return False, 'not connected'
        deadline = self.clock() + timeout
        with self._lock:
            while ref not in self._acks:
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return False, 'no acknowledgement from UGV'
                self._ack_event.wait(remaining)
            ack = self._acks.pop(ref)
        return bool(ack.get('ok')), ack.get('reason', '')

    # ---------------------------------------------------------- connection thread
    def _drop(self, reason):
        with self._lock:
            sock, self._sock, self._session = self._sock, None, None
            if self.state != DISCONNECTED:
                self.last_error = reason
            self.state = DISCONNECTED
            self.offset = None
            self._sync.clear()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.connect_timeout)
        frame, session_key = protocol.answer_challenge(self.key, protocol.read_frame(sock), self.client_id)
        sock.sendall(frame)
        session = Session(session_key)
        hello = session.open(protocol.read_frame(sock))
        if hello.get('type') != 'auth_ok':
            raise protocol.AuthError('UGV did not confirm authentication.')
        sock.settimeout(0.05)
        with self._lock:
            self._sock, self._session = sock, session
            self.state = CONNECTED
            self.connections += 1
            self.limits = hello.get('limits', {})
            self.last_error = ''
        return sock, session

    def _run(self):
        backoff = 0.5
        while not self._stop.is_set():
            with self._lock:
                self.state = CONNECTING
            try:
                sock, session = self._connect()
                backoff = 0.5
                self._receive_loop(sock, session)
            except (OSError, ConnectionError, ProtocolError, ValueError) as exc:
                self._drop('%s: %s' % (type(exc).__name__, exc))
            if not self._stop.is_set():
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 5.0)

    def _receive_loop(self, sock, session):
        reader = protocol.FrameReader()
        next_ping = 0.0
        if self.camera_rate > 0:   # restore the camera stream after a reconnect
            self._send({'type': 'camera', 'rate': self.camera_rate})
        while not self._stop.is_set():
            now = self.clock()
            if now >= next_ping:
                next_ping = now + self.ping_period
                if self._send({'type': 'ping', 'client_time': now}) is None:
                    raise ConnectionError('ping failed')
            with self._lock:
                if self._sock is not sock:
                    raise ConnectionError('socket replaced')
                last = self.telemetry_time
            if last is not None and now - last > max(3 * self.link_timeout, 2.0):
                raise ConnectionError('telemetry timeout')
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError('UGV closed the connection')
            for raw in reader.feed(chunk):
                self._handle(session.open(raw))

    def _handle(self, body):
        now = self.clock()
        kind = body['type']
        with self._lock:
            if kind == 'telemetry':
                self.telemetry, self.telemetry_time = body, now
            elif kind == 'pong':
                rtt = now - float(body['client_time'])
                if 0 <= rtt < 2.0:
                    self._sync.append((rtt, float(body['server_time']) - (float(body['client_time']) + rtt / 2)))
                    best = min(self._sync)
                    self.rtt, self.offset = rtt, best[1]
            elif kind == 'ack':
                self._acks[body.get('ref')] = body
                self._ack_event.notify_all()
            elif kind == 'reject':
                self.rejects.append((round(now, 3), body.get('for'), body.get('reason')))
            elif kind == 'image':
                try:
                    data = base64.b64decode(body.get('data', ''), validate=True)
                except (ValueError, TypeError):
                    return
                self.image = (data, str(body.get('format', '')), now, float(body.get('age', 0.0)))
                self.image_count += 1
            elif kind == 'camera_info':
                k, d = body.get('k'), body.get('d')
                if isinstance(k, list) and len(k) == 9 and all(isinstance(v, (int, float)) for v in k):
                    self.camera_info = dict(width=int(body.get('width', 0)), height=int(body.get('height', 0)),
                                            k=[float(v) for v in k],
                                            d=[float(v) for v in d] if isinstance(d, list) else [])
