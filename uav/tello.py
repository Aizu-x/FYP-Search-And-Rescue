"""The link to a Tello drone: plain text over UDP, as described in the official "Tello SDK 1.3" document.

    this computer  --"command", "takeoff", "rc 0 40 0 0", "land", "battery?" ...-->  drone 192.168.10.1 : 8889
                   <--"ok" / "error" / a value--------------------------------------  (answers return to the sender)
    port 8890      <--"pitch:0;roll:0;yaw:0;...;tof:10;h:0;bat:87;...\\r\\n"----------  status, several times a second
    port 11111     <--H.264 video, after "streamon"---------------------------------  received by uav/video.py

From that document: the port numbers, "command" first, the answers "ok" / "error", the status text and its units
(h and tof in cm, bat in %), "rc a b c d" with a = left/right, b = forward/backward, c = up/down, d = yaw, each
-100..100, and the drone's own safety feature: it lands by itself when it receives no command for 15 seconds.

NOT in that document, so still to be confirmed with the real drone (docs/UAV.md, "First flight"):
  * which sign of an "rc" value is which direction. Used here, as the community libraries do:
    a + = right, b + = forward, c + = up, d + = clockwise;
  * what the drone does with the last "rc" values when no more arrive (so the values are sent 20 times a second,
    and zeros are sent whenever the operator is not holding a key).

Safety rules of this link:
  * it never takes off by itself: only ``takeoff()``, and not with a battery below ``min_battery``;
  * ``land()`` is accepted in every state, goes out at once and is repeated until the drone confirms;
  * stick values go out only while flying, and fall back to zero (hover) when the operator's page has not
    reported its keys for ``key_timeout_s``;
  * nothing heard from the drone for ``lost_land_s`` while flying: "land" is sent;
  * datagrams that do not come from the drone's address are dropped, and the drone's address has to be one of a
    local network.
"""
from __future__ import annotations

import collections
import ipaddress
import math
import re
import socket
import threading
import time
from dataclasses import dataclass

DISCONNECTED, CONNECTING, READY, TAKING_OFF, FLYING, LANDING = (
    'disconnected', 'connecting', 'ready', 'taking_off', 'flying', 'landing')
AIRBORNE = (TAKING_OFF, FLYING, LANDING)
KEYS = ('w', 'a', 's', 'd', 'up', 'down', 'left', 'right')
ZERO = (0, 0, 0, 0)
_VALUE = re.compile(r'^(-?\d+(?:\.\d+)?)\s*([a-z%~]*)')
_UNIT_TO_CM = {'': 1.0, 'cm': 1.0, 'dm': 10.0, 'mm': 0.1, 'm': 100.0}


class TelloError(Exception):
    """Something the operator has to know; the message is shown as it is."""


@dataclass
class TelloConfig:
    host: str = '192.168.10.1'
    cmd_port: int = 8889              # the drone listens here and answers from here
    state_port: int = 8890            # on this computer: the drone sends its status here (0 = any free port)
    local_ip: str = ''                # this computer's address on the drone's network ('' = found by itself)
    connect_tries: int = 3
    reply_timeout_s: float = 1.5
    takeoff_timeout_s: float = 15.0
    land_timeout_s: float = 15.0
    land_resend_s: float = 2.0
    stale_reply_s: float = 1.0        # answers this soon after "land" belong to something sent before it
    rc_hz: float = 20.0
    key_timeout_s: float = 0.5        # no key report from the page for this long: hover
    min_battery: int = 20             # no take-off below this (%)
    status_stale_s: float = 1.5       # no status for this long: ask for battery and height instead
    ask_period_s: float = 1.0
    silent_s: float = 2.5             # nothing at all heard for this long: the sticks go to zero
    lost_land_s: float = 5.0          # ... and for this long while flying: "land" is sent


# ---------------------------------------------------------------------------------------------- small helpers
def check_host(host):
    """The drone's address as text, or ValueError. Only an IPv4 address of a local network (or this computer)."""
    try:
        address = ipaddress.IPv4Address(str(host).strip())
    except ValueError:
        raise ValueError('The drone address must look like 192.168.10.1.')
    local = address.is_private or address.is_loopback or address.is_link_local
    if not local or address.is_multicast or address.is_unspecified or int(address) == 0xFFFFFFFF:
        raise ValueError('The drone address must be an address of a local network, like 192.168.10.1.')
    return str(address)


def local_address_for(host, port=8889):
    """The address this computer would use to reach ``host`` ('' when there is no network route at all)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((host, port))            # UDP: nothing is sent, the system only picks the route
            return probe.getsockname()[0]
    except OSError:
        return ''


def parse_state(text):
    """'pitch:0;roll:0;...;h:80;bat:87;...' -> {'pitch': 0.0, ..., 'h': 80.0, 'bat': 87.0}.
    Fields that are not ``name:number`` are skipped (other firmware adds some, and anything may arrive)."""
    out = {}
    for part in str(text).strip().split(';')[:40]:
        name, colon, value = part.partition(':')
        name = name.strip()
        if not colon or not name.isalnum() or len(name) > 12:
            continue
        try:
            number = float(value)
        except ValueError:
            continue
        if math.isfinite(number) and abs(number) < 1e7:
            out[name] = number
    return out


def parse_value(text):
    """The answer to a read command ('87', '6dm', '766mm', '20s') -> (number, unit) or None."""
    match = _VALUE.match(str(text).strip().lower())
    return (float(match.group(1)), match.group(2)) if match else None


def sticks_from_keys(keys, percent):
    """Held keys -> the four "rc" values (a, b, c, d). Opposite keys cancel.
    W / S forward / back, A / D left / right, up / down arrows height, left / right arrows rotation."""
    held = set(keys)
    p = int(round(max(0.0, min(100.0, float(percent)))))

    def axis(plus, minus):
        return p * ((plus in held) - (minus in held))
    return (axis('d', 'a'), axis('w', 's'), axis('up', 'down'), axis('right', 'left'))


def same_network(a, b):
    """Rough check: same first three numbers (the drone's own Wi-Fi is 192.168.10.x)."""
    return a.split('.')[:3] == b.split('.')[:3]


# ---------------------------------------------------------------------------------------------- the link
class TelloLink:
    def __init__(self, config=None, clock=time.monotonic):
        self.cfg = config or TelloConfig()
        self.clock = clock
        self.host = check_host(self.cfg.host)
        self.state = DISCONNECTED
        self.local_ip = ''
        self.state_port = 0
        self.sticks = ZERO                 # what is being sent as "rc" while flying
        self.keys = ()
        self.status_port_error = ''
        self.rejected = 0                  # datagrams dropped because they did not come from the drone
        self.sent = collections.Counter()  # commands sent, by first word
        self.last_rc = ''
        self.send_error = ''
        self.status_packets = 0
        self.replies = 0
        self._lock = threading.RLock()
        self._values = {}                  # name -> (number, time it arrived)
        self._status_time = None
        self._heard = None                 # newest datagram of any kind from the drone
        self._events = collections.deque(maxlen=200)
        self._pending = None               # a command whose answer somebody is waiting for
        self._query = None                 # (read command, time sent) asked because the status is silent
        self._query_turn = 0
        self._asked_at = -1e9
        self._keys_time = -1e9
        self._since = clock()
        self._ready_since = None
        self._land_sent = -1e9
        self._land_errors = 0
        self._ground_since = None
        self._skip_ok_until = -1e9
        self._quiet_until = -1e9
        self._hovering_said = False
        self._silent_said = False
        self._sock = self._status_sock = None
        self._stop = threading.Event()
        self._threads = []

    # ------------------------------------------------------------------ connecting
    def open(self):
        self.bind()
        return self.handshake()

    def bind(self):
        """Open the sockets (nothing is sent yet). After this ``state_port`` is known."""
        cfg = self.cfg
        if self._sock is not None:
            raise TelloError('The drone link is already open.')
        local = cfg.local_ip or local_address_for(self.host, cfg.cmd_port)
        if not local:
            raise TelloError('This computer has no network connection that reaches %s. '
                             'Join the drone\'s Wi-Fi first.' % self.host)
        self.local_ip = local
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((local, 0))
            sock.settimeout(0.25)
            self._sock = sock
        except OSError as exc:
            raise TelloError('Could not open a network socket on %s (%s).' % (local, exc.strerror or exc))
        try:
            status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            status.bind((local, cfg.state_port))
            status.settimeout(0.25)
            self._status_sock, self.state_port = status, status.getsockname()[1]
        except OSError:
            self.status_port_error = ('Port %d of this computer is used by another program (another drone app, or '
                                      'a second console?), so battery and height are asked for instead.'
                                      % cfg.state_port)
        self._stop.clear()
        self._set_state(CONNECTING)
        loops = [(self._reply_loop, 'tello-replies'), (self._run_loop, 'tello-run')]
        if self._status_sock is not None:
            loops.append((self._status_loop, 'tello-status'))
        for target, name in loops:
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def handshake(self):
        """Say "command" until the drone answers "ok" (TelloError when it does not)."""
        cfg = self.cfg
        for _ in range(max(1, cfg.connect_tries)):
            answer = self.ask('command', cfg.reply_timeout_s)
            if answer is not None and answer.strip().lower() == 'ok':
                break
            if self._stop.is_set():
                raise TelloError('Connecting was cancelled.')
        else:
            text = 'The drone at %s did not answer.' % self.host
            if self.send_error:
                text += ' (%s)' % self.send_error
            if not same_network(self.local_ip, self.host):
                text += (' This computer\'s address on that route is %s, which is not on the drone\'s network: join '
                         'the drone\'s Wi-Fi (its name starts with TELLO-) and try again.' % self.local_ip)
            else:
                text += ' Is it switched on, with its light blinking, and close enough?'
            self.close(land=False)
            raise TelloError(text)
        with self._lock:
            self._set_state(READY)
            self._ready_since = self.clock()
        return self

    def close(self, land=True):
        """Close the link. ``land``: a drone that is in the air is told to land first (three times, to be sure)."""
        if land and self.state in AIRBORNE:
            for _ in range(3):
                self._send('land')
        self._stop.set()
        pending = self._pending
        if pending is not None:
            pending['done'].set()
        for sock in (self._sock, self._status_sock):
            try:
                if sock is not None:
                    sock.close()
            except OSError:
                pass
        me = threading.current_thread()
        for thread in self._threads:
            if thread is not me:
                thread.join(timeout=1.5)
        self._threads = []
        self._sock = self._status_sock = None
        with self._lock:
            self.state, self.sticks, self.keys = DISCONNECTED, ZERO, ()

    # ------------------------------------------------------------------ operator actions
    def takeoff(self):
        """Send "takeoff". TelloError (with the reason) when it must not be sent now."""
        now = self.clock()
        with self._lock:
            if self.state != READY:
                raise TelloError({FLYING: 'The drone is already flying.', TAKING_OFF: 'The drone is already taking off.',
                                  LANDING: 'The drone is landing: wait until it is down.'}.get(
                                      self.state, 'The drone is not connected.'))
            if now < self._quiet_until:
                raise TelloError('Just a moment: wait a second, then press Take off again.')
            battery = self._fresh('bat', now, 15.0)
            if battery is None:
                raise TelloError('No battery reading from the drone yet, so it is not allowed to take off. '
                                 'Wait a moment; if it stays like this, reconnect.')
            if battery < self.cfg.min_battery:
                raise TelloError('Battery %d %%: too low to take off (at least %d %% is needed).'
                                 % (battery, self.cfg.min_battery))
            self._set_state(TAKING_OFF)
            self.sticks, self.keys, self._query = ZERO, (), None
        if not self._send('takeoff'):
            with self._lock:
                self._set_state(READY)
            raise TelloError('The take-off command could not be sent (%s).' % (self.send_error or 'network error'))
        self._event('info', 'Take-off command sent.')

    def land(self, reason='LAND pressed'):
        """Land now. Never waits; allowed in every state (also "on the ground", in case the console is wrong)."""
        now = self.clock()
        with self._lock:
            state = self.state
            if state in (DISCONNECTED, CONNECTING):
                return False
            self.sticks, self.keys = ZERO, ()
            height = self._fresh('h', now)
            if state == LANDING:
                pass
            elif state == READY and height is not None and height < 20:
                self._quiet_until = now + self.cfg.stale_reply_s        # on the ground: "land" is sent anyway
            else:
                if state == TAKING_OFF:                # the "ok" of the take-off is still on its way: it must not
                    self._skip_ok_until = now + self.cfg.takeoff_timeout_s      # be taken for "landed"
                self._set_state(LANDING)
                self._land_errors, self._ground_since = 0, None
                self._quiet_until = now + self.cfg.stale_reply_s
                self._event('warn', 'Landing (%s).' % reason)
            self._land_sent = now
        self._send('land')
        return True

    def emergency(self):
        """Stop all motors at once (the drone falls). Sent three times."""
        with self._lock:
            if self.state in (DISCONNECTED, CONNECTING):
                return False
            self.sticks, self.keys = ZERO, ()
            self._set_state(READY)
            self._quiet_until = self.clock() + 2 * self.cfg.stale_reply_s
        for _ in range(3):
            self._send('emergency')
        self._event('error', 'EMERGENCY: motors cut.')
        return True

    def set_keys(self, keys, percent):
        """The keys the operator is holding right now (called often by the page; the newest report counts)."""
        keys = tuple(k for k in KEYS if k in set(keys))
        sticks = sticks_from_keys(keys, percent)
        with self._lock:
            self._keys_time = self.clock()
            self.keys = keys
            flying = self.state == FLYING
            lost = self._heard is not None and self._keys_time - self._heard > self.cfg.silent_s
            changed = flying and not lost and sticks != self.sticks
            self.sticks = sticks if flying else ZERO
            if keys:
                self._hovering_said = False
        if changed:
            self._send('rc %d %d %d %d' % sticks)          # at once, not at the next 50 ms tick

    def stream(self, on=True):
        """"streamon" / "streamoff". True when the drone answered "ok"."""
        answer = self.ask('streamon' if on else 'streamoff')
        return answer is not None and answer.strip().lower() == 'ok'

    def ask(self, text, timeout=None):
        """Send a command that is answered and wait for the answer. None: no answer in time, or it cannot be asked
        now (taking off, landing, another question is still open)."""
        timeout = self.cfg.reply_timeout_s if timeout is None else timeout
        wait = self._quiet_until - self.clock()
        if 0 < wait <= 5.0:
            self._stop.wait(wait)
        pending = dict(text=text, done=threading.Event(), answer=None)
        with self._lock:
            if self._sock is None or self.state in (DISCONNECTED, TAKING_OFF, LANDING) or self._pending is not None:
                return None
            self._pending, self._query = pending, None
        self._send(text)
        pending['done'].wait(timeout)
        with self._lock:
            if self._pending is pending:
                self._pending = None
            if pending['answer'] is None:              # a late answer must not be taken for the next command's
                self._quiet_until = max(self._quiet_until, self.clock() + self.cfg.stale_reply_s)
        return pending['answer']

    # ------------------------------------------------------------------ what the console shows
    def battery(self):
        return self._fresh('bat', self.clock(), 15.0)

    def height_cm(self):
        return self._fresh('h', self.clock())

    def snapshot(self):
        now = self.clock()
        cfg = self.cfg
        with self._lock:
            status_age = None if self._status_time is None else now - self._status_time
            heard = None if self._heard is None else now - self._heard
            connected = self.state not in (DISCONNECTED, CONNECTING)
            status_ok = status_age is not None and status_age <= cfg.status_stale_s
            waited = self._ready_since is not None and now - self._ready_since > 3.0
            return dict(
                state=self.state, since=now - self._since, host=self.host, local_ip=self.local_ip,
                battery=self._fresh('bat', now, 15.0), height_cm=self._fresh('h', now),
                tof_cm=self._fresh('tof', now), flight_s=self._fresh('time', now),
                temp_c=self._fresh('temph', now), pitch=self._fresh('pitch', now), roll=self._fresh('roll', now),
                yaw=self._fresh('yaw', now), status_age=status_age, heard_age=heard,
                source='status' if status_ok else 'asked' if connected and self._fresh('bat', now, 15.0) is not None
                else 'none',
                silent=connected and heard is not None and heard > cfg.silent_s,
                # the drone answers commands but its status never arrives: something blocks the incoming port
                status_blocked=connected and waited and not status_ok and not self.status_port_error
                and heard is not None and heard <= cfg.silent_s,
                sticks=list(self.sticks), keys=list(self.keys), status_port_error=self.status_port_error,
                rejected=self.rejected, send_error=self.send_error)

    def take_events(self):
        """(level, text) messages for the operator's log since the last call."""
        out = []
        while True:
            try:
                out.append(self._events.popleft())
            except IndexError:
                return out

    # ------------------------------------------------------------------ internals
    def _event(self, level, text):
        self._events.append((level, text))

    def _set_state(self, state):
        self.state = state
        self._since = self.clock()

    def _fresh(self, name, now, max_age=3.0):
        entry = self._values.get(name)
        return entry[0] if entry is not None and now - entry[1] <= max_age else None

    def _send(self, text):
        sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendto(text.encode('ascii'), (self.host, self.cfg.cmd_port))
        except (OSError, ValueError) as exc:
            self.send_error = getattr(exc, 'strerror', None) or str(exc)
            return False
        self.send_error = ''
        word = text.split(' ', 1)[0]
        self.sent[word] += 1
        if word == 'rc':
            self.last_rc = text
        return True

    def _landed(self, text, level='success'):
        self._set_state(READY)
        self.sticks, self.keys = ZERO, ()
        self._quiet_until = self.clock() + self.cfg.stale_reply_s
        self._event(level, text)

    def _reply_loop(self):
        sock, peer = self._sock, (self.host, self.cfg.cmd_port)
        while not self._stop.is_set():
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:                        # (Windows reports an unreachable port here) / socket closed
                if self._stop.wait(0.05):
                    break
                continue
            if address != peer:
                self.rejected += 1
                continue
            self._on_reply(data[:200].decode('ascii', 'replace').strip())

    def _on_reply(self, text):
        now = self.clock()
        low = text.lower()
        with self._lock:
            self._heard = now
            self.replies += 1
            pending = self._pending
            value = parse_value(low)
            if value is not None:                              # the answer to a read command
                if pending is not None and pending['text'].endswith('?') and pending['answer'] is None:
                    pending['answer'] = text
                    pending['done'].set()
                elif self._query is not None:
                    name, self._query = self._query[0], None
                    if name == 'battery?' and 0 <= value[0] <= 100:
                        self._values['bat'] = (value[0], now)
                    elif name == 'height?' and value[1] in _UNIT_TO_CM:
                        self._values['h'] = (value[0] * _UNIT_TO_CM[value[1]], now)
                return
            if now < self._quiet_until:                        # left over from something sent earlier
                return
            ok = low == 'ok'
            if ok and now < self._skip_ok_until:               # the answer to the "takeoff" that "land" overtook
                self._skip_ok_until = -1e9
                return
            if self.state == LANDING:
                if ok:
                    self._landed('The drone has landed.')
                else:
                    self._land_errors += 1
            elif self.state == TAKING_OFF:
                if ok:
                    self._set_state(FLYING)
                    self._keys_time, self._hovering_said = now, True
                    self._event('success', 'The drone is in the air. W A S D move it, the arrow keys change '
                                           'height and turn it. LAND = Esc.')
                else:
                    self._set_state(READY)
                    self._event('warn', 'The drone refused to take off (it answered "%s"). Usual reasons: battery '
                                        'too low, it is not standing level, or it is too hot.' % text[:40])
            elif pending is not None and pending['answer'] is None:
                pending['answer'] = text
                pending['done'].set()

    def _status_loop(self):
        sock = self._status_sock
        while not self._stop.is_set():
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.wait(0.05):
                    break
                continue
            if address[0] != self.host:
                self.rejected += 1
                continue
            values = parse_state(data[:1024].decode('ascii', 'replace'))
            if not values:
                continue
            now = self.clock()
            with self._lock:
                for name, number in values.items():
                    self._values[name] = (number, now)
                self._status_time = self._heard = now
                self.status_packets += 1

    def _run_loop(self):
        period = 1.0 / max(1.0, self.cfg.rc_hz)
        while not self._stop.wait(period):
            try:
                self._run_once()
            except Exception as exc:               # a bug here must never leave the sticks where they were
                self.sticks = ZERO
                self._event('error', 'Drone link error: %s' % exc)

    def _run_once(self):
        cfg, now = self.cfg, self.clock()
        send = []
        with self._lock:
            state = self.state
            silent = None if self._heard is None else now - self._heard
            height = self._fresh('h', now)
            if state == FLYING:
                if self.sticks != ZERO and now - self._keys_time > cfg.key_timeout_s:
                    self.sticks, self.keys = ZERO, ()
                    if not self._hovering_said:
                        self._hovering_said = True
                        self._event('warn', 'The console page stopped reporting its keys: hovering.')
                lost = silent is not None and silent > cfg.silent_s
                if lost and not self._silent_said:
                    self._silent_said = True
                    self._event('warn', 'Nothing heard from the drone for %.0f s: sticks to zero. Without contact it '
                                        'lands by itself (Tello safety feature, about 15 s).' % silent)
                elif not lost:
                    self._silent_said = False
                if silent is not None and silent > cfg.lost_land_s:
                    self._set_state(LANDING)
                    self._land_errors, self._ground_since, self._land_sent = 0, None, now
                    self._quiet_until = now + cfg.stale_reply_s
                    self.sticks, self.keys = ZERO, ()
                    self._event('warn', 'Landing (nothing heard from the drone for %.0f s).' % silent)
                    send.append('land')
                else:
                    send.append('rc %d %d %d %d' % (ZERO if lost else self.sticks))
            elif state == LANDING:
                elapsed = now - self._since
                if height is not None and height <= 0:
                    self._ground_since = self._ground_since or now
                else:
                    self._ground_since = None
                if self._ground_since is not None and elapsed > 2.5 and now - self._ground_since > 1.5:
                    self._landed('The drone is on the ground.')
                elif self._land_errors >= 2 and height is not None and height <= 0:     # "land" refused twice, height 0
                    self._landed('The drone is on the ground.')
                elif elapsed > cfg.land_timeout_s:
                    if height is not None and height >= 20:
                        self._set_state(FLYING)
                        self._keys_time = now
                        self._event('error', 'The drone did not land: it still reports %d cm. The keys work again; '
                                             'press LAND to try once more.' % height)
                    else:
                        self._landed('No confirmation of the landing arrived; the drone should be on the ground.', 'warn')
                elif now - self._land_sent >= cfg.land_resend_s:
                    self._land_sent = now
                    send.append('land')
            elif state == TAKING_OFF and now - self._since > cfg.takeoff_timeout_s:
                if height is not None and height >= 20:
                    self._set_state(FLYING)
                    self._keys_time = now
                    self._event('success', 'The drone is in the air (%d cm).' % height)
                else:
                    self._set_state(READY)
                    self._event('warn', 'The drone did not answer the take-off command. If it is in the air anyway, '
                                        'press LAND.')
            # the status port is silent (blocked by a firewall, or in use): ask for battery and height instead
            if state in (READY, FLYING) and self._pending is None and self._ready_since is not None:
                status_age = now - (self._status_time if self._status_time is not None else self._ready_since)
                if status_age > cfg.status_stale_s and now - self._asked_at >= cfg.ask_period_s:
                    name = ('battery?', 'height?')[self._query_turn % 2]
                    self._query_turn += 1
                    self._query, self._asked_at = (name, now), now
                    send.append(name)
        for text in send:
            self._send(text)
