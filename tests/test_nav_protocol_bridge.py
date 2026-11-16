"""Test B (software part): protocol, authentication, bridge validation, watchdog. No motors."""
import math
import struct
import time
import unittest

from ugv_bridge import protocol
from ugv_bridge.backends.base import SensorSnapshot
from ugv_bridge.backends.dryrun_backend import DryRunBackend
from ugv_bridge.protocol import AuthError, ProtocolError, Session
from ugv_bridge.safety import obstacle_blocking, slew
from ugv_bridge.server import BridgeConfig

from nav_helpers import KEY, RawClient, start_bridge, wait_for


class ProtocolTests(unittest.TestCase):
    def test_roundtrip_and_tamper(self):
        frame = protocol.encode_frame(KEY, {'type': 'x', 'v': 1})
        data = frame[4:]
        self.assertEqual(protocol.decode_frame(KEY, data)['v'], 1)
        tampered = bytearray(data); tampered[-2] ^= 1
        with self.assertRaises(AuthError):
            protocol.decode_frame(KEY, bytes(tampered))
        with self.assertRaises(AuthError):
            protocol.decode_frame(bytes(32), data)

    def test_rejects_nan_and_non_object(self):
        for payload in (b'{"type":"cmd_vel","vx":NaN}', b'[1,2]', b'{"no_type":1}', b'\xff\xfe'):
            import hmac, hashlib
            data = hmac.new(KEY, payload, hashlib.sha256).digest() + payload
            with self.assertRaises(ProtocolError):
                protocol.decode_frame(KEY, data)
        with self.assertRaises(ValueError):
            protocol.encode_frame(KEY, {'type': 'x', 'v': float('nan')})

    def test_frame_reader_split_and_oversize(self):
        reader = protocol.FrameReader()
        frame = protocol.encode_frame(KEY, {'type': 'a'}) + protocol.encode_frame(KEY, {'type': 'b'})
        out = []
        for i in range(len(frame)):
            out += reader.feed(frame[i:i + 1])
        self.assertEqual([protocol.decode_frame(KEY, f)['type'] for f in out], ['a', 'b'])
        with self.assertRaises(ProtocolError):
            protocol.FrameReader().feed(struct.pack('>I', protocol.MAX_FRAME + 1))

    def test_replay_and_order(self):
        tx, rx = Session(KEY), Session(KEY)
        a, b = tx.seal({'type': 'x'})[4:], tx.seal({'type': 'y'})[4:]
        self.assertEqual(rx.open(a)['type'], 'x')
        self.assertEqual(rx.open(b)['type'], 'y')
        with self.assertRaises(ProtocolError):
            rx.open(a)

    def test_handshake_and_wrong_key(self):
        challenge, nonce = protocol.make_challenge(KEY, 1.0)
        auth, client_key = protocol.answer_challenge(KEY, challenge[4:], 'edge')
        server_key, cid = protocol.verify_auth(KEY, auth[4:], nonce)
        self.assertEqual(client_key, server_key); self.assertEqual(cid, 'edge')
        with self.assertRaises(AuthError):
            protocol.answer_challenge(bytes(32), challenge[4:], 'edge')
        auth2, _ = protocol.answer_challenge(KEY, challenge[4:], 'edge')
        _, other_nonce = protocol.make_challenge(KEY, 1.0)
        with self.assertRaises(AuthError):
            protocol.verify_auth(KEY, auth2[4:], other_nonce)  # replayed auth for a different challenge

    def test_key_file_encodings(self):
        import os, tempfile
        key_hex = protocol.generate_key_hex()
        with tempfile.TemporaryDirectory() as d:
            for name, data in [('ascii', key_hex.encode()), ('utf8bom', b'\xef\xbb\xbf' + key_hex.encode() + b'\r\n'),
                               ('utf16', (key_hex + '\r\n').encode('utf-16'))]:  # PowerShell 5 '>' output
                p = os.path.join(d, name); open(p, 'wb').write(data)
                self.assertEqual(protocol.load_key(p), bytes.fromhex(key_hex))
            p = os.path.join(d, 'new.key'); protocol.write_key_file(p)
            self.assertEqual(len(protocol.load_key(p)), 32)
            with self.assertRaises(FileExistsError):
                protocol.write_key_file(p)  # never overwrites an existing key

    def test_key_parsing(self):
        with self.assertRaises(AuthError):
            protocol.parse_key('abcd')
        with self.assertRaises(AuthError):
            protocol.parse_key('zz' * 32)
        self.assertEqual(len(protocol.parse_key(protocol.generate_key_hex())), 32)


class SafetyGeometryTests(unittest.TestCase):
    def test_forward_corridor(self):
        args = (0.10, 0.08, 0.20, 0.03)
        self.assertIsNotNone(obstacle_blocking([(0.25, 0.0)], 0.1, 0, 0, *args))
        self.assertIsNone(obstacle_blocking([(0.35, 0.0)], 0.1, 0, 0, *args))
        self.assertIsNone(obstacle_blocking([(0.25, 0.2)], 0.1, 0, 0, *args))   # beside the corridor
        self.assertIsNone(obstacle_blocking([(-0.2, 0.0)], 0.1, 0, 0, *args))   # behind, moving forward
        self.assertIsNotNone(obstacle_blocking([(-0.2, 0.0)], -0.1, 0, 0, *args))
        self.assertIsNotNone(obstacle_blocking([(0.0, 0.14)], 0, 0, 0.5, *args))  # turning radius
        self.assertIsNone(obstacle_blocking([(0.05, 0.0)], 0.1, 0, 0, *args))   # self-return inside body

    def test_slew(self):
        self.assertAlmostEqual(slew(0.0, 0.2, 0.05), 0.05)
        self.assertEqual(slew(0.2, 0.0, 0.05), 0.0)       # braking immediate
        self.assertEqual(slew(0.1, -0.1, 0.05), 0.0)      # pass through zero


class BridgeConfigTests(unittest.TestCase):
    def test_requires_measured_dimensions_and_ceilings(self):
        with self.assertRaises(ValueError):
            BridgeConfig({})
        with self.assertRaises(ValueError):
            BridgeConfig({'half_length_m': 0.1, 'half_width_m': 0.1, 'max_vx': 2.0})
        with self.assertRaises(ValueError):
            BridgeConfig({'half_length_m': 0.1, 'half_width_m': 0.1, 'typo_key': 1})


class StaleOdomBackend(DryRunBackend):
    def __init__(self):
        super().__init__()
        self.odom_age = 0.0
        self.obstacle_ahead = None

    def latest(self):
        snap = super().latest()
        snap.odom_time -= self.odom_age
        if self.obstacle_ahead is not None:
            ranges = snap.scan['ranges']
            ranges[len(ranges) // 2] = self.obstacle_ahead  # angle 0 = straight ahead
        return snap


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.backend = StaleOdomBackend()
        self.server, _ = start_bridge(self.backend)

    def tearDown(self):
        self.server.shutdown()

    def nonzero_commands(self):
        return [c for c in self.backend.commands if any(c[1:])]

    def arm(self, client):
        client.send({'type': 'arm'})
        ack = client.recv_until('ack')
        self.assertTrue(ack['ok'], ack)

    def cmd(self, client, vx=0.1, vy=0.0, wz=0.0, ttl=0.3, expires=None):
        client.send({'type': 'cmd_vel', 'vx': vx, 'vy': vy, 'wz': wz,
                     'expires': expires if expires is not None else client.server_now() + ttl})

    def test_starts_disarmed_and_rejects_motion(self):
        c = RawClient(self.server.port)
        tel = c.recv_until('telemetry')
        self.assertFalse(tel['armed'])
        self.cmd(c)
        self.assertEqual(c.recv_until('reject')['reason'], 'not_armed')
        time.sleep(0.2)
        self.assertEqual(self.nonzero_commands(), [])
        c.close()

    def test_wrong_key_rejected(self):
        with self.assertRaises((AuthError, OSError, ConnectionError, ProtocolError)):
            RawClient(self.server.port, key=bytes(32))
        self.assertTrue(wait_for(lambda: self.server.counters.get('auth_failed', 0) >= 1))

    def test_arm_command_clamp_and_watchdog(self):
        c = RawClient(self.server.port)
        self.arm(c)
        for _ in range(10):
            self.cmd(c, vx=0.25)  # above max_vx 0.2 but below reject factor -> clamped
            time.sleep(0.03)
        self.assertTrue(wait_for(lambda: self.server.output[0] > 0))
        self.assertLessEqual(max(x[1] for x in self.backend.commands), 0.2 + 1e-9)
        # Stop sending: watchdog must zero the output within cmd_hold (+ one period), then disarm.
        t0 = time.monotonic()
        self.assertTrue(wait_for(lambda: self.server.output == (0.0, 0.0, 0.0), timeout=1.0))
        self.assertLess(time.monotonic() - t0, 0.45)
        self.assertEqual(self.backend.last_command()[1:], (0.0, 0.0, 0.0))
        self.assertTrue(wait_for(lambda: not self.server.armed, timeout=1.5))
        c.close()

    def test_stop_disarms(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.cmd(c, vx=0.1)
        self.assertTrue(wait_for(lambda: self.server.output[0] > 0))
        c.send({'type': 'stop'})
        c.recv_until('ack')
        self.assertFalse(self.server.armed)
        self.assertEqual(self.server.output, (0.0, 0.0, 0.0))
        self.cmd(c)
        self.assertEqual(c.recv_until('reject')['reason'], 'not_armed')
        c.close()

    def test_stale_and_future_and_malformed_commands(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.cmd(c, expires=c.server_now() - 0.2)
        self.assertTrue(c.recv_until('reject')['reason'].startswith('stale'))
        self.cmd(c, expires=c.server_now() + 30)
        self.assertTrue(c.recv_until('reject')['reason'].startswith('bad_expiry'))
        c.send({'type': 'cmd_vel', 'vx': 'fast', 'vy': 0, 'wz': 0, 'expires': c.server_now() + 0.2})
        self.assertTrue(c.recv_until('reject')['reason'].startswith('malformed'))
        c.send({'type': 'cmd_vel', 'vx': True, 'vy': 0, 'wz': 0, 'expires': c.server_now() + 0.2})
        self.assertTrue(c.recv_until('reject')['reason'].startswith('malformed'))
        c.send({'type': 'self_destruct'})
        self.assertEqual(c.recv_until('reject')['reason'], 'unknown message type')
        self.assertEqual(self.nonzero_commands(), [])
        c.close()

    def test_over_limit_disarms(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.cmd(c, vx=5.0)
        self.assertTrue(c.recv_until('reject')['reason'].startswith('over_limit'))
        self.assertFalse(self.server.armed)
        self.assertEqual(self.nonzero_commands(), [])
        c.close()

    def test_lateral_motion_rejected_when_max_vy_zero(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.cmd(c, vx=0.0, vy=0.05)
        self.assertTrue(c.recv_until('reject')['reason'].startswith('over_limit'))
        self.assertTrue(all(cmd[2] == 0.0 for cmd in self.backend.commands))
        c.close()

    def test_bad_mac_closes_and_disarms(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.assertTrue(self.server.armed)
        frame = bytearray(c.session.seal({'type': 'ping', 'client_time': 1.0}))
        frame[-3] ^= 0xFF
        c.sock.sendall(bytes(frame))
        self.assertTrue(wait_for(lambda: self.server.client is None))
        self.assertFalse(self.server.armed)
        self.assertGreaterEqual(self.server.counters.get('malformed', 0), 1)
        c.close()

    def test_disconnect_disarms_and_reconnect_stays_disarmed(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.cmd(c, vx=0.1)
        self.assertTrue(wait_for(lambda: self.server.output[0] > 0))
        c.close()
        self.assertTrue(wait_for(lambda: not self.server.armed and self.server.output == (0.0, 0.0, 0.0)))
        c2 = RawClient(self.server.port)
        self.assertFalse(c2.recv_until('telemetry')['armed'])
        self.cmd(c2)
        self.assertEqual(c2.recv_until('reject')['reason'], 'not_armed')
        c2.close()

    def test_heartbeat_loss_drops_client(self):
        c = RawClient(self.server.port)
        self.arm(c)
        # Send nothing at all for longer than heartbeat_timeout (0.6 s).
        self.assertTrue(wait_for(lambda: self.server.client is None, timeout=2.0))
        self.assertFalse(self.server.armed)
        c.close()

    def test_second_client_rejected(self):
        c = RawClient(self.server.port)
        c.recv_until('telemetry')
        with self.assertRaises((OSError, ConnectionError, ProtocolError)):
            RawClient(self.server.port)
        c.close()

    def test_arm_refused_with_stale_odometry(self):
        self.backend.odom_age = 2.0
        c = RawClient(self.server.port)
        c.send({'type': 'arm'})
        ack = c.recv_until('ack')
        self.assertFalse(ack['ok']); self.assertIn('odometry', ack['reason'])
        c.close()

    def test_odometry_goes_stale_while_moving(self):
        c = RawClient(self.server.port)
        self.arm(c)
        self.backend.odom_age = 2.0
        for _ in range(5):
            self.cmd(c, vx=0.1); time.sleep(0.03)
        self.assertEqual(self.nonzero_commands(), [])
        self.assertEqual(self.server.safety_reason, 'odometry stale')
        c.close()

    def test_lidar_obstacle_blocks_forward_but_not_reverse(self):
        self.backend.obstacle_ahead = 0.20  # 0.10 m beyond the 0.10 m half-length, inside 0.20 m stop zone
        c = RawClient(self.server.port)
        self.arm(c)
        for _ in range(5):
            self.cmd(c, vx=0.1); time.sleep(0.03)
        self.assertEqual(self.nonzero_commands(), [])
        self.assertIn('obstacle', self.server.safety_reason)
        reasons = []
        for _ in range(6):
            self.cmd(c, vx=0.1)
            reasons.append(c.recv_until('telemetry')['safety_stop'] or '')
        self.assertTrue(any('obstacle' in r for r in reasons), reasons)
        for _ in range(5):
            self.cmd(c, vx=-0.1); time.sleep(0.03)
        self.assertTrue(wait_for(lambda: self.server.output[0] < 0))
        c.close()


class FakeClockBridgeTests(unittest.TestCase):
    """Deterministic watchdog tests via BridgeServer.step() with a fake clock (no sockets)."""

    def test_slew_hold_and_starvation(self):
        from ugv_bridge.server import BridgeServer
        from nav_helpers import bridge_config
        now = [100.0]
        backend = DryRunBackend()
        backend.latest = lambda: SensorSnapshot(dict(x=0, y=0, yaw=0, vx=0, vy=0, wz=0), now[0], None, None,
                                                dict(angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                                                     ranges=[3.0] * 360, range_min=0.05, range_max=8), now[0])
        s = BridgeServer(bridge_config(max_accel_v=0.4), backend, KEY, clock=lambda: now[0])
        s.armed = True; s.last_valid_cmd = now[0]
        s.target = (0.2, 0.0, 0.0); s.target_deadline = now[0] + 0.3
        outputs = []
        for _ in range(10):
            now[0] += 0.05; s.step(now[0], 0.05); outputs.append(s.output[0])
        self.assertAlmostEqual(outputs[0], 0.02)          # 0.4 m/s^2 * 0.05 s
        self.assertTrue(all(b >= a for a, b in zip(outputs, outputs[1:5])))
        self.assertEqual(outputs[-1], 0.0)                # command held only until its deadline
        now[0] += 1.0; s.step(now[0], 0.05)
        self.assertFalse(s.armed)                         # starved of commands -> disarmed


if __name__ == '__main__':
    unittest.main()
