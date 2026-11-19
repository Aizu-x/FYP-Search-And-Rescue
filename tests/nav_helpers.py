"""Shared helpers for the navigation tests (no hardware)."""
import socket
import time

from ugv_bridge import protocol
from ugv_bridge.backends.dryrun_backend import DryRunBackend
from ugv_bridge.protocol import Session
from ugv_bridge.server import BridgeConfig, BridgeServer

KEY = bytes(range(32))


def bridge_config(**overrides):
    values = dict(bind_host='127.0.0.1', port=0, half_length_m=0.10, half_width_m=0.08,
                  max_vx=0.2, max_vy=0.0, max_wz=0.8, cmd_hold=0.3, disarm_after_no_cmd=0.6,
                  heartbeat_timeout=0.6, publish_rate_hz=50.0, telemetry_rate_hz=20.0)
    values.update(overrides)
    return BridgeConfig(values)


def start_bridge(backend=None, **overrides):
    backend = backend or DryRunBackend()
    return BridgeServer(bridge_config(**overrides), backend, KEY).start(), backend


class RawClient:
    """Minimal protocol client for adversarial tests."""

    def __init__(self, port, key=KEY):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=2)
        frame, session_key = protocol.answer_challenge(key, protocol.read_frame(self.sock), 'test')
        self.sock.sendall(frame)
        self.session = Session(session_key)
        hello = self.session.open(protocol.read_frame(self.sock))
        assert hello['type'] == 'auth_ok'
        self.server_offset = hello['server_time'] - time.monotonic()

    def send(self, body):
        self.sock.sendall(self.session.seal(body))

    def recv_until(self, kind, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            body = self.session.open(protocol.read_frame(self.sock))
            if body['type'] == kind:
                return body
        raise TimeoutError(kind)

    def server_now(self):
        return time.monotonic() + self.server_offset

    def close(self):
        self.sock.close()


def wait_for(predicate, timeout=3.0, step=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


SECOND_ADDRESS = '127.0.0.2'
NO_SECOND_ADDRESS = ('this computer does not deliver datagrams sent from a second loopback address (%s), '
                     'so a datagram "from another address" cannot be made here' % SECOND_ADDRESS)


def second_address_socket(test):
    """A UDP socket on another address of this computer, for a test that forges the sender of a datagram.

    None when this computer cannot do that: some systems have no second loopback address, and Windows has been
    seen to stop delivering datagrams that are sent from one. The test then has nothing to send and should say
    so (``skipTest(NO_SECOND_ADDRESS)``) instead of failing. The socket is closed when the test ends.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    test.addCleanup(sock.close)
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((SECOND_ADDRESS, 0))
        probe.bind(('127.0.0.1', 0))
        probe.settimeout(1.0)
        sock.sendto(b'probe', probe.getsockname())
        sender = probe.recvfrom(16)[1]
        return sock if sender[0] == SECOND_ADDRESS else None
    except OSError:                         # no such address, sending refused, or nothing arrived in time
        return None
    finally:
        probe.close()
