"""Authenticated, length-prefixed message protocol shared by the edge laptop and the UGV.

Standard library only and Python 3.8 compatible, so the same file runs on the
Raspberry Pi (which may have an older Python inside the HiWonder image) and on
the edge laptop.

Wire format (TCP)::

    frame   = length(4 bytes, big-endian, unsigned) || mac(32 bytes) || payload
    payload = UTF-8 JSON object, must contain a string "type"
    mac     = HMAC-SHA256(key, payload)

Handshake (mutual authentication, fresh session key per connection)::

    UGV  -> edge : challenge {server_nonce, server_time, proto}   MAC'd with pre-shared key
    edge -> UGV  : auth      {server_nonce, client_nonce, client} MAC'd with pre-shared key
    both derive  : session_key = HMAC(psk, "mentorpi-session" || server_nonce || client_nonce)
    UGV  -> edge : auth_ok   {...}                                MAC'd with session key

After the handshake every frame is MAC'd with the session key and carries a
strictly increasing per-direction ``seq``. Frames with a bad MAC, malformed
JSON, NaN/Infinity, or a replayed/out-of-order seq are rejected and the
connection is closed (the UGV stops and disarms).

Messages are authenticated but NOT encrypted. Telemetry is readable by anyone
on the Wi-Fi network; commands cannot be forged or replayed without the key.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import socket
import struct

PROTOCOL_VERSION = 1
MAC_LEN = 32
MAX_FRAME = 1024 * 1024        # 1 MiB: room for one base64 JPEG camera frame
MIN_KEY_BYTES = 32
_HEADER = struct.Struct('>I')


class ProtocolError(Exception):
    """Malformed, oversized, replayed or otherwise invalid frame."""


class AuthError(ProtocolError):
    """MAC verification or handshake failure."""


# ---------------------------------------------------------------- keys

def generate_key_hex() -> str:
    return os.urandom(MIN_KEY_BYTES).hex()


def parse_key(text: str) -> bytes:
    try:
        key = bytes.fromhex(text.strip())
    except ValueError as exc:
        raise AuthError('Key file must contain hexadecimal text.') from exc
    if len(key) < MIN_KEY_BYTES:
        raise AuthError('Key must be at least %d bytes (%d hex characters).' % (MIN_KEY_BYTES, 2 * MIN_KEY_BYTES))
    return key


def load_key(path) -> bytes:
    """Read a hex key file. Accepts ASCII/UTF-8 and the UTF-16 files that Windows
    PowerShell 5 creates with ``>`` redirection."""
    with open(path, 'rb') as handle:
        raw = handle.read()
    if raw.startswith((b'\xff\xfe', b'\xfe\xff')):
        text = raw.decode('utf-16')
    else:
        try:
            text = raw.decode('utf-8-sig')
        except UnicodeDecodeError as exc:
            raise AuthError('Key file is not text.') from exc
    return parse_key(text)


def write_key_file(path) -> None:
    """Create a new random key file (ASCII hex), readable only by the owner on POSIX."""
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(path, flags, 0o600)
    with os.fdopen(fd, 'w', encoding='ascii') as handle:
        handle.write(generate_key_hex() + '\n')


# ---------------------------------------------------------------- framing

def _reject_constant(name):
    raise ProtocolError('Non-finite number in message: %s' % name)


def encode_frame(key: bytes, body: dict) -> bytes:
    payload = json.dumps(body, separators=(',', ':'), allow_nan=False).encode('utf-8')
    data = hmac.new(key, payload, hashlib.sha256).digest() + payload
    if len(data) > MAX_FRAME:
        raise ProtocolError('Frame too large.')
    return _HEADER.pack(len(data)) + data


def decode_frame(key: bytes, data: bytes) -> dict:
    if len(data) <= MAC_LEN:
        raise ProtocolError('Frame too short.')
    tag, payload = data[:MAC_LEN], data[MAC_LEN:]
    if not hmac.compare_digest(tag, hmac.new(key, payload, hashlib.sha256).digest()):
        raise AuthError('MAC verification failed.')
    try:
        body = json.loads(payload.decode('utf-8'), parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError) as exc:
        raise ProtocolError('Malformed JSON payload.') from exc
    if not isinstance(body, dict) or not isinstance(body.get('type'), str):
        raise ProtocolError('Payload must be an object with a string "type".')
    return body


class FrameReader:
    """Incremental stream reassembly; rejects oversized length prefixes immediately."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list:
        self._buffer.extend(chunk)
        frames = []
        while len(self._buffer) >= _HEADER.size:
            (length,) = _HEADER.unpack_from(self._buffer, 0)
            if length > MAX_FRAME or length <= MAC_LEN:
                raise ProtocolError('Invalid frame length %d.' % length)
            if len(self._buffer) < _HEADER.size + length:
                break
            frames.append(bytes(self._buffer[_HEADER.size:_HEADER.size + length]))
            del self._buffer[:_HEADER.size + length]
        return frames


def read_frame(sock: socket.socket) -> bytes:
    """Blocking read of exactly one frame (used during the handshake only)."""
    header = _recv_exact(sock, _HEADER.size)
    (length,) = _HEADER.unpack(header)
    if length > MAX_FRAME or length <= MAC_LEN:
        raise ProtocolError('Invalid frame length %d.' % length)
    return _recv_exact(sock, length)


def _recv_exact(sock, count):
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError('Peer closed the connection.')
        data.extend(chunk)
    return bytes(data)


# ---------------------------------------------------------------- session

class Session:
    """Session-key MAC plus strictly increasing per-direction sequence numbers."""

    def __init__(self, key: bytes):
        self.key = key
        self.send_seq = 0
        self.recv_seq = 0

    def seal(self, body: dict) -> bytes:
        self.send_seq += 1
        body = dict(body)
        body['seq'] = self.send_seq
        return encode_frame(self.key, body)

    def open(self, data: bytes) -> dict:
        body = decode_frame(self.key, data)
        seq = body.get('seq')
        if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.recv_seq:
            raise ProtocolError('Replayed or out-of-order sequence number.')
        self.recv_seq = seq
        return body


def derive_session_key(psk: bytes, server_nonce: str, client_nonce: str) -> bytes:
    material = b'mentorpi-session' + bytes.fromhex(server_nonce) + bytes.fromhex(client_nonce)
    return hmac.new(psk, material, hashlib.sha256).digest()


def _nonce() -> str:
    return os.urandom(16).hex()


def _hex_nonce(value) -> bool:
    if not isinstance(value, str) or len(value) != 32:
        return False
    try:
        bytes.fromhex(value)
    except ValueError:
        return False
    return True


def make_challenge(psk: bytes, server_time: float):
    nonce = _nonce()
    return encode_frame(psk, {'type': 'challenge', 'proto': PROTOCOL_VERSION,
                              'server_nonce': nonce, 'server_time': server_time}), nonce


def answer_challenge(psk: bytes, data: bytes, client_id: str):
    """Edge side. Returns (auth_frame, session_key)."""
    body = decode_frame(psk, data)
    if body.get('type') != 'challenge' or body.get('proto') != PROTOCOL_VERSION or not _hex_nonce(body.get('server_nonce')):
        raise AuthError('Unexpected challenge or protocol version.')
    client_nonce = _nonce()
    frame = encode_frame(psk, {'type': 'auth', 'proto': PROTOCOL_VERSION, 'server_nonce': body['server_nonce'],
                               'client_nonce': client_nonce, 'client': str(client_id)[:64]})
    return frame, derive_session_key(psk, body['server_nonce'], client_nonce)


def verify_auth(psk: bytes, data: bytes, server_nonce: str):
    """UGV side. Returns (session_key, client_id)."""
    body = decode_frame(psk, data)
    if (body.get('type') != 'auth' or body.get('proto') != PROTOCOL_VERSION
            or not hmac.compare_digest(str(body.get('server_nonce')), server_nonce)
            or not _hex_nonce(body.get('client_nonce'))):
        raise AuthError('Invalid authentication response.')
    return derive_session_key(psk, server_nonce, body['client_nonce']), str(body.get('client', ''))[:64]


# ---------------------------------------------------------------- validation helpers

def finite_number(body: dict, name: str):
    value = body.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError('Field %r must be a finite number.' % name)
    return float(value)


if __name__ == '__main__':  # python -m ugv_bridge.protocol genkey > mentorpi.key
    import sys
    if sys.argv[1:] == ['genkey']:
        print(generate_key_hex())
    else:
        print('usage: python -m ugv_bridge.protocol genkey > mentorpi.key')
