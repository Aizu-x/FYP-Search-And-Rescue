"""Local web server for the console (Python standard library only).

    browser page  <--  GET  /api/events   live state (Server-Sent Events, ~20 Hz)
                  <--  GET  /api/frame    newest camera frame + YOLO boxes (long poll)
                  -->  POST /api/cmd      one validated command  {"cmd": ..., "args": {...}}

Security (the page can start robot motion, so every request is treated as untrusted):

* listens on 127.0.0.1 only -- never reachable from the network;
* every /api request needs the random per-run token (header ``X-Console-Token`` or ``?k=``);
* ``Host`` must be this server (stops DNS-rebinding), a foreign ``Origin`` is refused (stops other
  web pages from posting commands), no CORS headers are ever sent;
* commands are size-limited JSON, dispatched through ``ConsoleApp.command`` (whitelist + validation);
* if the last console page disappears while the robot is moving, the robot is STOPPED.
"""
from __future__ import annotations

import hmac
import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from .views import SECTIONS

WEB_DIR = Path(__file__).resolve().parent / 'web'
MIME = {'.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
        '.js': 'text/javascript; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png',
        '.json': 'application/json', '.ico': 'image/x-icon'}
MAX_BODY = 512 * 1024
SLOW_EVERY = 4            # publisher ticks between "slow" sections (20 Hz / 4 = 5 Hz)
SLOWER = {'trail': 8, 'seen': 12, 'diag': 8, 'room': 12}
SECURITY_HEADERS = (('Cache-Control', 'no-store'), ('X-Content-Type-Options', 'nosniff'),
                    ('Referrer-Policy', 'no-referrer'), ('X-Frame-Options', 'DENY'),
                    ('Content-Security-Policy',
                     "default-src 'self'; img-src 'self' blob: data:; style-src 'self' 'unsafe-inline'; "
                     "connect-src 'self'; frame-ancestors 'none'"))


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'UGVConsole/2'
    sys_version = ''

    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def log_message(self, fmt, *args):
        if self.server.console.verbose:
            sys.stderr.write('%s %s\n' % (self.address_string(), fmt % args))

    # ------------------------------------------------------------------ plumbing
    def _send(self, code, body=b'', content_type='text/plain; charset=utf-8', extra=()):
        self.send_response(code)
        if code >= 400:                            # a refused request may have an unread body: never reuse the socket
            self.close_connection = True
            self.send_header('Connection', 'close')
        for name, value in SECURITY_HEADERS + tuple(extra):
            self.send_header(name, value)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        if body and self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, separators=(',', ':')).encode('utf-8'), 'application/json')

    def _trusted(self, query, need_token=True):
        console = self.server.console
        if self.headers.get('Host', '') not in console.hosts:
            self._send(403, b'Forbidden host.')
            return False
        origin = self.headers.get('Origin')
        if origin is not None and origin not in console.origins:
            self._send(403, b'Forbidden origin.')
            return False
        if need_token:
            token = self.headers.get('X-Console-Token') or (query.get('k') or [''])[0]
            if not hmac.compare_digest(token.encode('utf-8', 'replace'), console.token.encode()):
                self._send(401, b'Missing or wrong console token. Open the link printed in the terminal.')
                return False
        return True

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        url = urlsplit(self.path)
        query = parse_qs(url.query)
        path = url.path
        console = self.server.console
        try:
            if path == '/api/events':
                if self._trusted(query):
                    console._stream(self)
            elif path == '/api/frame':
                if self._trusted(query):
                    self._frame(query)
            elif path == '/api/state':
                if self._trusted(query):
                    self._json(200, console.full_state())
            elif path.startswith('/api/'):
                self._send(404, b'Not found.')
            elif self._trusted(query, need_token=False):
                self._static(path)
        except (BrokenPipeError, ConnectionError, socket.timeout):
            self.close_connection = True

    do_HEAD = do_GET

    def _static(self, path):
        if path in ('', '/'):
            path = '/index.html'
        relative = path.lstrip('/')
        allowed = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-./')
        if '..' in relative or not relative or any(ch not in allowed for ch in relative):
            return self._send(404, b'Not found.')
        target = (WEB_DIR / relative).resolve()
        if WEB_DIR not in target.parents or not target.is_file() or target.suffix not in MIME:
            return self._send(404, b'Not found.')
        self._send(200, target.read_bytes(), MIME[target.suffix])

    def _frame(self, query):
        try:
            after = int((query.get('after') or ['-1'])[0])
        except ValueError:
            after = -1
        vision = self.server.console.app.vision
        frame = vision.wait_frame(after, timeout=1.5) if vision is not None else None
        if frame is None:
            if vision is None:
                time.sleep(0.4)                   # camera off: do not spin
            return self._send(204)
        frame_id, jpeg, width, height, boxes = frame
        meta = dict(id=frame_id, w=width, h=height,
                    boxes=[dict(b=[round(v, 1) for v in box['box']], c=box['cls'], p=round(box['conf'], 2),
                                r=None if box['range'] is None else round(box['range'], 3), u=bool(box['used']),
                                n=box['note']) for box in boxes[:40]])
        self._send(200, jpeg, 'image/jpeg', extra=(('X-Frame', json.dumps(meta, separators=(',', ':'))),))

    # ------------------------------------------------------------------ POST
    def do_POST(self):
        url = urlsplit(self.path)
        if not self._trusted(parse_qs(url.query)):
            return
        if url.path != '/api/cmd':
            return self._send(404, b'Not found.')
        if 'application/json' not in self.headers.get('Content-Type', ''):
            return self._send(415, b'JSON only.')
        try:
            length = int(self.headers.get('Content-Length', ''))
        except ValueError:
            return self._send(411, b'Length required.')
        if not 0 < length <= MAX_BODY:
            self.close_connection = True
            return self._send(413, b'Request too large.')
        try:
            body = json.loads(self.rfile.read(length).decode('utf-8'))
            name, args = body['cmd'], body.get('args') or {}
            if not isinstance(name, str) or not isinstance(args, dict):
                raise ValueError
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            return self._json(400, dict(ok=False, message='Malformed command.', level='error'))
        self._json(200, self.server.console.app.command(name, args))

    def do_OPTIONS(self):                          # no CORS: cross-origin pages get nothing
        self._send(403, b'Forbidden.')


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False
    request_queue_size = 32

    def handle_error(self, request, client_address):
        if self.console.verbose:
            traceback.print_exc()


class ConsoleServer:
    def __init__(self, app, host='127.0.0.1', port=0, token=None, deadman_s=1.5, idle_exit_s=0.0, verbose=False):
        if host not in ('127.0.0.1', 'localhost'):
            raise ValueError('The console only listens on this computer (127.0.0.1).')
        self.app = app
        self.verbose = verbose
        self.token = token or secrets.token_urlsafe(24)
        self.deadman_s, self.idle_exit_s = deadman_s, idle_exit_s
        self.httpd = _Server(('127.0.0.1', port), _Handler)
        self.httpd.console = self
        self.port = self.httpd.server_address[1]
        self.hosts = {'127.0.0.1:%d' % self.port, 'localhost:%d' % self.port}
        self.origins = {'http://' + h for h in self.hosts}
        self.exit_requested = threading.Event()
        self._stop = threading.Event()
        self._cond = threading.Condition()
        self._seq, self._fast = 0, '{}'
        self._json, self._ver = {}, {}
        self._clients = 0
        self._ever_connected = False
        self._last_client = time.monotonic()
        self._deadman_fired = False
        self._threads = []

    @property
    def url(self):
        return 'http://127.0.0.1:%d/#k=%s' % (self.port, self.token)

    def start(self):
        for target, name in ((self.httpd.serve_forever, 'console-http'), (self._publish_loop, 'console-publish'),
                             (self._watchdog_loop, 'console-watchdog')):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def shutdown(self):
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self.httpd.shutdown()
        self.httpd.server_close()

    # ------------------------------------------------------------------ state publishing
    def _build(self, name):
        try:
            text = json.dumps(self.app.view(name), separators=(',', ':'), allow_nan=False)
        except Exception:
            if self.verbose:
                traceback.print_exc()
            return
        if text != self._json.get(name):
            self._json[name] = text
            self._ver[name] = self._ver.get(name, 0) + 1

    def _publish_loop(self):
        tick, period, seen_revision = 0, 0.05, None
        deadline = time.monotonic()
        while not self._stop.is_set():
            try:
                fast = json.dumps(self.app.fast(), separators=(',', ':'), allow_nan=False)
            except Exception:
                fast = '{}'
            revision = sum(self.app.revs.values())         # a command changed something: publish it right away
            if tick % SLOW_EVERY == 0 or revision != seen_revision:
                urgent = revision != seen_revision
                seen_revision = revision
                for name in SECTIONS:
                    if name == 'static' and name in self._json:
                        continue
                    if name in SLOWER and tick % SLOWER[name] and name in self._json and not urgent:
                        continue
                    self._build(name)
            with self._cond:
                self._fast = fast
                self._seq += 1
                self._cond.notify_all()
            tick += 1
            deadline += period
            delay = deadline - time.monotonic()
            if delay < -0.5:                       # fell behind (e.g. laptop was asleep): do not burst
                deadline = time.monotonic()
            self._stop.wait(max(0.0, delay))

    def full_state(self):
        with self._cond:
            sections = {name: json.loads(text) for name, text in self._json.items()}
            sections['fast'] = json.loads(self._fast)
        return sections

    def _stream(self, handler):
        handler.send_response(200)
        for name, value in SECURITY_HEADERS:
            handler.send_header(name, value)
        handler.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        handler.send_header('Connection', 'close')
        handler.end_headers()
        handler.close_connection = True
        seen, last = {}, -1
        with self._cond:
            self._clients += 1
            self._ever_connected = True
            self._deadman_fired = False
        try:
            handler.wfile.write(b'retry: 800\n\n')
            while not self._stop.is_set():
                with self._cond:
                    self._cond.wait_for(lambda: self._seq != last or self._stop.is_set(), timeout=1.0)
                    if self._seq == last:
                        payload = ': keepalive\n\n'
                    else:
                        last = self._seq
                        parts = ['"fast":' + self._fast]
                        for name, version in self._ver.items():
                            if seen.get(name) != version:
                                seen[name] = version
                                parts.append('"%s":%s' % (name, self._json[name]))
                        payload = 'data: {' + ','.join(parts) + '}\n\n'
                handler.wfile.write(payload.encode('utf-8'))
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        finally:
            with self._cond:
                self._clients -= 1
                self._last_client = time.monotonic()

    # ------------------------------------------------------------------ console presence
    def clients(self):
        return self._clients

    def _watchdog_loop(self):
        while not self._stop.wait(0.25):
            if self._clients > 0 or not self._ever_connected:
                continue
            idle = time.monotonic() - self._last_client
            if idle > self.deadman_s and not self._deadman_fired and self.app.is_moving():
                self._deadman_fired = True
                self.app.command('stop', dict(reason='Console window closed'))
                print('Console page closed while the robot was moving: STOP sent.')
            if self.idle_exit_s and idle > self.idle_exit_s:
                self.exit_requested.set()


# ---------------------------------------------------------------------------- launching
def _browser_candidates():
    names = []
    if sys.platform.startswith('win'):
        roots = [os.environ.get(k) for k in ('PROGRAMFILES(X86)', 'PROGRAMFILES', 'LOCALAPPDATA')]
        for root in filter(None, roots):
            names += [os.path.join(root, 'Microsoft', 'Edge', 'Application', 'msedge.exe'),
                      os.path.join(root, 'Google', 'Chrome', 'Application', 'chrome.exe'),
                      os.path.join(root, 'BraveSoftware', 'Brave-Browser', 'Application', 'brave.exe')]
    elif sys.platform == 'darwin':
        names += ['/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge',
                  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome']
    else:
        names += [shutil.which(n) for n in ('microsoft-edge', 'google-chrome', 'chromium', 'chromium-browser')]
    return [n for n in names if n and os.path.isfile(n)]


def open_console(url, app_window=True):
    """Open the console page: its own app-style window if Edge/Chrome is found, else a normal browser tab."""
    if app_window:
        for exe in _browser_candidates():
            try:
                subprocess.Popen([exe, '--app=' + url, '--window-size=1520,940'],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return 'window'
            except OSError:
                continue
    try:
        return 'tab' if webbrowser.open(url) else 'none'
    except Exception:
        return 'none'


def _pick_port(preferred):
    for port in ([preferred] if preferred else []) + list(range(8780, 8800)):
        with socket.socket() as probe:
            try:
                probe.bind(('127.0.0.1', port))
                return port
            except OSError:
                continue
    return 0


def run(config_path=None, port=None, browser='window', stay=False, verbose=False):
    """Start the console and block until Ctrl+C (or until the page has been closed for a while)."""
    from .app import ConsoleApp
    from ugv_nav.config import DEFAULT_CONFIG
    app = ConsoleApp(config_path or DEFAULT_CONFIG)
    server = ConsoleServer(app, port=_pick_port(port), idle_exit_s=0.0 if stay else 25.0, verbose=verbose).start()
    print('\nUGV console is running.  Keep this terminal open.\n\n    %s\n' % server.url)
    if app.config_error:
        print('WARNING: config/navigation.toml could not be loaded (%s). Planner and simulator still work.'
              % app.config_error)
    opened = open_console(server.url, app_window=browser == 'window') if browser != 'none' else 'none'
    print({'window': 'The console opened in its own window.', 'tab': 'The console opened in your browser.',
           'none': 'Open the link above in Edge or Chrome.'}[opened])
    print('STOP = Esc in the console. Ctrl+C here quits%s.\n'
          % ('' if stay else ' (it also quits 25 s after you close the console page)'))
    try:
        while not server.exit_requested.wait(0.25):
            pass
        print('Console page closed: shutting down.')
    except KeyboardInterrupt:
        print('\nShutting down.')
    finally:
        server.shutdown()
        app.close()
    return 0
