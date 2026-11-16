"""The local web server of the console: untrusted requests are refused, commands and live state work,
and the robot is stopped when the last console page disappears."""
import http.client
import json
import tempfile
import time
import unittest
from pathlib import Path

from ugv_console.app import ConsoleApp
from ugv_console.server import ConsoleServer, WEB_DIR

from nav_helpers import wait_for


class ServerCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv server test ')
        root = Path(self.tmp.name)
        self.app = ConsoleApp(settings_path=None, edit_dir=root / 'edited', export_dir=root / 'exports')
        self.server = ConsoleServer(self.app, deadman_s=0.8).start()
        self.host = '127.0.0.1:%d' % self.server.port
        time.sleep(0.4)                                    # first state published

    def tearDown(self):
        self.server.shutdown()
        self.app.close()
        self.tmp.cleanup()

    def request(self, method, path, body=None, token=True, headers=None, host=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=5)
        send = {'Host': host or self.host}
        if token:
            send['X-Console-Token'] = self.server.token if token is True else token
        if body is not None and 'Content-Type' not in (headers or {}):
            send['Content-Type'] = 'application/json'
        send.update(headers or {})
        payload = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        conn.request(method, path, body=payload, headers=send)
        response = conn.getresponse()
        data = response.read()
        conn.close()
        return response, data

    def cmd(self, name, **args):
        response, data = self.request('POST', '/api/cmd', dict(cmd=name, args=args))
        self.assertEqual(response.status, 200, data)
        return json.loads(data)


class SecurityTests(ServerCase):
    def test_only_this_computer(self):
        self.assertEqual(self.server.httpd.server_address[0], '127.0.0.1')
        with self.assertRaises(ValueError):
            ConsoleServer(self.app, host='0.0.0.0')

    def test_token_host_and_origin_are_required(self):
        stop = dict(cmd='stop', args={})
        self.assertEqual(self.request('POST', '/api/cmd', stop, token=False)[0].status, 401)
        self.assertEqual(self.request('POST', '/api/cmd', stop, token='wrong')[0].status, 401)
        self.assertEqual(self.request('GET', '/api/state', token=False)[0].status, 401)
        self.assertEqual(self.request('GET', '/api/events', token=False)[0].status, 401)
        self.assertEqual(self.request('GET', '/api/frame?after=0', token=False)[0].status, 401)
        # DNS rebinding: right token but the browser thinks it talks to another host
        self.assertEqual(self.request('POST', '/api/cmd', stop, host='evil.example:%d' % self.server.port)[0].status, 403)
        self.assertEqual(self.request('GET', '/', token=False, host='evil.example')[0].status, 403)
        # another web page (foreign Origin) posting with a stolen-looking request
        self.assertEqual(self.request('POST', '/api/cmd', stop, headers={'Origin': 'http://evil.example'})[0].status, 403)
        self.assertEqual(self.request('OPTIONS', '/api/cmd', headers={'Origin': 'http://evil.example'})[0].status, 403)
        ok, _ = self.request('POST', '/api/cmd', stop, headers={'Origin': 'http://' + self.host})
        self.assertEqual(ok.status, 200)
        self.assertEqual(self.request('GET', '/api/state?k=' + self.server.token, token=False)[0].status, 200)
        self.assertIsNone(ok.getheader('Access-Control-Allow-Origin'))          # never any CORS permission

    def test_malformed_and_oversized_commands(self):
        self.assertEqual(self.request('POST', '/api/cmd', b'{"cmd":"stop"}', headers={'Content-Type': 'text/plain'})[0].status, 415)
        self.assertEqual(self.request('POST', '/api/cmd', b'{not json')[0].status, 400)
        self.assertEqual(self.request('POST', '/api/cmd', b'[1,2,3]')[0].status, 400)
        self.assertEqual(self.request('POST', '/api/cmd', {'cmd': 5})[0].status, 400)
        try:                       # the server may hang up before the whole oversized body is sent: also a refusal
            self.assertEqual(self.request('POST', '/api/cmd', b'{"cmd":"x","pad":"' + b'a' * 600000 + b'"}')[0].status, 413)
        except (BrokenPipeError, ConnectionResetError):
            pass
        self.assertEqual(self.request('POST', '/api/other', dict(cmd='stop'))[0].status, 404)
        unknown = self.cmd('format_disk')
        self.assertFalse(unknown['ok'])
        self.assertFalse(self.cmd('__init__')['ok'])
        self.assertFalse(self.cmd('set_goal', x='1; rm -rf /', y=1)['ok'])
        self.assertEqual(self.request('GET', '/api/nothing')[0].status, 404)

    def test_static_files_only_from_the_web_folder(self):
        response, body = self.request('GET', '/', token=False)
        self.assertEqual(response.status, 200)
        self.assertIn(b'UGV Search', body)
        self.assertIn("default-src 'self'", response.getheader('Content-Security-Policy'))
        self.assertEqual(response.getheader('X-Content-Type-Options'), 'nosniff')
        self.assertEqual(response.getheader('Cache-Control'), 'no-store')
        for name in ('/style.css', '/js/main.js', '/js/map.js', '/js/ui.js', '/js/api.js', '/js/camera.js', '/icon.svg'):
            self.assertEqual(self.request('GET', name, token=False)[0].status, 200, name)
        for name in ('/../app.py', '/js/../../app.py', '/..%2fapp.py', '/%2e%2e/server.py', '/js/main.py', '/index.html.bak',
                     '/C:/Windows/win.ini', '//etc/passwd'):
            self.assertEqual(self.request('GET', name, token=False)[0].status, 404, name)
        self.assertNotIn(b'http://', (WEB_DIR / 'index.html').read_bytes().replace(b'http://www.w3.org', b''))   # nothing external
        self.assertNotIn(b'https://', (WEB_DIR / 'index.html').read_bytes())
        self.assertNotIn(b'https://', (WEB_DIR / 'style.css').read_bytes())


class LiveStateTests(ServerCase):
    def read_events(self, seconds, want=2):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=seconds + 3)
        conn.request('GET', '/api/events?k=' + self.server.token, headers={'Host': self.host})
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        self.assertTrue(response.getheader('Content-Type').startswith('text/event-stream'))
        events, deadline = [], time.monotonic() + seconds
        while time.monotonic() < deadline and len(events) < want:
            line = response.fp.readline().decode()
            if line.startswith('data: '):
                events.append(json.loads(line[6:]))
        conn.keep = response                               # the stream stays open while the test holds this
        return conn, events

    def test_commands_and_event_stream(self):
        r = self.cmd('set_goal', x=1.395, y=1.335)
        self.assertTrue(r['ok'])
        time.sleep(0.2)                                    # (a change reaches the stream within one 50 ms tick)
        conn, events = self.read_events(4, want=25)
        conn.close()
        first = events[0]
        for section in ('fast', 'static', 'settings', 'map', 'plan', 'nav', 'log', 'offline'):
            self.assertIn(section, first, section)                              # a new page gets everything once
        self.assertEqual(first['plan']['goal'], [1.395, 1.335])
        self.assertEqual(first['map']['w'], 100)
        later = events[5]
        self.assertIn('fast', later)
        self.assertNotIn('map', later)                                          # unchanged sections are not resent
        times = [e['fast']['t'] for e in events]
        self.assertEqual(times, sorted(times))
        gaps = [b - a for a, b in zip(times[3:], times[4:])]
        self.assertLess(sum(gaps) / len(gaps), 0.08)                            # ~20 samples per second
        state = json.loads(self.request('GET', '/api/state')[1])
        self.assertEqual(state['settings']['opts']['mode'], 'offline')
        self.assertEqual(self.request('GET', '/api/frame?after=-1')[0].status, 204)   # camera off: no frame

    def test_robot_is_stopped_when_the_console_page_disappears(self):
        app = self.app
        self.assertTrue(self.cmd('set_mode', mode='sim')['ok'])
        self.assertTrue(self.cmd('set_options', values=dict(radius=0.143, margin=0.02, verified=True))['ok'])
        self.assertTrue(self.cmd('set_start', x=1.215, y=0.405, heading=0)['ok'])
        self.assertTrue(self.cmd('connect')['ok'])
        self.assertTrue(wait_for(lambda: app.client.link_ok(), timeout=6)); time.sleep(0.3)
        self.assertTrue(self.cmd('confirm_pose')['ok'])
        self.assertTrue(self.cmd('set_goal', x=2.205, y=0.405)['ok'])
        conn, _ = self.read_events(3, want=2)                                   # a console page is open
        self.assertEqual(self.server.clients(), 1)
        prepared = self.cmd('authorize_prepare', action='drive')
        self.assertTrue(self.cmd('authorize_confirm', nonce=prepared['nonce'])['ok'])
        self.assertEqual(app.nav.state, 'RUNNING')
        time.sleep(1.2)
        self.assertEqual(app.nav.state, 'RUNNING', app.nav.message)             # page open: keeps driving
        conn.keep.close(); conn.close()                                         # window closed / browser crashed
        self.assertTrue(wait_for(lambda: app.nav.state == 'STOPPED', timeout=4), app.nav.state)
        self.assertTrue(app.nav.safety.estop)
        self.assertIn('Console window closed', app.nav.message)
        self.assertTrue(wait_for(lambda: not app.session.server.armed, timeout=2))


if __name__ == '__main__':
    unittest.main()
