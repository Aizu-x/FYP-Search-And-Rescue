"""The UAV section of the console (ugv_console/uav_console.py): sections, settings, the take-off handshake,
flying with key reports, landing, footage, and what the local server adds (picture endpoint, landing when the
page is gone).

The "drone" is the simulated one (uav/sim.py) started by the console as its own process, exactly as the
"Simulated drone" switch does. No test sends anything to a real drone.
"""
import http.client
import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from ugv_console.app import ConsoleApp
from ugv_console.server import ConsoleServer

from nav_helpers import wait_for


class UavCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='uav console test ')
        self.dir = Path(self.tmp.name)
        self.footage = self.dir / 'drone footage'
        self.apps = []
        self.app = self.make()

    def make(self, settings=False):
        app = ConsoleApp(settings_path=self.dir / 'console_settings.json' if settings else None,
                         edit_dir=self.dir / 'edited', export_dir=self.dir / 'exports')
        app.opts.update(uav_sim=True, uav_dir=str(self.footage), record_dir=str(self.dir / 'mission videos'))
        self.apps.append(app)
        return app

    def tearDown(self):
        for app in self.apps:
            app.close()
        self.tmp.cleanup()

    def cmd(self, name, **args):
        return self.app.command(name, args)

    def ok(self, name, **args):
        r = self.cmd(name, **args)
        self.assertTrue(r['ok'], '%s -> %s' % (name, r['message']))
        return r

    def refused(self, name, text, **args):
        r = self.cmd(name, **args)
        self.assertFalse(r['ok'], name)
        self.assertIn(text, r['message'])
        return r

    def view(self):
        return self.app.view('uav')

    def connect(self):
        self.ok('set_section', section='uav')
        self.ok('uav_connect')
        self.assertTrue(wait_for(lambda: self.app.uav is not None, 40), self.app.uav_note)
        self.assertTrue(wait_for(self.app._uav_live, 20), 'no live picture from the simulated drone')
        self.assertTrue(wait_for(lambda: self.view()['battery'] is not None, 5))

    def take_off(self):
        prepared = self.ok('uav_takeoff_prepare')
        self.ok('uav_takeoff_confirm', nonce=prepared['nonce'])
        self.assertTrue(wait_for(lambda: self.app.uav.state == 'flying', 10), self.app.uav.state)
        return prepared

    def hold(self, keys, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.stamp = getattr(self, 'stamp', 1000.0) + 100.0
            self.ok('uav_keys', keys=keys, t=self.stamp)
            time.sleep(0.1)

    def log(self):
        return ' | '.join(line['text'] for line in self.app.log_lines(200))


class SectionTests(UavCase):
    def test_the_console_starts_in_the_ugv_section_and_switches(self):
        app = self.app
        self.assertEqual(app.opts['section'], 'ugv')
        self.assertEqual(self.view()['state'], 'disconnected')
        self.ok('set_section', section='uav')
        self.assertEqual(app.view('settings')['opts']['section'], 'uav')
        self.refused('set_options', 'has its own command', values=dict(section='ugv'))
        self.refused('set_section', 'must be one of', section='moon')
        self.ok('set_mode', mode='sim')                                         # a mode button leaves the UAV section
        self.assertEqual((app.opts['section'], app.opts['mode']), ('ugv', 'sim'))
        calls = []
        original = app._disconnect
        app._disconnect = lambda: (calls.append(1), original())[1]
        self.ok('set_section', section='uav')
        self.ok('set_mode', mode='sim')                                         # back to the mode the UGV was in:
        self.assertEqual((app.opts['section'], calls), ('ugv', []))             # ... nothing is disconnected
        self.ok('set_mode', mode='offline')
        self.assertEqual(calls, [1])

    def test_the_section_is_remembered(self):
        first = self.make(settings=True)
        self.assertTrue(first.command('set_section', dict(section='uav'))['ok'])
        self.assertTrue(first.command('set_options', dict(values=dict(uav_speed=20)))['ok'])
        first.close()
        second = self.make(settings=True)
        self.assertEqual((second.opts['section'], second.opts['uav_speed']), ('uav', 20))

    def test_not_to_the_drone_while_the_robot_moves(self):
        self.app.is_moving = lambda: True
        self.refused('set_section', 'robot is moving', section='uav')
        self.assertEqual(self.app.opts['section'], 'ugv')

    def test_settings_are_checked(self):
        app = self.app
        self.ok('set_options', values=dict(uav_ip='192.168.10.1', uav_sim=False))
        self.refused('set_options', 'local network', values=dict(uav_ip='8.8.8.8'))
        self.refused('set_options', '192.168.10.1', values=dict(uav_ip='tello'))
        self.assertEqual(app.opts['uav_ip'], '192.168.10.1')                    # a refused value changes nothing
        self.refused('set_options', 'between 10 and 100', values=dict(uav_speed=5))
        self.refused('set_options', 'between 10 and 100', values=dict(uav_speed=150))
        self.ok('set_options', values=dict(uav_speed=60))
        self.assertEqual(self.view()['speed'], 60)
        blocker = self.dir / 'a file'
        blocker.write_text('x')
        self.refused('set_options', 'footage folder', values=dict(uav_dir=str(blocker)))
        self.assertEqual(app.opts['uav_dir'], str(self.footage))
        self.ok('set_options', values=dict(uav_dir=str(self.dir / 'other')))
        self.assertTrue((self.dir / 'other').is_dir())
        self.assertEqual(self.view()['folder'], str(self.dir / 'other'))
        self.ok('set_options', values=dict(uav_record=False))
        self.assertFalse(self.view()['record_on'])

    def test_without_a_drone_nothing_can_be_flown(self):
        self.ok('set_section', section='uav')
        self.refused('uav_takeoff_prepare', 'Connect the drone first')
        self.refused('uav_takeoff_confirm', 'expired', nonce='abc')
        self.refused('uav_land', 'not connected')
        self.refused('uav_emergency', 'not connected')
        self.refused('uav_record', 'Connect the drone first', on=True)
        self.refused('uav_photo', 'no live picture')
        self.ok('uav_keys', keys=['w', 'up'])                                   # accepted, steers nothing
        self.assertIn('not in the air', self.ok('stop')['message'])
        self.ok('set_section', section='ugv')
        self.assertIn('robot stopped', self.ok('stop')['message'])              # the robot's STOP is unchanged
        self.assertEqual(self.ok('uav_disconnect')['message'], 'The drone is not connected.')

    def test_key_reports_are_checked(self):
        for bad in (dict(keys=['x']), dict(keys='w'), dict(keys=['w'] * 9), dict(keys=[1]), dict(), dict(keys=['w'], t='now'),
                    dict(keys=['w'], t=True)):
            self.assertFalse(self.cmd('uav_keys', **bad)['ok'], bad)

    def test_a_drone_that_does_not_answer(self):
        self.ok('set_options', values=dict(uav_sim=False, uav_ip='127.0.0.1'))  # nothing listens on this computer
        self.ok('set_section', section='uav')
        self.ok('uav_connect')
        self.assertEqual(self.view()['state'], 'connecting')
        self.assertEqual(self.ok('uav_connect')['message'], 'Still connecting…')
        self.assertTrue(wait_for(lambda: not self.app.uav_connecting, 15))
        view = self.view()
        self.assertEqual((view['state'], view['connected'], view['note_level']), ('disconnected', False, 'warn'))
        self.assertIn('did not answer', view['note'])
        self.assertIsNone(self.app.uav_video)                                   # the video worker was never started


class FlightTests(UavCase):
    def test_a_whole_flight(self):
        app = self.app
        self.connect()
        view = self.view()
        self.assertEqual((view['state'], view['state_text'], view['height_cm'], view['source'], view['simulated']),
                         ('ready', 'On the ground', 0, 'status', True))
        self.assertIn(view['battery'], (85, 86, 87))
        self.assertTrue(view['video']['live'])
        self.assertEqual((view['takeoff_hint'], view['warnings'], view['rec']['active']), ('', [], False))
        self.hold(['w', 'up'], 0.5)                                             # keys before take-off: nothing
        self.assertEqual((self.view()['sticks'], app.uav.sent['rc'], app.uav.sent['takeoff']), ([0, 0, 0, 0], 0, 0))

        prepared = self.ok('uav_takeoff_prepare')                               # step 1: the question
        self.assertEqual((prepared['title'], prepared['confirm_label']), ('Take off', 'Take off'))
        self.assertIn('Battery', prepared['text'])
        self.assertIn(str(self.footage), prepared['text'])
        self.assertEqual(app.uav.sent['takeoff'], 0)                            # asking does not start anything
        self.refused('uav_takeoff_confirm', 'expired', nonce='0' * 16)          # a wrong code ...
        self.refused('uav_takeoff_confirm', 'expired', nonce=prepared['nonce'])   # ... also uses up the question
        self.assertEqual(app.uav.state, 'ready')
        told, tell = [], app.uav_video.set_airborne                             # what the video worker is told
        app.uav_video.set_airborne = lambda flag: (told.append(flag), tell(flag))[1]
        self.take_off()                                                         # step 2: a fresh question, confirmed
        self.assertEqual(told[0], True)                                         # (it lands the drone if the console dies)
        view = self.view()
        self.assertEqual((view['state'], view['airborne'], view['height_cm']), ('flying', True, 80))
        self.assertTrue(view['rec']['active'] and view['rec']['auto'])          # the flight is being filmed
        self.refused('uav_takeoff_prepare', 'already flying')

        self.hold(['up'], 1.2)
        self.assertEqual(self.view()['sticks'], [0, 0, 35, 0])                  # 35 % is the default stick strength
        self.assertGreater(self.view()['height_cm'], 80)
        self.hold(['right'], 1.2)
        self.assertGreater(self.view()['yaw'], 10)
        self.ok('set_options', values=dict(uav_speed=60))
        self.hold(['w', 'a'], 0.5)
        self.assertEqual((self.view()['sticks'], self.view()['keys']), ([-60, 60, 0, 0], ['w', 'a']))
        self.assertTrue(self.ok('uav_keys', keys=['s'], t=self.stamp - 50)['stale'])     # an old report that arrives late
        self.assertEqual(self.view()['sticks'], [-60, 60, 0, 0])
        time.sleep(0.8)                                                         # the page stops reporting: hover
        self.assertEqual(self.view()['sticks'], [0, 0, 0, 0])
        self.ok('uav_photo')

        self.assertIn('drone is landing', self.ok('stop')['message'])           # STOP / Esc lands the drone
        self.assertIn(app.uav.state, ('landing', 'ready'))
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))
        self.assertTrue(wait_for(lambda: told[-1] is False, 2), told)
        self.assertTrue(wait_for(lambda: app.uav_made == 1, 12), self.log())    # saved 2 s after the landing
        last = self.view()['last']
        self.assertRegex(last['name'], r'^flight_.*_sim\.(mp4|avi)$')
        self.assertEqual(Path(last['path']).parent, self.footage)
        self.assertGreater(last['seconds'], 6.0)                                # take-off + flying + landing + tail
        capture = cv2.VideoCapture(last['path'])
        ok, image = capture.read()
        frames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        capture.release()
        self.assertTrue(ok)
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertAlmostEqual(frames / 30.0, last['seconds'], delta=0.3)
        self.assertTrue(wait_for(lambda: app.uav_photos == 1, 5))
        self.assertTrue(Path(self.view()['photo']['path']).is_file())
        self.assertFalse(self.view()['rec']['active'])
        log = self.log()
        for text in ('Simulated drone connected', 'The drone is in the air', 'The drone has landed', 'Drone footage saved'):
            self.assertIn(text, log)
        self.ok('uav_disconnect')
        self.assertEqual((self.view()['state'], app.uav, app.uav_video, app.uav_sim), ('disconnected', None, None, None))
        self.assertEqual(sorted(p.suffix for p in self.footage.iterdir()), ['.jpg', Path(last['name']).suffix])

    def test_one_machine_at_a_time(self):
        app = self.app
        self.ok('set_mode', mode='sim')                                         # the simulated robot, connected and ready
        self.ok('set_options', values=dict(radius=0.143, margin=0.02, verified=True))
        self.ok('set_start', x=1.215, y=0.405, heading=0)
        self.ok('connect')
        self.assertTrue(wait_for(lambda: app.client.link_ok(), 6))
        time.sleep(0.3)
        self.ok('confirm_pose')
        self.ok('set_goal', x=2.205, y=0.405)
        self.connect()                                                          # the robot stays connected meanwhile
        self.assertIsNotNone(app.nav)
        app.is_moving = lambda: True
        self.refused('uav_takeoff_prepare', 'robot is moving')
        del app.is_moving
        prepared = self.ok('uav_takeoff_prepare')
        app.is_moving = lambda: True                                            # ... or starts to move after the question
        self.refused('uav_takeoff_confirm', 'robot is moving', nonce=prepared['nonce'])
        del app.is_moving
        self.assertEqual(app.uav.sent['takeoff'], 0)
        self.take_off()
        self.refused('set_section', 'land it', section='ugv')                   # the section cannot be left in flight
        self.refused('set_mode', 'land it', mode='offline')
        self.refused('uav_disconnect', 'land it first')
        self.refused('set_options', 'disconnect the drone first', values=dict(uav_sim=False))
        self.refused('authorize_prepare', 'drone is in the air', action='drive')   # and the robot does not drive
        self.assertEqual((app.opts['section'], app.uav.state, app.nav.state), ('uav', 'flying', 'PLANNED'))

        asked = self.cmd('uav_emergency')                                       # cutting the motors asks first
        self.assertEqual((asked['ok'], asked['need_confirm']), (False, 'uav_emergency'))
        self.assertEqual(app.uav.state, 'flying')
        self.ok('uav_emergency', confirm=True)
        self.assertEqual(app.uav.state, 'ready')
        self.assertTrue(wait_for(lambda: self.view()['height_cm'] == 0, 3))
        self.assertGreaterEqual(app.uav.sent['emergency'], 3)
        self.ok('set_section', section='ugv')                                   # on the ground the section can be left
        self.assertTrue(self.ok('authorize_prepare', action='drive')['nonce'])

    def test_recording_by_hand_and_switched_off(self):
        app = self.app
        self.ok('set_options', values=dict(uav_record=False))
        self.connect()
        self.assertIn('not filmed', self.ok('uav_takeoff_prepare')['text'])
        self.ok('uav_record', on=True)                                          # on the ground, by hand
        self.assertTrue(self.view()['rec']['active'])
        self.assertFalse(self.view()['rec']['auto'])
        self.assertEqual(self.ok('uav_record', on=True)['message'], 'Already recording.')
        time.sleep(2.0)
        self.ok('uav_record', on=False)
        self.assertTrue(wait_for(lambda: app.uav_made == 1, 10), self.log())
        self.assertAlmostEqual(self.view()['last']['seconds'], 2.0, delta=0.8)
        self.take_off()                                                         # recording switched off: not filmed
        time.sleep(0.5)
        self.assertFalse(self.view()['rec']['active'])
        self.ok('uav_land')
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))
        time.sleep(2.6)
        self.assertEqual(app.uav_made, 1)
        self.assertEqual(self.ok('uav_record', on=False)['message'], 'Nothing is being recorded.')

    def test_closing_the_console_lands_the_drone_and_keeps_the_footage(self):
        app = self.app
        self.connect()
        self.take_off()
        time.sleep(1.0)
        link = app.uav
        app.close()
        self.assertGreaterEqual(link.sent['land'], 3)
        self.assertEqual(link.state, 'disconnected')
        files = [p.name for p in self.footage.iterdir()]
        self.assertEqual(len(files), 1, files)
        self.assertNotIn('_recording', files[0])
        self.assertIn('Drone footage saved', self.log())


class ServerTests(UavCase):
    def setUp(self):
        super().setUp()
        self.server = ConsoleServer(self.app, deadman_s=0.8, uav_deadman_s=1.5).start()
        self.host = '127.0.0.1:%d' % self.server.port
        time.sleep(0.4)

    def tearDown(self):
        self.server.shutdown()
        super().tearDown()

    def get(self, path, token=True):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=8)
        headers = {'Host': self.host}
        if token:
            headers['X-Console-Token'] = self.server.token
        conn.request('GET', path, headers=headers)
        response = conn.getresponse()
        data = response.read()
        conn.close()
        return response, data

    def open_page(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=8)
        conn.request('GET', '/api/events?k=' + self.server.token, headers={'Host': self.host})
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        first = None
        while first is None:
            line = response.fp.readline().decode()
            if line.startswith('data: '):
                first = json.loads(line[6:])
        return conn, response, first

    def test_the_drone_picture_is_served_to_the_page(self):
        self.assertEqual(self.get('/api/uav_frame?after=-1', token=False)[0].status, 401)
        self.assertEqual(self.get('/api/uav_frame?after=-1')[0].status, 204)     # no drone: no picture
        conn, stream, first = self.open_page()
        self.assertEqual(first['uav']['state'], 'disconnected')                  # the page is told about the drone
        self.assertIn('section', first['settings']['opts'])
        stream.close(); conn.close()
        self.connect()
        response, data = self.get('/api/uav_frame?after=-1')
        self.assertEqual((response.status, response.getheader('Content-Type')), (200, 'image/jpeg'))
        meta = json.loads(response.getheader('X-Frame'))
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual((image.shape, meta['w'], meta['h']), ((480, 640, 3), 640, 480))
        response, _ = self.get('/api/uav_frame?after=%d' % meta['id'])           # waits for the next picture
        self.assertEqual(response.status, 200)
        self.assertGreater(json.loads(response.getheader('X-Frame'))['id'], meta['id'])
        self.assertEqual(self.get('/api/uav_frame?after=abc')[0].status, 200)

    def test_the_drone_lands_when_the_console_page_stays_away(self):
        app = self.app
        self.connect()
        conn, stream, _ = self.open_page()                                      # a console page is open
        self.take_off()
        self.hold(['w'], 1.0)
        self.assertEqual(app.uav.state, 'flying')
        stream.close(); conn.close()                                            # the window is closed / the browser crashed
        time.sleep(0.9)
        self.assertEqual(app.uav.state, 'flying')                               # first it only hovers ...
        self.assertEqual(app.uav.sticks, (0, 0, 0, 0))
        self.assertTrue(wait_for(lambda: app.uav.state in ('landing', 'ready'), 4))    # ... then it is brought down
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))
        self.assertIn('the console page was closed', self.log())


if __name__ == '__main__':
    unittest.main()
