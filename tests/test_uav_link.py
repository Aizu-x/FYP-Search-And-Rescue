"""The link to the Tello drone (uav/tello.py), tested against the simulated drone (uav/sim.py) on this computer.

Nothing in here talks to a real drone: every socket is on 127.0.0.1. What these tests can show is that the link
follows the SDK document and its own safety rules; what the real aircraft does with the commands is not tested.
"""
import socket
import time
import unittest

from uav.sim import FakeTello
from uav.tello import (FLYING, LANDING, READY, TAKING_OFF, TelloConfig, TelloError, TelloLink, check_host,
                       parse_state, parse_value, sticks_from_keys)

from nav_helpers import NO_SECOND_ADDRESS, second_address_socket, wait_for


class TextTests(unittest.TestCase):
    def test_status_text_is_read(self):
        text = ('pitch:1;roll:-2;yaw:30;vgx:0;vgy0;vgz:0;templ:60;temph:62;tof:118;h:110;bat:87;baro:44.53;'
                'time:12;agx:1.00;agy:-3.00;agz:-999.00;\r\n')                  # ("vgy0": the SDK document's own typo)
        values = parse_state(text)
        self.assertEqual((values['h'], values['bat'], values['tof'], values['yaw'], values['time']), (110, 87, 118, 30, 12))
        self.assertAlmostEqual(values['baro'], 44.53)
        self.assertNotIn('vgy0', values)
        newer = parse_state('mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;bat:55;h:20;')   # other firmware adds fields
        self.assertEqual((newer['bat'], newer['h']), (55, 20))
        self.assertNotIn('mpry', newer)
        for junk in ('', 'ok', ';;;:', 'bat:abc;h:;', 'bat:nan;h:inf', 'a' * 5000, 'x:1e99'):
            self.assertEqual(parse_state(junk), {}, junk)

    def test_answers_to_read_commands(self):
        self.assertEqual(parse_value('87\r\n'), (87.0, ''))
        self.assertEqual(parse_value('6dm'), (6.0, 'dm'))
        self.assertEqual(parse_value('766mm'), (766.0, 'mm'))
        self.assertEqual(parse_value('-3'), (-3.0, ''))
        for text in ('ok', 'error', 'error Not joystick', '', 'unknown command: x'):
            self.assertIsNone(parse_value(text), text)

    def test_keys_become_stick_values(self):
        # rc a b c d: a = left / right, b = forward / back, c = up / down, d = turn (SDK document)
        self.assertEqual(sticks_from_keys(['w'], 40), (0, 40, 0, 0))
        self.assertEqual(sticks_from_keys(['s'], 40), (0, -40, 0, 0))
        self.assertEqual(sticks_from_keys(['d'], 40), (40, 0, 0, 0))
        self.assertEqual(sticks_from_keys(['a'], 40), (-40, 0, 0, 0))
        self.assertEqual(sticks_from_keys(['up'], 40), (0, 0, 40, 0))
        self.assertEqual(sticks_from_keys(['down'], 40), (0, 0, -40, 0))
        self.assertEqual(sticks_from_keys(['right'], 40), (0, 0, 0, 40))
        self.assertEqual(sticks_from_keys(['left'], 40), (0, 0, 0, -40))
        self.assertEqual(sticks_from_keys(['w', 'a', 'up', 'left'], 25), (-25, 25, 25, -25))
        self.assertEqual(sticks_from_keys(['w', 's', 'left', 'right'], 80), (0, 0, 0, 0))     # opposite keys cancel
        self.assertEqual(sticks_from_keys([], 80), (0, 0, 0, 0))
        self.assertEqual(sticks_from_keys(['w'], 500), (0, 100, 0, 0))                        # never above 100
        self.assertEqual(sticks_from_keys(['w'], -5), (0, 0, 0, 0))

    def test_only_addresses_of_a_local_network(self):
        for good in ('192.168.10.1', ' 192.168.10.1 ', '127.0.0.1', '10.0.0.7', '172.16.5.5', '169.254.1.1'):
            self.assertEqual(check_host(good), good.strip())
        for bad in ('8.8.8.8', 'tello.local', '', '192.168.10', '255.255.255.255', '224.0.0.1', '0.0.0.0', '1.2.3.4.5',
                    'fe80::1', None):
            with self.assertRaises(ValueError, msg=bad):
                check_host(bad)


class LinkCase(unittest.TestCase):
    SIM = dict(video=False, takeoff_s=0.4, land_speed=1.0)
    CFG = dict(takeoff_timeout_s=3.0, land_timeout_s=6.0, land_resend_s=0.4, stale_reply_s=0.15, key_timeout_s=0.3,
               silent_s=0.7, lost_land_s=1.4, status_stale_s=0.5, ask_period_s=0.25, reply_timeout_s=0.6)

    def setUp(self):
        self.sim = FakeTello(**self.SIM).start()
        self.link = None
        self.addCleanup(self.sim.stop)

    def connect(self, **cfg):
        settings = dict(self.CFG, **cfg)
        self.link = TelloLink(TelloConfig(host='127.0.0.1', cmd_port=self.sim.cmd_port, state_port=0, **settings))
        self.addCleanup(self.link.close)
        self.link.bind()
        self.sim.state_port = self.link.state_port
        self.link.handshake()
        return self.link

    def fly(self, **cfg):
        link = self.connect(**cfg)
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        link.takeoff()
        self.assertTrue(wait_for(lambda: link.state == FLYING, 4), link.state)
        return link

    def hold(self, keys, seconds, percent=60):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.link.set_keys(keys, percent)
            time.sleep(0.08)

    def events(self):
        return ' | '.join(text for _, text in self.link.take_events())


class LinkTests(LinkCase):
    def test_connects_and_reads_battery_and_height(self):
        link = self.connect()
        self.assertEqual(link.state, READY)
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        snap = link.snapshot()
        self.assertIn(snap['battery'], (86, 87))
        self.assertEqual((snap['height_cm'], snap['source'], snap['silent'], snap['status_blocked']), (0, 'status', False, False))
        self.assertEqual(list(self.sim.commands), ['command'])                  # "command" first, as the SDK says

    def test_nothing_moves_before_take_off(self):
        link = self.connect()
        self.hold(['w', 'up'], 0.6)
        self.assertEqual(link.sticks, (0, 0, 0, 0))
        self.assertEqual([c for c in self.sim.commands if c != 'command'], [])   # no "rc", no "takeoff"
        self.assertEqual((self.sim.phase, self.sim.z), ('ground', 0.0))

    def test_take_off_fly_with_the_keys_and_land(self):
        link, sim = self.fly(), self.sim
        self.assertAlmostEqual(sim.z, 0.8, delta=0.05)
        self.assertGreaterEqual(link.height_cm(), 70)
        x0 = sim.x
        self.hold(['w'], 0.8)
        self.assertGreater(sim.x - x0, 0.25)                                    # W: forward (the drone faces +x)
        self.assertEqual(link.last_rc, 'rc 0 60 0 0')
        y0 = sim.y
        self.hold(['d'], 0.8)
        self.assertLess(sim.y - y0, -0.25)                                      # D: to the right
        y0 = sim.y
        self.hold(['a'], 1.2)
        self.assertGreater(sim.y - y0, 0.25)                                    # A: to the left
        z0 = sim.z
        self.hold(['up'], 0.8)
        self.assertGreater(sim.z - z0, 0.15)                                    # arrow up: higher
        z0 = sim.z
        self.hold(['down'], 0.8)
        self.assertLess(sim.z - z0, -0.1)                                       # arrow down: lower
        self.hold([], 0.6)
        yaw0 = sim.yaw_deg()
        self.hold(['right'], 0.8)
        self.assertGreater(sim.yaw_deg() - yaw0, 15)                            # right arrow: clockwise
        yaw0 = sim.yaw_deg()
        self.hold(['left'], 1.2)
        self.assertLess(sim.yaw_deg() - yaw0, -15)                              # left arrow: counter-clockwise
        self.hold([], 0.3)
        self.assertEqual(link.last_rc, 'rc 0 0 0 0')                            # keys released: hover
        self.assertTrue(link.land())
        self.assertEqual(link.state, LANDING)
        self.assertTrue(wait_for(lambda: link.state == READY, 6))
        self.assertEqual((sim.phase, sim.z), ('ground', 0.0))
        self.assertEqual(link.sticks, (0, 0, 0, 0))
        rc_before = link.sent['rc']
        time.sleep(0.4)
        self.assertEqual(link.sent['rc'], rc_before)                            # on the ground: no stick values
        self.assertIn('The drone is in the air', self.events())

    def test_hover_when_the_page_stops_reporting_its_keys(self):
        link = self.fly()
        link.set_keys(['w'], 70)                                                # one report, then the page is silent
        self.assertEqual(link.sticks, (0, 70, 0, 0))
        time.sleep(0.2)
        self.assertEqual(link.last_rc, 'rc 0 70 0 0')
        time.sleep(0.5)
        self.assertEqual(link.sticks, (0, 0, 0, 0))
        self.assertEqual(link.last_rc, 'rc 0 0 0 0')
        self.assertIn('stopped reporting its keys', self.events())
        self.assertEqual(link.state, FLYING)                                    # hovering, not landing

    def test_land_is_repeated_until_it_gets_through(self):
        link, sim = self.fly(), self.sim
        sim.lose['land'] = 1                                                    # the first "land" is lost on the way
        link.land()
        time.sleep(0.25)
        self.assertEqual(sim.phase, 'air')
        self.assertTrue(wait_for(lambda: link.state == READY, 6))
        self.assertEqual(sim.phase, 'ground')
        self.assertGreaterEqual(link.sent['land'], 2)

    def test_landing_is_noticed_when_the_answer_is_lost(self):
        link, sim = self.fly(), self.sim
        sim.drop_answers.add('land')
        link.land()
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 4))
        self.assertTrue(wait_for(lambda: link.state == READY, 6), link.state)   # from the height in the status
        self.assertIn('on the ground', self.events())

    def test_no_take_off_with_a_low_battery(self):
        self.sim.battery = 12.0
        link = self.connect()
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        with self.assertRaises(TelloError) as caught:
            link.takeoff()
        self.assertIn('too low', str(caught.exception))
        self.assertEqual((link.state, link.sent['takeoff'], self.sim.phase), (READY, 0, 'ground'))

    def test_no_take_off_without_a_battery_reading(self):
        self.sim.mute_status = True
        self.sim.drop_answers.add('battery?')
        link = self.connect()
        time.sleep(0.3)
        with self.assertRaises(TelloError) as caught:
            link.takeoff()
        self.assertIn('No battery reading', str(caught.exception))
        self.assertEqual(link.sent['takeoff'], 0)

    def test_take_off_refused_by_the_drone(self):
        self.sim.refuse_takeoff = True
        link = self.connect()
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        link.takeoff()
        self.assertTrue(wait_for(lambda: link.state == READY, 3))
        self.assertIn('refused to take off', self.events())
        self.assertEqual(link.sent['takeoff'], 1)                               # never repeated by itself

    def test_take_off_is_noticed_when_the_answer_is_lost(self):
        self.sim.drop_answers.add('takeoff')
        link = self.connect(takeoff_timeout_s=1.2)
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        link.takeoff()
        time.sleep(0.8)
        self.assertEqual(link.state, TAKING_OFF)
        self.assertTrue(wait_for(lambda: link.state == FLYING, 3))              # from the height in the status
        self.assertEqual(link.sent['takeoff'], 1)

    def test_land_pressed_during_take_off(self):
        self.sim.takeoff_s = 1.0
        link, sim = self.connect(), self.sim
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        link.takeoff()
        time.sleep(0.3)
        link.land()                                                             # (the simulated drone answers "error": busy)
        self.assertEqual(link.state, LANDING)
        self.assertTrue(wait_for(lambda: sim.phase == 'air', 3))                # the take-off completes ...
        self.assertEqual(link.state, LANDING)                                   # ... its "ok" is not taken for the landing
        self.assertTrue(wait_for(lambda: link.state == READY, 6))               # ... and the repeated "land" brings it down
        self.assertLess(sim.z, 0.1)
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 2))

    def test_answers_to_stick_commands_do_not_end_a_landing_early(self):
        self.sim.answer_rc = True                                               # (the SDK document lists an answer to "rc")
        link, sim = self.fly(), self.sim
        self.hold(['up'], 0.6)
        link.land()
        heights = []
        while link.state == LANDING and len(heights) < 400:
            heights.append(sim.z)
            time.sleep(0.02)
        self.assertEqual(link.state, READY)
        self.assertLess(sim.z, 0.1)                                             # down (the height is reported in 10 cm steps)
        self.assertGreater(len(heights), 10)                                    # the landing took its time
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 2))

    def test_emergency_cuts_the_motors(self):
        link, sim = self.fly(), self.sim
        self.assertTrue(link.emergency())
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 2))
        self.assertEqual((sim.z, link.state, link.sticks), (0.0, READY, (0, 0, 0, 0)))
        self.assertGreaterEqual(link.sent['emergency'], 3)

    def test_blocked_status_port_battery_and_height_are_asked_for(self):
        self.sim.mute_status = True                                             # as with a firewall in the way
        link = self.connect()
        self.assertTrue(wait_for(lambda: link.battery() is not None and link.height_cm() is not None, 4))
        snap = link.snapshot()
        self.assertEqual((snap['source'], snap['height_cm']), ('asked', 0))
        self.assertIn(snap['battery'], (86, 87))
        link.takeoff()
        self.assertTrue(wait_for(lambda: link.state == FLYING, 4))
        self.assertTrue(wait_for(lambda: (link.height_cm() or 0) >= 70, 3))     # "8dm" -> 80 cm
        self.assertTrue(wait_for(lambda: link.snapshot()['status_blocked'], 5))
        asked = link.sent['battery?'] + link.sent['height?']
        time.sleep(1.0)
        self.assertLessEqual(link.sent['battery?'] + link.sent['height?'] - asked, 6)     # a few a second, not a flood

    def test_silence_means_hover_then_land(self):
        link = self.fly()
        self.hold(['w'], 0.4)
        self.sim.stop()                                                         # the drone is gone (out of range)
        self.hold(['w'], 1.0)                                                   # the operator still holds W
        self.assertEqual(link.last_rc, 'rc 0 0 0 0')                            # nothing heard: sticks to zero
        self.assertTrue(link.snapshot()['silent'])
        self.assertTrue(wait_for(lambda: link.state == LANDING, 3))             # still nothing: "land"
        self.assertTrue(wait_for(lambda: link.sent['land'] >= 1, 1))
        events = self.events()
        self.assertIn('Nothing heard from the drone', events)
        self.assertIn('Landing (nothing heard', events)

    def test_datagrams_from_elsewhere_are_dropped(self):
        other = second_address_socket(self)                                     # None: not possible on this computer
        link = self.connect()
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        link.takeoff()                                                          # waiting for the drone's answer
        stranger = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(stranger.close)
        stranger.bind(('127.0.0.1', 0))
        stranger.sendto(b'error', link._sock.getsockname())                     # not from the drone's port
        time.sleep(0.2)
        self.assertEqual(link.state, TAKING_OFF)
        self.assertEqual(link.rejected, 1)
        if other is not None:
            other.sendto(b'bat:3;h:900;', ('127.0.0.1', link.state_port))       # another address of this computer
            time.sleep(0.3)
            self.assertEqual(link.rejected, 2)
            self.assertGreater(link.battery(), 80)                              # the forged status changed nothing
        self.assertTrue(wait_for(lambda: link.state == FLYING, 4))
        if other is None:
            self.skipTest('the wrong-port half passed; the wrong-address half was not run: ' + NO_SECOND_ADDRESS)

    def test_no_answer_says_what_to_check(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
        probe.close()                                                           # nobody listens there
        link = TelloLink(TelloConfig(host='127.0.0.1', cmd_port=port, state_port=0, connect_tries=2, reply_timeout_s=0.3))
        t0 = time.monotonic()
        with self.assertRaises(TelloError) as caught:
            link.open()
        self.assertLess(time.monotonic() - t0, 3.0)
        self.assertIn('did not answer', str(caught.exception))
        self.assertEqual(link.state, 'disconnected')
        self.assertEqual(link.sent['takeoff'] + link.sent['rc'], 0)

    def test_closing_the_link_lands_a_flying_drone(self):
        link, sim = self.fly(), self.sim
        link.close()
        self.assertGreaterEqual(link.sent['land'], 3)
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 4))


class SimulatedDroneTests(unittest.TestCase):
    def test_lands_by_itself_without_commands(self):
        """The Tello safety feature of the SDK document (15 s; shortened here)."""
        sim = FakeTello(video=False, takeoff_s=0.3, land_speed=2.0, failsafe_s=0.8).start()
        self.addCleanup(sim.stop)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sock.close)
        sock.bind(('127.0.0.1', 0))
        sock.settimeout(3.0)
        target = ('127.0.0.1', sim.cmd_port)
        sock.sendto(b'takeoff', target)                                         # before "command": ignored
        time.sleep(0.2)
        self.assertEqual(sim.phase, 'ground')
        sock.sendto(b'command', target)
        self.assertEqual(sock.recvfrom(100)[0], b'ok')
        sock.sendto(b'takeoff', target)
        self.assertEqual(sock.recvfrom(100)[0], b'ok')
        self.assertEqual(sim.phase, 'air')
        for _ in range(8):                                                      # commands keep arriving: it stays up
            sock.sendto(b'rc 0 0 0 0', target)
            time.sleep(0.15)
        self.assertEqual(sim.phase, 'air')
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 4))             # silence: it lands
        self.assertTrue(sim.landed_by_itself)

    def test_bad_stick_values_are_refused(self):
        sim = FakeTello(video=False, takeoff_s=0.2).start()
        self.addCleanup(sim.stop)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sock.close)
        sock.bind(('127.0.0.1', 0))
        sock.settimeout(3.0)
        target = ('127.0.0.1', sim.cmd_port)
        for text, answer in ((b'command', b'ok'), (b'takeoff', b'ok'), (b'rc 0 500 0 0', b'error'), (b'rc 1 2 3', b'error'),
                             (b'rc a b c d', b'error'), (b'battery?', b'86'), (b'fly me to the moon', b'unknown command: fly me to the moon')):
            sock.sendto(text, target)
            self.assertEqual(sock.recvfrom(100)[0], answer, text)
        self.assertEqual(sim.rc, (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
