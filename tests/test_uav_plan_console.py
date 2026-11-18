"""Planned flight in the console (ugv_console/uav_plans.py): editing and saving plans, the two-step "fly the
plan" confirmation, flying a plan with the simulated drone process, every way of stopping it, and the flight
flown by hand that is remembered as a plan.
"""
import http.client
import json
import time

from ugv_console.server import ConsoleServer

from nav_helpers import wait_for
from test_uav_console import UavCase

EXAMPLE = [dict(do='up', cm=20), dict(do='cw', deg=360), dict(do='forward', cm=40), dict(do='land')]


class PlanCase(UavCase):
    def set_plan(self, steps, speed=30):
        return self.ok('uav_plan_set', plan=dict(speed=speed, steps=steps))

    def plan(self):
        return self.app.view('uavplan')

    def named(self, command, plan_name, **more):
        """A plan command that takes the plan's name (save, load, delete)."""
        return self.app.command(command, dict(name=plan_name, **more))

    def named_ok(self, command, plan_name, **more):
        r = self.named(command, plan_name, **more)
        self.assertTrue(r['ok'], '%s -> %s' % (command, r['message']))
        return r

    def named_refused(self, command, text, plan_name, **more):
        r = self.named(command, plan_name, **more)
        self.assertFalse(r['ok'], command)
        self.assertIn(text, r['message'])
        return r

    def run_view(self):
        return self.view()['run']

    def fly_plan(self):
        prepared = self.ok('uav_plan_prepare')
        self.ok('uav_plan_confirm', nonce=prepared['nonce'])
        self.assertTrue(wait_for(lambda: (self.run_view() or {}).get('state') == 'running', 5))
        return prepared

    def plan_over(self, timeout=60):
        self.assertTrue(wait_for(lambda: self.run_view() is not None and self.run_view()['state'] != 'running', timeout),
                        self.run_view())
        return self.run_view()


class PlanEditingTests(PlanCase):
    def test_flight_mode_is_a_setting(self):
        self.assertEqual(self.view()['mode'], 'manual')
        self.ok('set_options', values=dict(uav_mode='plan'))
        self.assertEqual(self.view()['mode'], 'plan')
        self.refused('set_options', 'must be one of', values=dict(uav_mode='auto'))

    def test_the_plan_on_the_page(self):
        empty = self.plan()
        self.assertEqual((empty['steps'], empty['name'], empty['speed'], empty['dirty'], empty['saved']), ([], '', 30, False, []))
        self.set_plan(EXAMPLE)
        view = self.plan()
        self.assertEqual([step['text'] for step in view['steps']], ['Up 20 cm', 'Turn right 360°', 'Forward 40 cm', 'Land'])
        self.assertEqual((view['dirty'], view['lands'], view['has_flight']), (True, True, False))
        self.assertGreater(view['seconds'], 15)
        self.assertEqual(view['limits']['cm'], [20, 500])
        self.set_plan([dict(step) for step in view['steps']])                   # the page sends its rows back (with "text")
        self.refused('uav_plan_set', 'Step 1', plan=dict(speed=30, steps=[dict(do='up', cm=10)]))
        self.refused('uav_plan_set', 'last step', plan=dict(speed=30, steps=[dict(do='land'), dict(do='up', cm=30)]))
        self.refused('uav_plan_set', 'speed', plan=dict(speed=500, steps=EXAMPLE))
        self.refused('uav_plan_set', 'not a flight plan', plan='up 20')
        self.assertEqual(len(self.plan()['steps']), 4)                          # a refused change changes nothing
        self.set_plan(EXAMPLE[:3])
        self.assertFalse(self.plan()['lands'])

    def test_saved_plans(self):
        self.named_refused('uav_plan_save', 'empty', 'Nothing')
        self.set_plan(EXAMPLE)
        self.named_refused('uav_plan_save', '1 to 40', 'bad/name')
        self.named_refused('uav_plan_save', 'last flight by hand', 'last manual flight')
        self.assertIn('saved', self.named_ok('uav_plan_save', 'First plan')['message'])
        view = self.plan()
        self.assertEqual((view['name'], view['dirty'], view['saved']), ('First plan', False, ['First plan']))
        self.set_plan(EXAMPLE[:2])
        self.named_ok('uav_plan_save', 'Second')
        self.assertEqual(self.plan()['saved'], ['First plan', 'Second'])
        asked = self.named('uav_plan_save', 'first PLAN')                       # another plan's name: asks first
        self.assertEqual((asked['ok'], asked['need_confirm']), (False, 'uav_plan_overwrite'))
        self.set_plan(EXAMPLE[:1])                                              # changes that are not saved ...
        asked = self.named('uav_plan_load', 'First plan')
        self.assertEqual(asked['need_confirm'], 'uav_plan_discard')             # ... are not thrown away silently
        self.assertEqual(self.cmd('uav_plan_new')['need_confirm'], 'uav_plan_discard')
        self.assertEqual(len(self.plan()['steps']), 1)
        self.named_ok('uav_plan_load', 'First plan', confirm=True)
        view = self.plan()
        self.assertEqual((view['name'], len(view['steps']), view['dirty']), ('First plan', 4, False))
        self.named_refused('uav_plan_load', 'no saved plan', 'Third')
        self.assertEqual(self.named('uav_plan_delete', 'Second')['need_confirm'], 'uav_plan_delete')
        self.named_ok('uav_plan_delete', 'Second', confirm=True)
        self.assertEqual(self.plan()['saved'], ['First plan'])
        self.ok('uav_plan_new')
        self.assertEqual((self.plan()['steps'], self.plan()['name']), ([], ''))
        self.refused('uav_plan_from_flight', 'No flight by hand')
        files = sorted(p.name for p in (self.dir / 'uav_plans').iterdir())
        self.assertEqual(files, ['First plan.json', '_working.json'])

    def test_the_plan_is_still_there_after_a_restart(self):
        self.set_plan(EXAMPLE, speed=45)
        self.named_ok('uav_plan_save', 'Keep me')
        self.set_plan(EXAMPLE + [])                                             # (no change)
        self.set_plan(EXAMPLE[:3], speed=45)                                    # ... and an unsaved change
        self.app.close()
        again = self.make()
        view = again.view('uavplan')
        self.assertEqual((view['name'], view['speed'], len(view['steps']), view['saved']), ('Keep me', 45, 3, ['Keep me']))

    def test_no_plan_is_flown_without_a_drone(self):
        self.ok('set_section', section='uav')
        self.refused('uav_plan_prepare', 'Connect the drone first')
        self.refused('uav_plan_confirm', 'expired', nonce='abc')
        self.assertEqual(self.ok('uav_plan_stop')['message'], 'No plan is being flown.')
        self.assertEqual(self.view()['plan_hint'], 'Connect the drone first.')
        self.assertIsNone(self.view()['run'])


class PlannedFlightTests(PlanCase):
    def test_the_example_plan_through_the_console(self):
        app = self.app
        self.connect()
        self.ok('set_options', values=dict(uav_mode='plan'))
        self.assertEqual(self.view()['plan_hint'], 'The plan is empty: add steps first.')
        self.refused('uav_plan_prepare', 'empty')
        self.set_plan(EXAMPLE[:2] + [dict(do='photo')] + EXAMPLE[2:])
        self.named_ok('uav_plan_save', 'Example')
        self.assertEqual(self.view()['plan_hint'], '')

        prepared = self.ok('uav_plan_prepare')                                  # step 1: the whole plan is shown
        for line in ('BY ITSELF', '1.  Take off', '2.  Up 20 cm', '3.  Turn right 360°', '4.  Photo', '5.  Forward 40 cm',
                     '6.  Land', 'Battery'.lower(), 'It lands at the end', 'Esc = LAND'):
            self.assertIn(line, prepared['text'])
        self.assertEqual(app.uav.sent['takeoff'], 0)                            # asking flies nothing
        self.refused('uav_plan_confirm', 'expired', nonce='0' * 16)
        self.refused('uav_plan_confirm', 'expired', nonce=prepared['nonce'])    # a wrong code uses up the question
        prepared = self.ok('uav_plan_prepare')
        self.set_plan(EXAMPLE)                                                  # the plan is changed after the question:
        self.refused('uav_plan_confirm', 'expired', nonce=prepared['nonce'])    # ... that question is void
        self.assertEqual((app.uav.state, app.uav.sent['takeoff']), ('ready', 0))
        self.named_ok('uav_plan_load', 'Example', confirm=True)

        self.fly_plan()                                                         # step 2: a fresh question, confirmed
        self.refused('uav_plan_set', 'being flown', plan=dict(speed=30, steps=EXAMPLE))      # hands off while it flies
        self.named_refused('uav_plan_load', 'being flown', 'Example')
        self.refused('set_options', 'being flown', values=dict(uav_mode='manual'))
        self.refused('uav_plan_prepare', 'being flown')
        self.refused('uav_takeoff_prepare', 'being flown')
        seen = set()
        while self.run_view()['state'] == 'running':
            view = self.view()
            seen.add(view['run']['index'])
            if view['state'] == 'flying':
                self.assertEqual(view['pilot'], 'plan')
            time.sleep(0.05)
        run = self.run_view()
        self.assertEqual((run['state'], run['index'], run['total'], run['name']), ('done', 5, 5, 'Example'))
        self.assertIn('landed', run['message'])
        self.assertTrue({-1, 0, 1, 3} <= seen, seen)
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 5))
        self.assertTrue(wait_for(lambda: app.uav_made == 1, 12), self.log())    # filmed from take-off to landing
        self.assertTrue(wait_for(lambda: app.uav_photos == 1, 5))               # the plan's photo
        self.assertEqual(self.view()['pilot'], 'keys')
        log = self.log()
        for text in ('Flying the plan "Example" by itself: 5 steps', 'the flight plan starts', 'Plan finished: the drone has landed',
                     'Drone footage saved', 'Photo saved'):
            self.assertIn(text, log)
        self.assertNotIn('remembered as the plan', log)                         # a planned flight is not "by hand"
        self.set_plan(EXAMPLE)                                                  # changing the plan clears the old result
        self.assertIsNone(self.run_view())

    def test_stop_plan_then_the_keys_then_a_plan_from_the_air(self):
        app = self.app
        self.connect()
        self.set_plan([dict(do='wait', s=30), dict(do='land')])
        self.fly_plan()
        self.assertTrue(wait_for(lambda: self.run_view()['index'] == 0, 10))
        time.sleep(0.5)
        self.assertIn('hovers', self.ok('uav_plan_stop')['message'])
        run = self.plan_over(4)
        self.assertEqual(run['state'], 'stopped')
        self.assertIn('STOP PLAN pressed', run['message'])
        self.assertEqual((app.uav.state, self.view()['pilot']), ('flying', 'keys'))
        self.hold(['up'], 0.8)                                                  # the operator flies on by hand
        self.assertEqual(self.view()['sticks'], [0, 0, 35, 0])
        self.refused('uav_plan_prepare', 'Let go of the flight keys')
        self.hold([], 0.3)
        self.set_plan([dict(do='ccw', deg=90), dict(do='wait', s=20)])
        prepared = self.fly_plan()                                              # a plan started while hovering
        self.assertNotIn('Take off', prepared['text'])
        self.assertIn('from where it hovers now', prepared['text'])
        self.assertIn('HOVERS', prepared['text'])
        self.assertTrue(wait_for(lambda: self.run_view()['index'] == 1, 10))
        self.assertTrue(wait_for(lambda: self.view()['yaw'] == -90, 2), self.view()['yaw'])      # a quarter turn left
        self.hold(['s'], 0.4)                                                   # a flight key during the plan: take over
        run = self.plan_over(4)
        self.assertEqual(run['state'], 'stopped')
        self.assertIn('you pressed a flight key', run['message'])
        self.assertEqual(app.uav.state, 'flying')
        self.assertIn('drone is landing', self.ok('stop')['message'])
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))

    def test_esc_lands_a_plan(self):
        app = self.app
        self.connect()
        self.set_plan([dict(do='forward', cm=300), dict(do='cw', deg=180), dict(do='land')], speed=20)
        self.fly_plan()
        self.assertTrue(wait_for(lambda: self.run_view()['index'] == 0 and app.uav.state == 'flying', 12))
        time.sleep(1.0)
        self.ok('stop')                                                         # STOP / Esc
        run = self.plan_over(3)
        self.assertEqual(run['state'], 'stopped')
        self.assertIn('landing', run['message'])
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 20))
        self.assertTrue(wait_for(lambda: self.view()['height_cm'] == 0, 3))

    def test_rules_for_starting_a_plan(self):
        app = self.app
        self.connect()
        self.set_plan(EXAMPLE)
        app.is_moving = lambda: True
        self.refused('uav_plan_prepare', 'robot is moving')
        del app.is_moving
        prepared = self.ok('uav_plan_prepare')
        app.is_moving = lambda: True                                            # ... or starts to move after the question
        self.refused('uav_plan_confirm', 'robot is moving', nonce=prepared['nonce'])
        del app.is_moving
        self.ok('set_section', section='ugv')
        self.refused('uav_plan_prepare', 'UAV section')
        self.assertEqual(app.uav.sent['takeoff'], 0)

    def test_a_flight_by_hand_is_remembered_and_flown_again(self):
        app = self.app
        self.connect()
        self.take_off()
        self.hold(['w'], 1.0)
        self.hold([], 0.3)
        self.ok('uav_photo')
        self.hold(['right'], 0.8)
        self.hold([], 0.3)
        by_hand = self.view()['yaw']
        self.ok('uav_land')
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))
        self.assertTrue(wait_for(lambda: self.plan()['has_flight'], 3), self.log())
        view = self.plan()
        self.assertEqual((view['saved'], view['flights'], view['flight_name']), (['Last manual flight'], 1, 'Last manual flight'))
        self.assertGreaterEqual(view['flight']['steps'], 4)
        self.assertIn('remembered as the plan "Last manual flight"', self.log())
        self.assertTrue(wait_for(lambda: app.uav_made == 1, 12))

        self.ok('set_options', values=dict(uav_mode='plan'))
        self.assertIn('loaded', self.ok('uav_plan_from_flight')['message'])
        steps = self.plan()['steps']
        kinds = [step['do'] for step in steps]
        self.assertEqual(kinds[-1], 'land')
        self.assertEqual([step['rc'] for step in steps if step['do'] == 'sticks'], [[0, 35, 0, 0], [0, 0, 0, 35]])
        self.assertIn('photo', kinds)
        self.assertTrue(any(step['text'].startswith('Keys: forward at 35 %') for step in steps))
        prepared = self.fly_plan()
        self.assertIn('repeats your key presses', prepared['text'])
        heading = []
        while self.run_view()['state'] == 'running':
            if self.run_view()['index'] == len(steps) - 1 and not heading:
                heading.append(self.view()['yaw'])                              # where it pointed when "Land" began
            time.sleep(0.02)
        self.assertEqual(self.run_view()['state'], 'done', self.run_view())
        self.assertTrue(heading)
        turned_again = heading[0] - by_hand                                     # (the drone still stands as the flight
        self.assertLess(abs(turned_again - by_hand), 15)                        # by hand left it: the same turn once more)
        self.assertTrue(wait_for(lambda: app.uav_photos == 2, 6))               # the photo was repeated too
        self.assertEqual(self.plan()['flights'], 1)                             # the replay is not remembered as "by hand"

    def test_disconnecting_ends_what_is_left_of_a_plan(self):
        app = self.app
        self.connect()
        self.set_plan([dict(do='wait', s=20)])
        self.fly_plan()
        self.assertTrue(wait_for(lambda: app.uav.state == 'flying', 12))
        link, runner = app.uav, app.uav_run
        app.close()                                                             # the console is closed in mid-plan
        self.assertGreaterEqual(link.sent['land'], 3)
        self.assertTrue(wait_for(lambda: not runner.active, 3))
        self.assertIsNone(app.uav_run)


class PlanServerTests(PlanCase):
    def setUp(self):
        super().setUp()
        self.server = ConsoleServer(self.app, deadman_s=0.8, uav_deadman_s=4.0).start()
        self.host = '127.0.0.1:%d' % self.server.port
        time.sleep(0.4)

    def tearDown(self):
        self.server.shutdown()
        super().tearDown()

    def test_a_plan_stops_when_the_console_page_disappears(self):
        app = self.app
        self.connect()
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=8)
        conn.request('GET', '/api/events?k=' + self.server.token, headers={'Host': self.host})
        stream = conn.getresponse()
        first = None
        while first is None:
            line = stream.fp.readline().decode()
            if line.startswith('data: '):
                first = json.loads(line[6:])
        self.assertEqual(first['uavplan']['steps'], [])                         # the page is told about the plan
        self.set_plan([dict(do='wait', s=40), dict(do='land')])
        self.fly_plan()
        self.assertTrue(wait_for(lambda: app.uav.state == 'flying' and self.run_view()['index'] == 0, 12))
        time.sleep(1.2)
        self.assertEqual(self.run_view()['state'], 'running')                   # page open: the plan goes on
        stream.close(); conn.close()                                            # the window is closed
        run = self.plan_over(3)
        self.assertEqual(run['state'], 'stopped')                               # nobody is watching: it hovers ...
        self.assertIn('the console page was closed', run['message'])
        self.assertEqual(app.uav.state, 'flying')
        self.assertTrue(wait_for(lambda: app.uav.state in ('landing', 'ready'), 6))      # ... and then it is brought down
        self.assertTrue(wait_for(lambda: app.uav.state == 'ready', 12))


if __name__ == '__main__':
    import unittest
    unittest.main()
