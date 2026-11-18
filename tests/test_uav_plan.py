"""Flight plans (uav/plan.py): what a plan may contain, saved plans, and flying a plan with the simulated drone.

As everywhere in the UAV tests the "drone" is uav/sim.py on 127.0.0.1. What is shown here is that the right
commands go out in the right order and that every way of stopping a plan works; how exactly the real aircraft
flies "up 20" or "cw 360" is the aircraft's business and not tested.
"""
import json
import math
import tempfile
import time
import unittest
from pathlib import Path

from uav.plan import (FLIGHT_NAME, MAX_STEPS, WORKING, PlanRunner, PlanStore, check_name, clean_plan, clean_step,
                      describe, estimate_seconds, pieces, steps_from_flight)
from uav.tello import FLYING, LANDING, READY

from nav_helpers import wait_for
from test_uav_link import LinkCase

EXAMPLE = [dict(do='up', cm=20), dict(do='cw', deg=360), dict(do='forward', cm=40), dict(do='land')]


def plan(steps, speed=30):
    return clean_plan(dict(speed=speed, steps=steps))


class PlanTextTests(unittest.TestCase):
    def test_steps_are_checked(self):
        self.assertEqual(clean_step(dict(do='up', cm=20.0, text='Up 20 cm', extra=1)), dict(do='up', cm=20))
        self.assertEqual(clean_step(dict(do='ccw', deg=90)), dict(do='ccw', deg=90))
        self.assertEqual(clean_step(dict(do='wait', s=1.234)), dict(do='wait', s=1.23))
        self.assertEqual(clean_step(dict(do='speed', cm_s=50)), dict(do='speed', cm_s=50))
        self.assertEqual(clean_step(dict(do='sticks', rc=[0, 35, 0, -35], s=0.5)), dict(do='sticks', rc=[0, 35, 0, -35], s=0.5))
        self.assertEqual(clean_step(dict(do='photo', cm=3)), dict(do='photo'))
        for bad in (dict(do='up', cm=19), dict(do='up', cm=501), dict(do='up', cm=20.5), dict(do='up'), dict(do='up', cm='20'),
                    dict(do='up', cm=True), dict(do='up', cm=float('nan')), dict(do='cw', deg=0), dict(do='cw', deg=361),
                    dict(do='wait', s=0.1), dict(do='wait', s=121), dict(do='speed', cm_s=9), dict(do='speed', cm_s=101),
                    dict(do='flip'), dict(), 'up 20', None, dict(do='sticks', rc=[0, 0, 0, 0], s=1),
                    dict(do='sticks', rc=[0, 101, 0, 0], s=1), dict(do='sticks', rc=[0, 50, 0], s=1),
                    dict(do='sticks', rc=[0, 50.5, 0, 0], s=1), dict(do='sticks', rc=[0, 50, 0, 0], s=0.01)):
            with self.assertRaises(ValueError, msg=bad):
                clean_step(bad, 3)
        with self.assertRaises(ValueError) as caught:
            clean_step(dict(do='forward', cm=5), 4)
        self.assertIn('Step 4', str(caught.exception))                          # the message names the step
        self.assertIn('20', str(caught.exception))

    def test_a_plan_is_checked(self):
        checked = clean_plan(dict(speed=30, steps=EXAMPLE, junk=1))
        self.assertEqual(checked, dict(name='', speed=30, steps=EXAMPLE))
        self.assertEqual(clean_plan(dict(steps=[]))['speed'], 30)               # an empty plan is a plan
        with self.assertRaises(ValueError) as caught:
            clean_plan(dict(steps=[dict(do='land'), dict(do='up', cm=20)]))
        self.assertIn('last step', str(caught.exception))
        for bad in (None, [], dict(), dict(steps='up'), dict(steps=EXAMPLE, speed=5), dict(steps=EXAMPLE, speed='fast'),
                    dict(steps=[dict(do='wait', s=1)] * (MAX_STEPS + 1)), dict(steps=EXAMPLE, name='../x')):
            with self.assertRaises(ValueError, msg=bad):
                clean_plan(bad)

    def test_steps_in_words(self):
        said = [describe(step) for step in EXAMPLE + [dict(do='wait', s=2.5), dict(do='photo'), dict(do='speed', cm_s=50),
                                                      dict(do='ccw', deg=45), dict(do='sticks', rc=[-35, 35, 0, 35], s=1.8),
                                                      dict(do='sticks', rc=[0, 0, -20, 0], s=0.5)]]
        self.assertEqual(said, ['Up 20 cm', 'Turn right 360°', 'Forward 40 cm', 'Land', 'Hover 2.5 s', 'Photo',
                                'Speed 50 cm/s', 'Turn left 45°', 'Keys: left + forward + turn right at 35 % for 1.8 s',
                                'Keys: down at 20 % for 0.5 s'])

    def test_long_distances_are_flown_in_pieces(self):
        self.assertEqual(pieces(40, 30), [40])
        self.assertEqual(pieces(100, 100), [100])
        self.assertEqual(pieces(250, 100), [84, 83, 83])
        self.assertEqual(pieces(41, 10), [21, 20])
        for cm in (20, 39, 40, 41, 99, 100, 101, 333, 500):
            for speed in (10, 20, 30, 55, 100):
                parts = pieces(cm, speed)
                self.assertEqual(sum(parts), cm)
                self.assertTrue(all(20 <= part <= 100 for part in parts), (cm, speed, parts))
                self.assertTrue(all(part / speed <= 4.0 for part in parts), (cm, speed, parts))     # LAND never waits long

    def test_time_estimate(self):
        short, longer = plan(EXAMPLE), plan([dict(do='forward', cm=300)] + EXAMPLE)
        self.assertGreater(estimate_seconds(longer), estimate_seconds(short) + 8)
        self.assertGreater(estimate_seconds(short), estimate_seconds(short, takeoff=False))
        slow = plan([dict(do='forward', cm=300)], speed=10)
        fast = plan([dict(do='speed', cm_s=100), dict(do='forward', cm=300)], speed=10)
        self.assertGreater(estimate_seconds(slow), estimate_seconds(fast) + 15)
        self.assertAlmostEqual(estimate_seconds(plan([dict(do='wait', s=7)]), takeoff=False), 7.0)

    def test_names(self):
        self.assertEqual(check_name('  My  first plan-2_b '), 'My first plan-2_b')
        for bad in ('', ' ', 'a' * 41, '../x', 'a/b', 'a.b', 'con', 'NUL', '_hidden', '-x', None, 7, 'plan?'):
            with self.assertRaises(ValueError, msg=bad):
                check_name(bad)

    def test_a_flight_by_hand_becomes_steps(self):
        w, turn = (0, 35, 0, 0), (0, 0, 0, 35)
        flight = dict(seconds=9.0, by_hand=True, segments=[
            ((0, 0, 0, 0), 1.0), (w, 1.5), ((0, 0, 0, 0), 0.05), (w, 0.5),      # a blink of the key: one movement
            ((0, 0, 0, 0), 0.6), (turn, 0.9), ('photo', 0.0), (turn, 0.3),      # a photo while turning
            ((0, 0, 0, 0), 0.4), ((0, 0, 0, 0), 0.3), ((0, 20, 20, 0), 0.02),   # (too short to be anything)
            ((0, 0, 0, 0), 2.0)])                                               # a long hover before the landing
        steps = steps_from_flight(flight)
        self.assertEqual(steps, [dict(do='wait', s=1.0), dict(do='sticks', rc=[0, 35, 0, 0], s=2.0), dict(do='wait', s=0.6),
                                 dict(do='sticks', rc=[0, 0, 0, 35], s=0.9), dict(do='photo'),
                                 dict(do='sticks', rc=[0, 0, 0, 35], s=0.3), dict(do='wait', s=1.0), dict(do='land')])
        self.assertEqual(clean_plan(dict(steps=steps))['steps'], steps)          # a valid plan as it is
        self.assertEqual(steps_from_flight(dict(segments=[((0, 0, 0, 0), 5.0)])), [dict(do='wait', s=1.0), dict(do='land')])
        self.assertEqual(steps_from_flight(dict(segments=[(w, 1.0), ((0, 0, 0, 0), 0.4)]))[1:],
                         [dict(do='wait', s=0.4), dict(do='land')])             # a short last hover is kept as it was
        endless = dict(segments=[(w, 0.2), (turn, 0.2)] * 150)
        with self.assertRaises(ValueError) as caught:
            steps_from_flight(endless)
        self.assertIn('too many', str(caught.exception))


class PlanStoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='uav plan test ')
        self.addCleanup(self.tmp.cleanup)
        self.store = PlanStore(Path(self.tmp.name) / 'plans')

    def test_save_load_delete(self):
        store = self.store
        self.assertEqual(store.names(), [])                                     # (the folder does not exist yet)
        path = store.save(plan(EXAMPLE), 'Round trip')
        store.save(plan([dict(do='wait', s=1)]), 'another one')
        store.save(dict(plan(EXAMPLE), name='Round trip'), WORKING, internal=True)
        self.assertEqual(store.names(), ['another one', 'Round trip'])          # the working copy is not listed
        self.assertEqual(path.name, 'Round trip.json')
        loaded = store.load('Round trip')
        self.assertEqual(loaded, dict(name='Round trip', speed=30, steps=EXAMPLE))
        self.assertEqual(store.load(WORKING, internal=True)['name'], 'Round trip')
        store.delete('another one')
        self.assertEqual(store.names(), ['Round trip'])
        for missing in (lambda: store.load('nothing'), lambda: store.delete('nothing'), lambda: store.load('../Round trip')):
            with self.assertRaises(ValueError):
                missing()

    def test_a_damaged_or_foreign_file_is_refused(self):
        store = self.store
        store.save(plan(EXAMPLE), 'good')
        folder = store.folder
        (folder / 'broken.json').write_text('{not json', encoding='utf-8')
        (folder / 'wrong.json').write_text(json.dumps(dict(speed=30, steps=[dict(do='up', cm=9999)])), encoding='utf-8')
        (folder / 'list.json').write_text('[1, 2, 3]', encoding='utf-8')
        (folder / 'huge.json').write_text(' ' * 400000, encoding='utf-8')
        for name in ('broken', 'wrong', 'list', 'huge'):
            with self.assertRaises(ValueError, msg=name):
                store.load(name)
        self.assertEqual(store.load('good')['steps'], EXAMPLE)


class PlanCase(LinkCase):
    def ready(self, **cfg):
        link = self.connect(**cfg)
        self.assertTrue(wait_for(lambda: link.battery() is not None, 3))
        return link

    def run_plan(self, steps, speed=30, wait=True, timeout=40, **extra):
        runner = PlanRunner(self.link, plan(steps, speed), **extra).start()
        if wait:
            self.assertTrue(wait_for(lambda: not runner.active, timeout), runner.snapshot())
        return runner

    def received(self):
        return [c for c in self.sim.commands if not c.startswith('rc ') and c != 'command' and not c.endswith('?')]


class PlanFlightTests(PlanCase):
    def test_the_example_plan_is_flown_step_by_step(self):
        """After lift-off: up 20 cm, rotate 360 degrees, forward 40 cm, land."""
        link, sim = self.ready(), self.sim
        runner = self.run_plan(EXAMPLE, wait=False)
        seen, top = [], 0.0
        while runner.active:
            top = max(top, sim.z)
            snap = runner.snapshot()
            if not seen or seen[-1] != snap['index']:
                seen.append(snap['index'])
            time.sleep(0.01)
        self.assertEqual((runner.state, runner.message), ('done', 'Plan finished: the drone has landed.'))
        received = self.received()
        self.assertEqual(received[:6], ['takeoff', 'speed 30', 'up 20', 'cw 360', 'forward 40', 'land'])
        self.assertEqual(set(received[6:]) - {'land'}, set())                   # ("land" is repeated until confirmed)
        self.assertEqual(seen[0], -1)                                           # -1 = the take-off
        self.assertEqual([i for i in seen if i >= 0], [0, 1, 2, 3][-len([i for i in seen if i >= 0]):])
        self.assertAlmostEqual(top, 1.0, delta=0.03)                            # 0.8 m after take-off + 0.2
        self.assertAlmostEqual(sim.x, 0.40, delta=0.02)
        self.assertAlmostEqual(sim.y, 0.0, delta=0.02)
        self.assertEqual((sim.phase, sim.z, sim.yaw_deg()), ('ground', 0.0, 0))  # a full turn: facing as before
        self.assertEqual((link.state, link.pilot, runner.took_off), (READY, 'keys', True))
        commands = list(sim.commands)                                           # no stick values between the moves
        first, last = commands.index('up 20'), commands.index('forward 40')
        self.assertEqual([c for c in commands[first:last] if c.startswith('rc ')], [])
        self.assertFalse(link.take_flight()['by_hand'])                         # not a flight "flown by hand"

    def test_a_plan_started_in_the_air_ends_with_a_hover(self):
        link, sim = self.fly(), self.sim
        runner = self.run_plan([dict(do='forward', cm=30), dict(do='wait', s=0.4), dict(do='ccw', deg=90)])
        self.assertEqual(runner.state, 'done')
        self.assertIn('hovers', runner.message)
        self.assertEqual(self.received(), ['takeoff', 'speed 30', 'forward 30', 'ccw 90'])
        self.assertEqual((link.state, link.pilot, sim.phase, runner.took_off), (FLYING, 'keys', 'air', False))
        self.assertEqual(sim.yaw_deg(), -90)
        self.assertAlmostEqual(sim.x, 0.30, delta=0.02)
        time.sleep(0.3)
        self.assertEqual(link.last_rc, 'rc 0 0 0 0')                            # hovering, and stick values flow again
        self.hold(['w'], 0.4)                                                   # ... the keys steer again
        self.assertEqual(link.last_rc, 'rc 0 60 0 0')

    def test_speed_steps_and_long_distances(self):
        link, sim = self.fly(), self.sim
        runner = self.run_plan([dict(do='speed', cm_s=100), dict(do='forward', cm=250), dict(do='up', cm=50),
                                dict(do='back', cm=250), dict(do='left', cm=20), dict(do='right', cm=20), dict(do='down', cm=50)])
        self.assertEqual(runner.state, 'done', runner.message)
        self.assertEqual(self.received(), ['takeoff', 'speed 30', 'speed 100', 'forward 84', 'forward 83', 'forward 83',
                                           'up 50', 'back 84', 'back 83', 'back 83', 'left 20', 'right 20', 'down 50'])
        self.assertAlmostEqual(sim.x, 0.0, delta=0.03)                          # out and back
        self.assertAlmostEqual(sim.y, 0.0, delta=0.03)
        self.assertAlmostEqual(sim.z, 0.8, delta=0.03)

    def test_a_flight_key_takes_over(self):
        link, sim = self.ready(), self.sim
        runner = self.run_plan([dict(do='forward', cm=300), dict(do='cw', deg=90), dict(do='land')], speed=20, wait=False)
        self.assertTrue(wait_for(lambda: sim._move is not None, 6))
        time.sleep(0.3)
        y0 = sim.y
        self.hold(['a'], 0.5)                                                   # the operator presses a key ...
        self.assertTrue(link.takeover)
        self.assertEqual(link.sticks, (0, 0, 0, 0))                             # ... which does not steer the plan's drone
        self.assertAlmostEqual(sim.y, y0, delta=0.01)
        self.hold(['a'], 4.0)                                                   # ... and keeps holding it
        self.assertFalse(runner.active)
        self.assertEqual(runner.state, 'stopped')
        self.assertIn('you pressed a flight key', runner.message)
        self.assertIn('step 1', runner.message)
        self.assertEqual((link.state, link.pilot), (FLYING, 'keys'))
        self.assertEqual(self.received(), ['takeoff', 'speed 20', 'forward 60'])   # only the piece that was under way
        self.assertGreater(sim.y, y0 + 0.1)                                     # now the key does steer
        self.assertEqual(link.last_rc, 'rc -60 0 0 0')

    def test_stop_plan_makes_it_hover(self):
        link, sim = self.fly(), self.sim
        runner = self.run_plan([dict(do='wait', s=30), dict(do='forward', cm=100), dict(do='land')], wait=False)
        time.sleep(0.6)
        t0 = time.monotonic()
        runner.stop('STOP PLAN pressed')
        self.assertTrue(wait_for(lambda: not runner.active, 2))
        self.assertLess(time.monotonic() - t0, 0.5)                             # a hover step ends at once
        self.assertEqual(runner.state, 'stopped')
        self.assertIn('STOP PLAN pressed', runner.message)
        self.assertEqual((link.state, link.pilot, sim.phase), (FLYING, 'keys', 'air'))
        self.assertEqual(self.received(), ['takeoff', 'speed 30'])

    def test_land_in_the_middle_of_a_move(self):
        link, sim = self.ready(), self.sim
        runner = self.run_plan([dict(do='forward', cm=300), dict(do='land')], speed=20, wait=False)
        self.assertTrue(wait_for(lambda: sim._move is not None, 6))
        time.sleep(0.4)
        link.land('LAND pressed')                       # (this simulated drone refuses "land" while it is moving)
        self.assertEqual(link.state, LANDING)
        self.assertTrue(wait_for(lambda: not runner.active, 1))
        self.assertEqual(runner.state, 'stopped')
        self.assertIn('landing', runner.message)
        heights = []
        while link.state == LANDING and len(heights) < 1500:
            heights.append(sim.z)
            time.sleep(0.01)
        self.assertEqual(link.state, READY)
        self.assertLess(sim.z, 0.1)                                             # the move's "ok" was not taken for "landed"
        self.assertTrue(wait_for(lambda: sim.phase == 'ground', 2))
        self.assertAlmostEqual(sim.x, 0.60, delta=0.03)                         # it went down after the piece under way
        self.assertEqual(self.received()[:3], ['takeoff', 'speed 20', 'forward 60'])
        self.assertNotIn('forward 60', self.received()[3:])                     # ... and nothing more was flown

    def test_land_is_immediate_when_the_drone_accepts_it_while_moving(self):
        link, sim = self.ready(), self.sim
        sim.busy_refuses = False
        runner = self.run_plan([dict(do='forward', cm=300), dict(do='land')], speed=20, wait=False)
        self.assertTrue(wait_for(lambda: sim._move is not None, 6))
        time.sleep(0.5)
        x = sim.x
        link.land('LAND pressed')
        self.assertTrue(wait_for(lambda: link.state == READY, 6))
        self.assertLess(sim.x - x, 0.05)                                        # it stopped where it was
        self.assertEqual(runner.state, 'stopped')

    def test_the_drone_refuses_a_step(self):
        link, sim = self.fly(), self.sim
        sim.fail_moves.append('error No valid imu')
        runner = self.run_plan([dict(do='up', cm=30), dict(do='forward', cm=50), dict(do='land')])
        self.assertEqual(runner.state, 'failed')
        self.assertIn('error No valid imu', runner.message)
        self.assertIn('step 1 (Up 30 cm)', runner.message)
        self.assertEqual(self.received(), ['takeoff', 'speed 30', 'up 30'])      # nothing after the refused step
        self.assertEqual((link.state, link.pilot, sim.phase), (FLYING, 'keys', 'air'))
        time.sleep(0.3)
        self.assertEqual(link.last_rc, 'rc 0 0 0 0')

    def test_a_stick_complaint_is_tried_once_more(self):
        link, sim = self.fly(), self.sim
        sim.fail_moves.append('error Not joystick')
        runner = self.run_plan([dict(do='up', cm=30)])
        self.assertEqual(runner.state, 'done', runner.message)
        self.assertEqual(self.received(), ['takeoff', 'speed 30', 'up 30', 'up 30'])
        sim.fail_moves += ['error Not joystick', 'error Not joystick']          # twice in a row: the plan gives up
        runner = self.run_plan([dict(do='down', cm=30)])
        self.assertEqual(runner.state, 'failed')

    def test_no_answer_to_a_step(self):
        link, sim = self.fly(), self.sim
        sim.drop_answers.add('up')
        t0 = time.monotonic()
        runner = self.run_plan([dict(do='up', cm=20), dict(do='land')], speed=100)
        self.assertEqual(runner.state, 'failed')
        self.assertIn('No answer', runner.message)
        self.assertLess(time.monotonic() - t0, 9.0)
        self.assertEqual((link.state, link.pilot), (FLYING, 'keys'))
        self.assertNotIn('land', self.received())

    def test_a_low_battery_ends_the_plan_with_a_landing(self):
        link, sim = self.fly(), self.sim
        runner = self.run_plan([dict(do='wait', s=30), dict(do='forward', cm=100)], wait=False)
        time.sleep(0.5)
        sim.battery = 14.0
        self.assertTrue(wait_for(lambda: not runner.active, 4))
        self.assertEqual(runner.state, 'failed')
        self.assertRegex(runner.message, r'Battery 1[34] %')
        self.assertTrue(wait_for(lambda: link.state == READY, 8))
        self.assertEqual(sim.phase, 'ground')

    def test_a_refused_take_off_flies_nothing(self):
        link, sim = self.ready(), self.sim
        sim.refuse_takeoff = True
        runner = self.run_plan(EXAMPLE)
        self.assertEqual(runner.state, 'failed')
        self.assertIn('did not take off', runner.message)
        self.assertEqual(self.received(), ['takeoff'])
        self.assertEqual((link.state, link.pilot, sim.phase), (READY, 'keys', 'ground'))

    def test_a_silent_drone_ends_the_plan(self):
        link = self.fly()
        runner = self.run_plan([dict(do='wait', s=30), dict(do='forward', cm=100)], wait=False)
        time.sleep(0.4)
        self.sim.stop()                                                         # out of range
        self.assertTrue(wait_for(lambda: not runner.active, 4))
        self.assertEqual(runner.state, 'failed')
        self.assertIn('Nothing is heard', runner.message)
        self.assertTrue(wait_for(lambda: link.state == LANDING, 4))             # and the link lands it (as always)

    def test_a_plan_is_flown_without_status_messages(self):
        """A blocked status port: the drone is quiet while it does a movement, which is not "silence"."""
        self.sim.mute_status = True
        link, sim = self.connect(), self.sim
        self.assertTrue(wait_for(lambda: link.battery() is not None, 4))
        runner = self.run_plan([dict(do='cw', deg=360), dict(do='forward', cm=40), dict(do='land')])     # the turn takes 3.6 s
        self.assertEqual(runner.state, 'done', runner.message)
        self.assertEqual((sim.phase, sim.yaw_deg()), ('ground', 0))
        self.assertNotIn('Nothing heard', self.events())


class RememberedFlightTests(PlanCase):
    def test_a_flight_by_hand_is_remembered_and_flown_again(self):
        link, sim = self.fly(), self.sim
        self.assertIsNone(link.take_flight())                                   # still in the air
        for keys, seconds in ((['w'], 1.0), ([], 0.4), (['right'], 0.8), (['w', 'up'], 0.7), ([], 0.3), (['a'], 0.6)):
            self.hold(keys, seconds, percent=50)
            if keys == ['right']:
                link.log_action('photo')
        self.hold([], 0.3)
        by_hand = (sim.x, sim.y, sim.z, sim.yaw_deg())
        link.land()
        self.assertTrue(wait_for(lambda: link.state == READY, 6))
        flight = link.take_flight()
        self.assertTrue(flight['by_hand'])
        self.assertIsNone(link.take_flight())                                   # handed out once
        steps = steps_from_flight(flight)
        kinds = [step['do'] for step in steps]
        self.assertEqual(kinds, ['sticks', 'wait', 'sticks', 'photo', 'sticks', 'wait', 'sticks', 'wait', 'land'])
        self.assertEqual([step['rc'] for step in steps if step['do'] == 'sticks'],
                         [[0, 50, 0, 0], [0, 0, 0, 50], [0, 50, 50, 0], [-50, 0, 0, 0]])
        self.assertAlmostEqual(steps[0]['s'], 1.0, delta=0.15)
        sim.x = sim.y = sim.yaw = 0.0                                           # back on the start point
        photos = []
        runner = self.run_plan(steps, wait=False, take_photo=lambda: photos.append(1))
        again = None
        while runner.active:
            if runner.index == len(steps) - 1 and again is None:
                again = (sim.x, sim.y, sim.z, sim.yaw_deg())                    # where it was when "Land" began
            time.sleep(0.005)
        self.assertEqual(runner.state, 'done', runner.message)
        self.assertEqual(len(photos), 1)
        self.assertLess(math.hypot(again[0] - by_hand[0], again[1] - by_hand[1]), 0.25)
        self.assertLess(abs(again[2] - by_hand[2]), 0.15)
        self.assertLess(abs(again[3] - by_hand[3]), 15)
        self.assertEqual(sim.phase, 'ground')
        self.assertFalse(link.take_flight()['by_hand'])                         # the replay itself is not a flight by hand

    def test_a_silent_page_is_remembered_as_a_hover(self):
        link = self.fly()
        link.set_keys(['w'], 40)                                                # one key report, then the page is quiet:
        time.sleep(1.0)                                                         # the link hovers after 0.3 s
        link.land()
        self.assertTrue(wait_for(lambda: link.state == READY, 6))
        flight = link.take_flight()
        moved = [(what, seconds) for what, seconds in flight['segments'] if what == (0, 40, 0, 0)]
        self.assertEqual(len(moved), 1)
        self.assertAlmostEqual(moved[0][1], 0.3, delta=0.12)                    # what the drone was really sent
        self.assertEqual(FLIGHT_NAME, 'Last manual flight')

    def test_a_flight_ended_by_emergency_is_closed_too(self):
        link = self.fly()
        self.hold(['up'], 0.4)
        link.emergency()
        flight = link.take_flight()
        self.assertTrue(flight['by_hand'])
        self.assertEqual([what for what, _ in flight['segments']][:2], [(0, 0, 0, 0), (0, 0, 60, 0)])


if __name__ == '__main__':
    unittest.main()
