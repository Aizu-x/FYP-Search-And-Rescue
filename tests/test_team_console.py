"""The team simulator as the operator uses it (ugv_console/team_console.py): the console's commands and what
the page is shown, without a browser. Simulated robots only; with one robot the console is what it always was.
"""
import http.client
import importlib.util
import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from ugv_console.app import ConsoleApp
from ugv_console.server import ConsoleServer
from ugv_nav.navigator import RUNNING, STOPPED

from nav_helpers import wait_for

HAVE_CV2 = importlib.util.find_spec('cv2') is not None
ROBOT = dict(radius=0.143, margin=0.02, verified=True)
CROSS = ([(1.2, 0.8, 0), (1.9, 0.3, 90)], [(2.6, 0.8), (1.9, 1.5)])         # ENV1: the routes cross
APART = ([(1.2, 0.3, 0), (1.2, 1.0, 0)], [(2.6, 0.3), (2.6, 1.0)])          # ENV1: side by side


class TeamCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv team test ')
        self.dir = Path(self.tmp.name)
        self.apps = []
        self.app = self.make()

    def make(self, settings=False):
        app = ConsoleApp(settings_path=self.dir / 'console_settings.json' if settings else None,
                         edit_dir=self.dir / 'edited', export_dir=self.dir / 'exports')
        app.opts['record_dir'] = str(self.dir / 'mission videos')
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

    def refused(self, name, part, **args):
        r = self.cmd(name, **args)
        self.assertFalse(r['ok'], name)
        self.assertIn(part, r['message'])
        return r

    def choose(self, robots, **options):
        self.ok('set_mode', mode='sim')
        self.ok('set_options', values=dict(ROBOT, sim_robots=robots, **options))

    def team(self, scenario, plan=True, **options):
        """A started team simulator with its robots placed and their goals set (and planned)."""
        starts, goals = scenario
        self.choose(len(starts), **options)
        for number, (x, y, heading) in enumerate(starts, 1):
            self.ok('team_place', robot=number, x=x, y=y, heading=heading)
        self.ok('team_start')
        for number, (x, y) in enumerate(goals, 1):
            self.ok('team_goal', robot=number, x=x, y=y)
        if plan:
            self.ok('team_plan')
        return self.app.team

    def authorize(self):
        r = self.ok('team_run_prepare')
        return r, self.cmd('team_run_confirm', nonce=r['nonce'])

    def view(self):
        return self.app.view('team')

    def log(self):
        return [line['text'] for line in self.app.log_lines()]


class ChoosingTests(TeamCase):
    def test_one_robot_is_the_simulator_it_always_was(self):
        app = self.app
        self.assertEqual(app.opts['sim_robots'], 1)
        self.assertEqual(self.view(), dict(selected=False, size=1, running=False))
        self.ok('set_mode', mode='sim')
        self.ok('set_options', values=ROBOT)
        self.ok('set_start', x=1.215, y=0.405, heading=0)
        self.ok('connect')                                          # the single simulator, as before
        self.assertTrue(wait_for(lambda: app.client.link_ok(), 6))
        self.assertIsNone(app.team)
        self.assertNotIn('team', app.fast())
        self.refused('team_start', 'Choose Simulator mode and 2 to 4 robots')
        self.refused('team_place', 'Choose Simulator mode and 2 to 4 robots', robot=1, x=1.2, y=0.5)

    def test_choosing_more_robots_swaps_the_simulators(self):
        app = self.app
        self.ok('set_mode', mode='sim')
        self.ok('set_options', values=ROBOT)
        self.ok('connect')
        self.assertIsNotNone(app.nav)
        r = self.ok('set_options', values=dict(sim_robots=3))
        self.assertIn('3 robots', r['message'])
        self.assertIsNone(app.nav)                                  # the single simulator ended
        view = self.view()
        self.assertEqual((view['selected'], view['size'], view['running'], view['phase']), (True, 3, False, 'setup'))
        self.assertEqual([r['label'] for r in view['robots']], ['R1', 'R2', 'R3'])
        self.assertEqual(len(set(view['colours'])), 3)              # three different colours
        self.refused('connect', 'set to 3 robots')
        self.ok('team_start')
        self.assertEqual(len(app.team.robots), 3)
        self.assertEqual(len(app.fast()['team']), 3)
        self.ok('set_options', values=dict(sim_robots=1))
        self.assertIsNone(app.team)                                 # and the team ended
        self.assertFalse(self.view()['selected'])
        self.ok('connect')

    def test_more_than_four_robots_cannot_be_chosen(self):
        self.ok('set_mode', mode='sim')
        for bad in (5, 0, 2.5, 'many'):
            r = self.cmd('set_options', values=dict(sim_robots=bad))
            self.assertFalse(r['ok'], bad)
        self.assertEqual(self.app.opts['sim_robots'], 1)
        self.choose(4)
        self.refused('team_goal', 'out of range', robot=5, x=1.5, y=0.5)
        self.choose(2)
        self.refused('team_goal', 'There is no R3', robot=3, x=1.5, y=0.5)

    def test_default_starting_places_work_in_every_arena(self):
        self.choose(4)
        for env in ('ENV1', 'ENV2', 'ENV3'):
            self.ok('set_env', env=env, confirm=True)
            view = self.view()
            starts = view['starts']
            self.assertEqual(len(starts), 4)
            for k in range(4):
                for j in range(k):
                    self.assertGreater(math.dist(starts[k][:2], starts[j][:2]), 0.55, env)  # well apart
            self.ok('team_start')                                   # every place is one a robot can stand on
            self.assertEqual(len(self.app.team.robots), 4, env)
            self.ok('team_end')

    def test_the_number_of_robots_is_remembered(self):
        app = self.make(settings=True)
        app.command('set_mode', dict(mode='sim'))
        app.command('set_options', dict(values=dict(sim_robots=3)))
        app.close()
        again = self.make(settings=True)
        self.assertEqual((again.opts['mode'], again.opts['sim_robots']), ('sim', 3))
        self.assertTrue(again.view('team')['selected'])
        self.assertIsNone(again.team)                               # nothing starts by itself


class MissionTests(TeamCase):
    def test_place_plan_authorize_drive(self):
        team = self.team(CROSS, plan=False, team_scan=False)
        view = self.view()
        self.assertEqual((view['phase'], view['running']), ('ready', True))
        self.assertEqual(view['starts'][1], [1.9, 0.3, 90.0])
        self.assertEqual(view['goals'], [[2.6, 0.8], [1.9, 1.5]])
        self.assertFalse(view['run']['enabled'])
        self.assertEqual(view['run']['hint'], 'Plan the team mission first.')
        self.refused('team_run_prepare', 'Plan the team mission first')
        r = self.ok('team_plan')
        self.assertIn('R2 after R1', r['message'])
        view = self.view()
        self.assertEqual(view['phase'], 'planned')
        self.assertTrue(view['run']['enabled'])
        self.assertEqual(view['waves'][0], ['r1_clear_r2', 'r1_arrive'])
        self.assertEqual([(e['first'], e['then']) for e in view['edges']], [(1, 2)])
        self.assertFalse(self.app.is_moving())
        self.assertEqual([robot.nav.state for robot in team.robots], ['PLANNED', 'PLANNED'])     # nothing moves yet

        self.refused('team_run_confirm', 'no longer valid', nonce='0' * 16)      # no question was asked
        self.assertFalse(self.app.is_moving())
        asked, r = self.authorize()
        self.assertEqual((asked['title'], asked['confirm_label']), ('Start the team', 'Authorize + start'))
        self.assertIn('R1 starts at once', asked['text'])
        self.assertIn('R2 starts when R1 is out of the way', asked['text'])
        self.assertTrue(r['ok'], r['message'])
        self.assertTrue(self.app.is_moving())
        self.assertEqual(self.view()['phase'], 'running')
        self.refused('team_run_confirm', 'no longer valid', nonce=asked['nonce'])            # a code works once
        self.refused('team_goal', 'stop it before changing a goal', robot=1, x=2.5, y=0.8)
        self.refused('team_place', 'Stop the team before moving a robot', robot=1, x=1.3, y=0.8)
        self.refused('set_options', 'a robot is moving', values=dict(sim_robots=3))
        self.assertTrue(wait_for(lambda: self.app.team.phase != 'running', 60, 0.05))
        view = self.view()
        self.assertEqual(view['phase'], 'done', view['message'])
        self.assertEqual({task['state'] for task in view['tasks']}, {'SUCCEEDED'})
        self.assertFalse(self.app.is_moving())
        log = self.log()
        self.assertIn('Team mission AUTHORIZED by the operator.', log)
        self.assertLess(log.index("R1 is clear of R2's route."), log.index('R2 starts (R1 is out of its way).'))
        self.assertEqual(view['report']['robots'], 2)

    def test_the_authorization_belongs_to_one_plan(self):
        self.team(APART, team_scan=False)
        asked = self.ok('team_run_prepare')
        self.ok('team_goal', robot=2, x=2.5, y=1.0)                 # the plan is gone with the old goal
        self.refused('team_run_confirm', 'no longer valid', nonce=asked['nonce'])
        self.assertFalse(self.app.is_moving())
        self.ok('team_plan')
        asked = self.ok('team_run_prepare')
        with mock.patch('ugv_console.team_console.time.monotonic', return_value=time.monotonic() + 61):
            self.refused('team_run_confirm', 'no longer valid', nonce=asked['nonce'])        # too late
        asked = self.ok('team_run_prepare')
        self.refused('team_run_confirm', 'no longer valid', nonce=asked['nonce'][:-1] + 'x')
        self.refused('team_run_confirm', 'no longer valid', nonce=asked['nonce'])            # used up by the wrong try
        self.assertFalse(self.app.is_moving())

    def test_stop_stops_the_team(self):
        team = self.team(APART, team_scan=False)
        self.assertTrue(self.authorize()[1]['ok'])
        self.assertTrue(wait_for(lambda: all(robot.nav.state == RUNNING for robot in team.robots), 3))
        r = self.cmd('stop')
        self.assertIn('every robot of the team stopped', r['message'])
        self.assertEqual([robot.nav.state for robot in team.robots], [STOPPED, STOPPED])
        self.assertFalse(self.app.is_moving())
        self.assertEqual(self.view()['phase'], 'stopped')
        self.refused('team_run_prepare', 'Plan the team mission first')
        self.ok('team_plan')                                        # after a new plan it can be authorized again
        self.assertTrue(self.authorize()[1]['ok'])

    def test_a_closed_page_stops_the_team(self):
        team = self.team(APART, team_scan=False)
        server = ConsoleServer(self.app, deadman_s=0.8).start()
        self.addCleanup(server.shutdown)
        conn = http.client.HTTPConnection('127.0.0.1', server.port, timeout=5)
        conn.request('GET', '/api/events?k=' + server.token)
        response = conn.getresponse()
        response.read(64)                                           # a console page is open
        self.assertTrue(wait_for(lambda: server.clients() == 1, 3))
        self.assertTrue(self.authorize()[1]['ok'])
        time.sleep(1.2)
        self.assertEqual(team.phase, 'running')                     # page open: the team keeps driving
        response.close(); conn.close()                              # window closed / browser crashed
        self.assertTrue(wait_for(lambda: team.phase == 'stopped', 5), team.phase)
        self.assertEqual([robot.nav.state for robot in team.robots], [STOPPED, STOPPED])
        self.assertFalse(self.app.is_moving())

    def test_one_machine_at_a_time(self):
        self.team(APART, team_scan=False)
        with mock.patch.object(self.app, 'uav_airborne', return_value=True):
            self.refused('team_run_prepare', 'The drone is in the air')
            self.assertFalse(self.view()['run']['enabled'])
        asked = self.ok('team_run_prepare')
        with mock.patch.object(self.app, 'uav_airborne', return_value=True):
            self.refused('team_run_confirm', 'The drone is in the air', nonce=asked['nonce'])
        self.assertFalse(self.app.is_moving())
        self.assertTrue(self.authorize()[1]['ok'])
        self.assertTrue(self.app.is_moving())                       # ... which is what keeps the drone on the ground
        self.refused('set_section', 'The robot is moving', section='uav')

    def test_a_setup_that_cannot_be_driven_is_refused_with_the_reason(self):
        self.team(([(1.2, 0.5, 0), (2.4, 0.5, 180)], [(2.4, 0.5), (1.2, 0.5)]), plan=False)      # swap places
        r = self.refused('team_plan', 'R1 and R2 block each other')
        self.assertIn('Move a start or a goal', r['message'])
        self.assertEqual(self.view()['phase'], 'ready')
        self.refused('team_run_prepare', 'Plan the team mission first')

    def test_goals_and_places_are_checked(self):
        self.team(APART, plan=False)
        self.refused('team_goal', 'outside the map', robot=1, x=9.0, y=0.3)
        self.refused('team_goal', 'inside a wall', robot=1, x=0.02, y=0.02)
        self.refused('team_place', 'too close together', robot=2, x=1.3, y=0.4)
        self.refused('team_place', 'R2 cannot stand there', robot=2, x=2.9, y=1.0)
        self.assertEqual(self.view()['starts'][1], [1.2, 1.0, 0.0])              # unchanged
        self.ok('team_clear_goals')
        self.assertEqual(self.view()['goals'], [None, None])
        self.refused('team_plan', 'Set a goal for at least one robot first')
        self.ok('team_place', robot=2, x=1.2, y=1.5)                             # heading kept when not given
        self.assertEqual(self.view()['starts'][1], [1.2, 1.5, 0.0])
        self.assertLess(math.dist(self.app.team.robots[1].backend.truth_pose()[:2], (1.2, 1.5)), 0.001)
        self.ok('team_reset')

    def test_a_robot_whose_goal_is_removed_stays_where_it_is(self):
        team = self.team(APART, team_scan=False)
        r = self.ok('team_clear_goals', robot=2)
        self.assertIn('R2 has no goal: it stays where it is', r['message'])
        view = self.view()
        self.assertEqual((view['goals'], view['phase']), ([[2.6, 0.3], None], 'ready'))      # the old plan is gone
        self.refused('team_clear_goals', 'There is no R3', robot=3)
        r = self.ok('team_plan')
        self.assertIn('R2 stays where it is (no goal)', r['message'])
        self.assertEqual([task['name'] for task in self.view()['tasks']], ['r1_arrive', 'r1_rescue', 'team_report'])
        self.assertEqual(self.view()['robots'][1]['task'], 'stays here')
        parked = team.robots[1].backend.truth_pose()
        self.assertTrue(self.authorize()[1]['ok'])
        self.assertTrue(wait_for(lambda: team.phase != 'running', 40, 0.05))
        self.assertEqual(self.view()['phase'], 'done', self.view()['message'])
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], parked[:2]), 0.002)

    def test_driving_up_to_survivors_can_be_switched_off(self):
        self.team(APART, team_scan=False)
        self.assertTrue(self.app.opts['team_approach'])                         # on unless switched off
        view = self.view()
        self.assertTrue(view['approach'])
        self.assertIn('r2_rescue', [task['name'] for task in view['tasks']])
        asked = self.ok('team_run_prepare')
        self.assertIn('A robot that has seen a survivor then drives up to it by itself', asked['text'])
        r = self.ok('set_options', values=dict(team_approach=False))
        self.assertIn('do not drive to them', r['message'])
        self.assertEqual(self.view()['phase'], 'ready')                         # another mission: plan again
        self.refused('team_run_confirm', 'Plan the team mission first', nonce=asked['nonce'])
        self.assertFalse(self.app.is_moving())
        self.ok('team_plan')
        view = self.view()
        self.assertFalse(view['approach'])
        self.assertEqual([task['name'] for task in view['tasks']], ['r1_arrive', 'r2_arrive', 'team_report'])
        self.assertNotIn('survivor', self.ok('team_run_prepare')['text'])
        self.assertTrue(self.authorize()[1]['ok'])
        self.refused('set_options', 'the team is driving', values=dict(team_approach=True))

    def test_the_calibration_of_the_real_robot_is_not_for_the_simulated_team(self):
        # "X scale" / "Y scale" correct the real robot's wheels. A simulated robot that believed them would think
        # it is somewhere it is not, and drive into a wall (which is what a team run of the operator once did).
        self.ok('apply_scale', x=0.9, y=0.965)                      # (the calibration the operator's robot has)
        team = self.team(APART, team_scan=False)
        self.assertEqual((self.app.opts['scale_x'], self.app.opts['scale_y']), (0.9, 0.965))
        for robot in team.robots:
            self.assertEqual((robot.nav.estimator.scale_x, robot.nav.estimator.scale_y), (1.0, 1.0))
        self.assertTrue(self.authorize()[1]['ok'])
        worst = 0.0
        while team.phase == 'running':
            for robot in team.robots:
                worst = max(worst, math.dist(robot.backend.truth_pose()[:2], robot.position()))
            time.sleep(0.05)
        self.assertEqual(self.view()['phase'], 'done', self.view()['message'])
        self.assertLess(worst, 0.05)                                # where a robot thinks it is, is where it is
        for robot, goal in zip(team.robots, APART[1]):
            self.assertLess(math.dist(robot.backend.truth_pose()[:2], goal), 0.12)
        self.ok('apply_scale', x=0.95, y=1.0)                       # changed while a team exists: still not theirs
        for robot in team.robots:
            self.assertEqual(robot.nav.estimator.scale_x, 1.0)


class HousekeepingTests(TeamCase):
    def test_leaving_the_simulator_ends_the_team(self):
        self.team(APART)
        self.ok('set_mode', mode='offline')
        self.assertIsNone(self.app.team)
        self.assertEqual(self.view(), dict(selected=False, size=1, running=False))
        self.refused('team_plan', 'Start the team simulator first')

    def test_another_arena_ends_the_team_and_gives_new_places(self):
        self.team(APART)
        before = self.view()['starts']
        self.ok('set_env', env='ENV3', confirm=True)
        self.app.tick_once()
        self.assertIsNone(self.app.team)
        view = self.view()
        self.assertEqual((view['running'], view['goals']), (False, [None, None]))
        self.assertNotEqual(view['starts'], before)

    def test_a_changed_map_is_planned_on(self):
        team = self.team(APART)
        self.assertEqual(self.view()['phase'], 'planned')
        cell = self.app.grid.world_to_cell((2.0, 1.6))
        self.app.grid = self.app.grid.edited(cell, True)            # the operator painted a wall cell
        self.app.tick_once()
        self.assertIs(team.grid, self.app.grid)
        self.assertEqual(self.view()['phase'], 'ready')             # the old plan is gone
        self.ok('team_plan')

    def test_the_team_drives_in_the_map_the_operator_sees(self):
        team = self.team(APART, plan=False)
        self.assertEqual(self.view()['env'], 'ENV1')
        self.assertIs(team.world, self.app.grid)                    # not the data set's hidden "true" arena
        self.assertIsNot(team.world, self.app.gt)
        cell = self.app.grid.world_to_cell((2.0, 1.6))
        self.assertFalse(team.robots[0].backend.world.cells[cell])
        self.app.grid = self.app.grid.edited(cell, True)            # a wall the operator draws ...
        self.app.tick_once()
        for robot in team.robots:                                   # ... is a wall for the simulated robots
            self.assertIs(robot.backend.world, self.app.grid)
            self.assertTrue(robot.backend.world.cells[cell])
            self.assertTrue(robot.backend.checker.grid.cells[cell])

    def test_closing_the_console_ends_the_team(self):
        team = self.team(APART, team_scan=False)
        self.assertTrue(self.authorize()[1]['ok'])
        robots = list(team.robots)
        self.app.close()
        self.assertIsNone(self.app.team)
        self.assertTrue(all(robot.nav.state != RUNNING for robot in robots))


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class CameraTests(TeamCase):
    def test_blocks_and_one_camera_per_robot(self):
        self.team(([(1.2, 0.5, 0), (1.2, 1.3, 0)], [(2.6, 0.5), (2.0, 1.3)]), plan=False)
        self.assertTrue(self.view()['camera'])
        self.ok('set_options', values=dict(sim_block_colour='green'))
        r = self.ok('team_block', x=1.8, y=1.3)                     # 60 cm in front of R2
        self.assertIn('green block was placed', r['message'])
        self.assertTrue(wait_for(lambda: self.app.team_frame(1, timeout=0.3) is not None, 6))
        self.assertTrue(wait_for(lambda: self.view()['findings'], 6))
        finding = self.view()['findings'][0]
        self.assertEqual(finding['by'][0], 'R2')
        self.assertEqual(self.view()['blocks'], [[1.8, 1.3, 'green_obstacle']])
        self.assertIsNone(self.app.team_frame(3))                   # there is no R3
        self.assertTrue(any('R2 sees a survivor' in text for text in self.log()))
        self.ok('team_clear_findings')
        self.assertTrue(wait_for(lambda: self.view()['findings'], 6))            # still in view: found again
        self.ok('team_goal', robot=1, x=2.6, y=0.5)
        self.ok('team_plan')
        self.assertTrue(self.authorize()[1]['ok'])
        self.assertEqual(len(self.view()['findings']), 1)                        # a mission does not empty the list

    def test_the_page_gets_each_robots_picture(self):
        self.team(APART, plan=False)
        server = ConsoleServer(self.app).start()
        self.addCleanup(server.shutdown)

        def get(path, token=True):
            conn = http.client.HTTPConnection('127.0.0.1', server.port, timeout=5)
            conn.request('GET', path, headers={'X-Console-Token': server.token} if token else {})
            response = conn.getresponse()
            data = response.read()
            conn.close()
            return response, data

        self.assertTrue(wait_for(lambda: get('/api/team_frame?robot=2')[0].status == 200, 8, 0.2))
        response, data = get('/api/team_frame?robot=2')
        self.assertEqual(response.getheader('Content-Type'), 'image/jpeg')
        self.assertEqual(data[:2], b'\xff\xd8')
        meta = json.loads(response.getheader('X-Frame'))
        self.assertEqual((meta['robot'], meta['w'], meta['h'], meta['boxes']), (2, 640, 480, []))
        self.assertEqual(get('/api/team_frame?robot=2&after=%d' % 10 ** 9)[0].status, 200)      # another id: newest
        self.assertEqual(get('/api/team_frame?robot=4')[0].status, 204)                         # no such robot
        self.assertEqual(get('/api/team_frame?robot=x')[0].status, 404)
        self.assertEqual(get('/api/team_frame?robot=1', token=False)[0].status, 401)


if __name__ == '__main__':
    unittest.main()
