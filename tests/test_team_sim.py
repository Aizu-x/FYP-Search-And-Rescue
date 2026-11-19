"""A team of simulated robots driving in the order of its task graph (ugv_nav/team_sim.py).

Every robot is the complete single-robot stack (real bridge server, edge client, navigator, simulated camera)
on 127.0.0.1; nothing here can reach a real robot. What these tests show is the team logic: who waits for
whom, what happens when a robot fails or the operator stops, and that the simulated robots share one floor.
"""
import importlib.util
import math
import threading
import time
import unittest

from ugv_nav.controller import ControllerConfig
from ugv_nav.localization import LocalizationConfig
from ugv_nav.navigator import ARRIVED, RUNNING, STOPPED, NavigatorConfig
from ugv_nav.safety_monitor import SafetyConfig
from ugv_nav.team_plan import BLOCKED, CANCELLED, FAILED, SUCCEEDED, TeamError
# (a robot that has stopped for good: its own tasks FAILED; a robot that then cannot leave: its tasks BLOCKED)
from ugv_nav.team_sim import COLOURS, REPLANS, TeamSim, TeamSimBackend, with_discs
from ugv_planner.config import Footprint, PlannerConfig
from ugv_planner.grid import load_environment

from nav_helpers import wait_for

HAVE_CV2 = importlib.util.find_spec('cv2') is not None

# ENV1: the big open part of the arena is roughly x 1.0-2.9 m, y 0.1-1.9 m
CROSS = ([(1.2, 0.8, 0.0), (1.9, 0.25, math.pi / 2)], {1: (2.6, 0.8), 2: (1.9, 1.5)})      # the routes cross
APART = ([(1.2, 0.3, 0.0), (1.2, 1.0, 0.0)], {1: (2.6, 0.3), 2: (2.6, 1.0)})                # side by side
FOUR = ([(1.2, 0.3, 0.0), (2.7, 0.3, math.pi), (1.2, 1.7, 0.0), (2.55, 1.45, math.pi)],
        {1: (2.7, 1.0), 2: (1.2, 1.2), 3: (2.2, 0.6), 4: (1.5, 0.7)})


class TeamCase(unittest.TestCase):
    def setUp(self):
        self.occ, self.gt = load_environment('ENV1')
        self.events = []
        self.teams = []

    def tearDown(self):
        for team in self.teams:
            team.close()

    def team(self, starts, **options):
        planner = PlannerConfig(8, Footprint(math.hypot(0.08, 0.07), 0.02, True), True, clearance_weight=4.0)
        nav = NavigatorConfig(controller=ControllerConfig(max_v=0.2, max_w=1.0, min_w=0.2),
                              localization=LocalizationConfig(drift_per_m=0.0, drift_per_rad_m=0.0, max_sigma_m=0.2),
                              safety=SafetyConfig(0.08, 0.07))
        options.setdefault('camera', False)
        team = TeamSim(self.gt, self.occ, planner, nav, starts,
                       say=lambda text, level='info': self.events.append(text), **options)
        self.teams.append(team)
        return team

    def started(self, scenario, scan=False, **options):
        starts, goals = scenario
        team = self.team(starts, **options)
        team.plan(goals, scan=scan)
        ok, text = team.run()
        self.assertTrue(ok, text)
        return team

    def finish(self, team, timeout):
        self.assertTrue(wait_for(lambda: team.phase != 'running', timeout, 0.05), team.snapshot()['tasks'])

    @staticmethod
    def truth(team):
        return [robot.backend.truth_pose() for robot in team.robots]

    def states(self, team):
        return {task['name']: task['state'] for task in team.snapshot()['tasks']}

    def said(self, part):
        return next((k for k, text in enumerate(self.events) if part in text), None)


class OrderTests(TeamCase):
    def test_crossing_routes_the_second_robot_waits_until_the_first_is_clear(self):
        team = self.started(CROSS)
        graph, first, second = team.graph, team.robots[0], team.robots[1]
        self.assertEqual(first.nav.state, RUNNING)                  # R1 was released at once
        self.assertNotEqual(second.nav.state, RUNNING)              # R2 has to wait for r1_clear_r2
        self.assertEqual(team.snapshot()['robots'][1]['task'], 'waits for R1')
        waiting_place = second.backend.truth_pose()
        closest, moved_early, r1_was_driving = 9.0, False, False
        deadline = time.monotonic() + 60
        while team.phase == 'running' and time.monotonic() < deadline:
            a, b = self.truth(team)
            closest = min(closest, math.dist(a[:2], b[:2]))
            if graph.state['r1_clear_r2'] != SUCCEEDED:
                moved_early = moved_early or math.dist(b[:2], waiting_place[:2]) > 0.01
            elif second.nav.state == RUNNING and first.nav.state == RUNNING:
                r1_was_driving = True                               # both drive: R2 did not wait for R1's arrival
            time.sleep(0.02)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertFalse(moved_early, 'R2 moved before R1 was clear of its route')
        self.assertTrue(r1_was_driving, 'R2 only started after R1 had arrived')
        self.assertGreater(closest, 2 * team.body + 0.05)           # they never came near each other
        self.assertLess(self.said("R1 is clear of R2's route"), self.said('R2 starts (R1 is out of its way)'))
        self.assertEqual(set(self.states(team).values()), {SUCCEEDED})
        for robot in team.robots:
            self.assertLess(math.dist(robot.backend.truth_pose()[:2], robot.goal), 0.12)
            self.assertFalse(robot.backend.collided)
        report = team.snapshot()['report']
        self.assertEqual((report['robots'], [w['robot'] for w in report['waited']]), (2, ['R2']))
        self.assertIn('complete', team.message)

    def test_routes_that_are_apart_start_together(self):
        team = self.started(APART)
        self.assertEqual([robot.nav.state for robot in team.robots], [RUNNING, RUNNING])
        self.assertEqual(team.plan_result.edges, ())
        self.finish(team, 40)
        self.assertEqual(team.phase, 'done', team.message)

    def test_four_robots_reach_their_goals_in_the_order_of_the_graph(self):
        team = self.started(FOUR)
        edges = {(e.first, e.then) for e in team.plan_result.edges}
        self.assertTrue(edges)                                      # this arena is crowded: somebody has to wait
        released = []
        closest = 9.0
        deadline = time.monotonic() + 150
        while team.phase == 'running' and time.monotonic() < deadline:
            for robot in team.robots:
                if robot.released and robot.id not in released:
                    released.append(robot.id)
            truth = self.truth(team)
            closest = min([closest] + [math.dist(truth[i][:2], truth[j][:2]) for i in range(4) for j in range(i)])
            time.sleep(0.05)
        self.assertEqual(team.phase, 'done', team.message)
        for first, then in edges:                                   # nobody was released before a robot it waits for
            self.assertLess(released.index(first), released.index(then))
        self.assertGreater(closest, 2 * team.body + 0.05)
        self.assertFalse(any(robot.backend.collided for robot in team.robots))
        for robot in team.robots:
            self.assertLess(math.dist(robot.backend.truth_pose()[:2], robot.goal), 0.12)

    def test_looking_around_is_a_task_after_arriving(self):
        team = self.started(([(1.2, 0.5, 0.0)], {1: (1.7, 0.5)}), scan=True)
        heading = []
        deadline = time.monotonic() + 80             # (it turns slowly, so that the camera sees while it turns)
        while team.phase == 'running' and time.monotonic() < deadline:
            if team.graph.state['r1_scan'] == 'RUNNING':
                heading.append(team.robots[0].backend.truth_pose()[2])
            time.sleep(0.05)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertGreater(max(heading) - min(heading), math.radians(200))      # it really turned round
        self.assertLess(self.said('R1 reached its goal'), self.said('R1 has looked around'))
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], (1.7, 0.5)), 0.12)   # ... on the spot


class TroubleTests(TeamCase):
    """A robot that stops for good does not take the team down with it: whoever can still reach its goal does."""

    def stop_first_at(self, team, x):
        """Stop R1 for good (a latched stop, like a fault) once it has driven as far as ``x``."""
        first = team.robots[0]
        self.assertTrue(wait_for(lambda: first.backend.truth_pose()[0] >= x, 30, 0.01))
        first.nav.emergency_stop('test: a fault in R1')
        return first.backend.truth_pose()

    def test_a_robot_that_stops_out_of_the_way_lets_the_waiting_robot_go(self):
        team = self.started(CROSS)
        self.assertEqual(team.snapshot()['robots'][1]['task'], 'waits for R1')
        time.sleep(0.8)
        team.robots[0].nav.emergency_stop('test: a fault in R1')          # R1 has hardly left its start
        self.finish(team, 40)
        states = self.states(team)
        self.assertEqual((states['r1_clear_r2'], states['r1_arrive']), (SUCCEEDED, FAILED))
        self.assertEqual((states['r2_arrive'], states['team_report']), (SUCCEEDED, BLOCKED))
        self.assertEqual(team.phase, 'failed')                            # not every robot reached its goal
        self.assertLess(self.said('R1 is not in the way of R2, which can go on'), self.said('R2 starts'))
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], (1.9, 1.5)), 0.12)
        self.assertEqual(team.robots[0].nav.state, STOPPED)               # nobody restarted the stopped robot
        self.assertIn('R2 reached its goal', team.message)
        self.assertIn('R1 stopped: test: a fault in R1', team.message)
        self.assertIn('test: a fault in R1', team.snapshot()['robots'][0]['note'])

    def test_a_robot_stopped_on_the_route_of_another_is_driven_around(self):
        team = self.started(CROSS)
        planned = team.plan_result.member(2).path
        stopped_at = self.stop_first_at(team, 1.85)                       # on R2's route, which runs up x = 1.9
        closest = 9.0
        deadline = time.monotonic() + 60
        while team.phase == 'running' and time.monotonic() < deadline:
            closest = min(closest, math.dist(team.robots[1].backend.truth_pose()[:2], stopped_at[:2]))
            time.sleep(0.02)
        states = self.states(team)
        self.assertEqual((states['r1_arrive'], states['r2_arrive']), (FAILED, SUCCEEDED))
        self.assertLess(self.said('R2 takes another way, around R1'), self.said('R2 starts'))
        self.assertNotEqual(team.plan_result.member(2).path, planned)     # the plan shows the route it really took
        self.assertGreater(closest, 2 * team.body + 0.05)                 # it kept its distance from R1
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], (1.9, 1.5)), 0.12)
        self.assertFalse(team.robots[1].backend.collided)
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], stopped_at[:2]), 0.05)   # R1 stayed

    def test_a_robot_stopped_in_the_only_way_out_keeps_the_other_one_in(self):
        # the corner at the bottom left of ENV1 is a dead end 0.5 m wide: R1 stands in its mouth, R2 behind it
        team = self.team([(0.85, 0.33, 0.0), (0.38, 0.33, 0.0)])
        plan = team.plan({1: (2.0, 0.4), 2: (1.6, 1.2)}, scan=False)
        self.assertEqual([(e.first, e.then, e.forced) for e in plan.edges], [(1, 2, True)])
        self.assertTrue(team.run()[0])
        waiting_place = team.robots[1].backend.truth_pose()
        time.sleep(0.3)
        team.robots[0].nav.emergency_stop('test: a fault in R1')
        self.finish(team, 10)
        states = self.states(team)
        self.assertEqual((states['r1_clear_r2'], states['r1_arrive']), (FAILED, FAILED))
        self.assertEqual((states['r2_arrive'], states['team_report']), (BLOCKED, BLOCKED))
        self.assertEqual(team.phase, 'failed')
        time.sleep(0.5)
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], waiting_place[:2]), 0.01)   # R2 never moved
        self.assertIsNotNone(self.said('R2 cannot start: there is no way around R1'))
        shown = team.snapshot()['robots'][1]
        self.assertEqual((shown['task'], shown['note']), ('cannot start', 'cannot start: there is no way around R1'))
        self.assertIn('Did not start: R2 (cannot start: there is no way around R1)', team.message)

    def test_a_failure_does_not_stop_a_robot_that_has_nothing_to_do_with_it(self):
        team = self.started(APART)
        time.sleep(0.8)
        team.robots[0].nav.emergency_stop('test: a fault in R1')
        self.finish(team, 40)
        states = self.states(team)
        self.assertEqual((states['r1_arrive'], states['r2_arrive']), (FAILED, SUCCEEDED))
        self.assertEqual(team.robots[1].nav.state, ARRIVED)
        self.assertEqual(team.phase, 'failed')
        self.assertIn('Team mission ended early. R2 reached its goal. R1 stopped: test: a fault in R1', team.message)
        self.assertIn('Plan again to go on from here', team.message)

    def halt_first(self, team, times):
        """Stop R1 the way the navigator stops itself when it loses its line (no latch), ``times`` times."""
        first, halts = team.robots[0], 0
        deadline = time.monotonic() + 90
        while team.phase == 'running' and time.monotonic() < deadline:
            nav = first.nav
            if halts < times and first.retries == halts and nav.state == RUNNING and nav.motion == 'path':
                time.sleep(0.6)
                nav.halt_motion('test: lost the line', kind='tracking')
                halts += 1
            time.sleep(0.05)
        return halts

    def test_a_robot_that_stops_for_something_it_can_get_over_plans_again_and_drives_on(self):
        team = self.started(APART)
        self.assertEqual(self.halt_first(team, 1), 1)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertLess(self.said('R1 stopped (test: lost the line) and plans again: try 1 of 3'),
                        self.said('R1 continues on a new route'))
        self.assertEqual(team.snapshot()['robots'][0]['retries'], 1)
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], (2.6, 0.3)), 0.12)

    def test_a_robot_that_keeps_stopping_gives_up_after_three_new_tries(self):
        team = self.started(APART)
        self.assertEqual(self.halt_first(team, 4), 4)
        states = self.states(team)
        self.assertEqual((states['r1_arrive'], states['r2_arrive']), (FAILED, SUCCEEDED))
        self.assertIsNotNone(self.said('try 3 of 3'))
        self.assertIsNone(self.said('try 4'))
        self.assertIn('R1 stopped (after 3 new tries): test: lost the line', team.message)
        self.assertEqual(team.robots[0].nav.state, STOPPED)

    def test_stop_stops_every_robot(self):
        team = self.started(APART)
        time.sleep(1.0)
        team.stop('Operator STOP')
        self.assertEqual([robot.nav.state for robot in team.robots], [STOPPED, STOPPED])
        self.assertEqual(team.phase, 'stopped')
        self.assertEqual(set(self.states(team).values()), {CANCELLED})
        time.sleep(0.6)
        before = self.truth(team)
        time.sleep(0.5)
        for a, b in zip(before, self.truth(team)):
            self.assertLess(math.dist(a[:2], b[:2]), 0.002)         # they really stand still
        self.assertFalse(team.moving())
        ok, text = team.run()                                       # and nothing restarts without a new plan
        self.assertFalse(ok)
        self.assertIn('Plan the team mission first', text)

    def test_a_robot_that_keeps_planning_new_routes_without_getting_anywhere_stops_for_good(self):
        # Something the map does not show stands right in front of R1 (its safety stop refuses, the navigator
        # plans the same route again, ...). The team lets it try a few times, not for ever.
        team = self.team(APART[0])
        team.plan(APART[1], scan=False)
        first = team.robots[0]
        go = first.nav.authorize

        def stopped_at_once():
            result = go()
            first.nav.halt_motion('test: obstacle ahead', kind='obstacle')
            first.nav.set_goal(first.goal)                          # a "new" route from where it stands
            return result

        first.nav.authorize = stopped_at_once
        self.assertTrue(team.run()[0])
        self.finish(team, 60)
        states = self.states(team)
        self.assertEqual((states['r1_arrive'], states['r2_arrive']), (FAILED, SUCCEEDED))
        self.assertEqual(sum('R1 continues on a new route' in text for text in self.events), REPLANS)
        self.assertIn('R1 cannot get past what is in front of it (%d new routes tried)' % REPLANS, team.message)
        self.assertLess(math.dist(first.backend.truth_pose()[:2], APART[0][0][:2]), 0.05)      # it never got away

    def test_a_stop_that_comes_in_the_middle_of_a_look_at_the_team_is_still_a_stop(self):
        # STOP does not wait for the supervisor. The supervisor may be in the middle of a step (it holds the
        # team's lock then) and find robots that have just been stopped: that must not count as a failure.
        team = self.started(CROSS)
        time.sleep(0.8)
        with team._lock:                                            # a step is under way ...
            stopper = threading.Thread(target=team.stop, args=('Operator STOP',))
            stopper.start()                                         # ... when STOP comes
            self.assertTrue(wait_for(lambda: all(robot.nav.state != RUNNING and robot.nav.safety.estop
                                                 for robot in team.robots), 3))
            for robot in team.robots:                               # the step goes on and looks at the robots
                team._drive_tasks(robot)
            team._release()
            self.assertFalse(any(robot.failed for robot in team.robots))
            self.assertNotIn(FAILED, self.states(team).values())
            team.step()
        stopper.join(3)
        self.assertFalse(stopper.is_alive())
        self.assertEqual(team.phase, 'stopped')
        self.assertEqual(set(self.states(team).values()), {CANCELLED})
        self.assertIn('Stopped: Operator STOP', team.message)
        self.assertIsNone(self.said('R1 stopped'))                  # nobody was reported as failed
        self.assertIsNone(team._stopping)
        team.robots[0].nav.emergency_stop('later')                  # and a later stop is not swallowed
        self.assertEqual(team.phase, 'stopped')

    def test_robots_that_come_too_close_are_stopped(self):
        team = self.started(APART)
        time.sleep(0.8)
        first, second = team.robots
        # as if R1 had strayed right next to R2 (its believed position is what the laptop goes by)
        first.position = lambda: (second.position()[0] + 0.10, second.position()[1])
        self.assertTrue(wait_for(lambda: self.said('too close to') is not None, 3))
        self.finish(team, 5)
        self.assertEqual([robot.nav.state for robot in team.robots], [STOPPED, STOPPED])
        self.assertEqual(self.states(team)['r1_arrive'], FAILED)
        self.assertEqual(team.phase, 'failed')

    def test_the_simulated_robots_cannot_drive_through_each_other(self):
        backend = TeamSimBackend(self.gt, (1.2, 0.5, 0.0), body_radius=0.08, realtime=False)
        backend.others = [(1.6, 0.5, 0.106)]                        # a team-mate 40 cm ahead
        backend.send_velocity(0.2, 0.0, 0.0)
        for _ in range(400):                                        # 4 simulated seconds
            backend.advance(0.01)
        self.assertTrue(backend.collided)
        self.assertLess(backend.truth_pose()[0], 1.6 - 0.106 - 0.08 + 0.01)       # stopped at its body, not inside
        free = TeamSimBackend(self.gt, (1.2, 0.5, 0.0), body_radius=0.08, realtime=False)
        free.send_velocity(0.2, 0.0, 0.0)
        for _ in range(400):
            free.advance(0.01)
        self.assertFalse(free.collided)
        self.assertGreater(free.truth_pose()[0], 1.8)

    def test_a_dropped_obstacle_is_driven_around(self):
        if not HAVE_CV2:
            self.skipTest('OpenCV not installed (pip install -r requirements-yolo.txt)')
        team = self.team([(1.2, 0.5, 0.0), (1.2, 1.3, 0.0)], camera=True)
        team.plan({1: (2.6, 0.5), 2: (2.0, 1.3)}, scan=False)
        planned = team.plan_result.member(1).path
        self.assertTrue(team.run()[0])
        time.sleep(1.2)
        team.toggle_block(2.1, 0.5, 'red')                          # in front of R1 while it is driving
        self.finish(team, 60)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertIsNotNone(self.said('R1 continues on a new route'))
        self.assertNotEqual(tuple(team.robots[0].nav.path), planned)
        self.assertFalse(team.robots[0].backend.collided)
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], (2.6, 0.5)), 0.12)


class PlanKeepingTests(TeamCase):
    """Between planning and the go-ahead the navigators keep their routes up to date. The plan follows them."""

    @staticmethod
    def blocks(*points):
        return [dict(x=x, y=y, size=0.05, cls='red_obstacle', id=k) for k, (x, y) in enumerate(points, 1)]

    def test_a_route_that_shifts_a_little_keeps_the_plan_and_the_authorization(self):
        team = self.team([(1.2, 0.3, 0.0), (1.2, 1.6, 0.0)])                    # R2 far away from R1's side of the arena
        plan = team.plan({1: (2.6, 0.3), 2: (1.9, 1.6)}, scan=False)
        self.assertEqual(plan.edges, ())
        before = plan.member(1).path
        team.robots[0].nav.set_vision_obstacles(self.blocks((1.9, 0.3)))        # R1's camera saw a block on its route
        self.assertTrue(wait_for(lambda: team.plan_result is not None and team.plan_result.member(1).path != before, 3))
        self.assertEqual(team.phase, 'planned')                                 # still a plan, with the new route
        self.assertEqual(team.plan_result.member(1).path, tuple(tuple(p) for p in team.robots[0].nav.path))
        self.assertEqual(team.plan_result.signature, plan.signature)            # the same order: nobody waits
        self.assertIsNone(self.said('order of the team'))
        ok, text = team.run(authorized=plan.signature)                          # the go-ahead given before still fits
        self.assertTrue(ok, text)
        self.finish(team, 50)
        self.assertEqual(team.phase, 'done', team.message)

    def test_a_route_change_that_changes_the_order_needs_a_new_go_ahead(self):
        team = self.team(APART[0])
        plan = team.plan(APART[1], scan=False)
        self.assertEqual(plan.edges, ())
        # a row of blocks across R1's way: its new route has to go up, right next to R2's route
        team.robots[0].nav.set_vision_obstacles(self.blocks(*[(1.9, 0.2 + 0.1 * k) for k in range(7)]))
        self.assertTrue(wait_for(lambda: team.plan_result is not None and team.plan_result.edges, 3))
        self.assertEqual([(e.first, e.then) for e in team.plan_result.edges], [(1, 2)])
        self.assertNotEqual(team.plan_result.signature, plan.signature)
        self.assertIsNotNone(self.said('A route changed, and with it the order of the team'))
        ok, text = team.run(authorized=plan.signature)
        self.assertFalse(ok)
        self.assertIn('The plan changed after you were asked', text)
        self.assertFalse(team.moving())
        ok, text = team.run(authorized=team.plan_result.signature)
        self.assertTrue(ok, text)
        self.assertEqual(team.robots[0].nav.state, RUNNING)
        self.assertNotEqual(team.robots[1].nav.state, RUNNING)                  # R2 waits now

    def test_a_route_that_is_lost_drops_the_plan(self):
        team = self.team(APART[0])
        team.plan(APART[1], scan=False)
        team.robots[1].nav.set_vision_obstacles(self.blocks((2.6, 1.0)))        # a block right on R2's goal
        self.assertTrue(wait_for(lambda: team.phase == 'ready', 3))
        self.assertIn('R2 has no planned route any more', team.message)
        self.assertEqual(team.run(), (False, 'Plan the team mission first.'))


class SetupTests(TeamCase):
    def test_two_robots_that_would_have_to_swap_places_are_refused(self):
        team = self.team([(1.2, 0.5, 0.0), (2.4, 0.5, math.pi)])
        with self.assertRaisesRegex(TeamError, 'R1 and R2 block each other'):
            team.plan({1: (2.4, 0.5), 2: (1.2, 0.5)})
        self.assertEqual(team.phase, 'ready')
        self.assertEqual(team.run(), (False, 'Plan the team mission first.'))

    def test_starts_and_goals_have_to_be_apart_and_inside_the_map(self):
        with self.assertRaisesRegex(TeamError, 'R1 and R2 are too close together.*starting places'):
            self.team([(1.2, 0.5, 0.0), (1.4, 0.5, 0.0)])
        with self.assertRaisesRegex(TeamError, 'R2 cannot stand there'):
            self.team([(1.2, 0.5, 0.0), (2.95, 0.5, 0.0)])          # inside the right wall
        with self.assertRaisesRegex(TeamError, '1 to 4 robots'):
            self.team([(1.2, 0.3 + 0.1 * k, 0.0) for k in range(5)])
        team = self.team(APART[0])
        with self.assertRaisesRegex(TeamError, 'Set a goal for at least one robot first'):
            team.plan({})
        with self.assertRaisesRegex(TeamError, 'Set a goal for at least one robot first'):
            team.plan({1: None, 2: None})
        with self.assertRaisesRegex(TeamError, 'too close together.*goals'):
            team.plan({1: (2.6, 0.3), 2: (2.5, 0.4)})
        with self.assertRaisesRegex(TeamError, 'outside the map'):
            team.plan({1: (2.6, 0.3), 2: (9.0, 1.0)})
        with self.assertRaisesRegex(TeamError, 'R1 has no route to its goal'):
            team.plan({1: (0.6, 0.9), 2: (2.6, 1.0)})               # a place walled in on all sides

    def test_a_robot_without_a_goal_stays_where_it_is_and_the_others_drive_around_it(self):
        team = self.team([(1.2, 0.8, 0.0), (1.9, 0.8, 0.0)])                    # R2 stands on R1's straight way
        plan = team.plan({1: (2.6, 0.8)}, scan=False)
        self.assertEqual([task.name for task in plan.tasks], ['r1_arrive', 'team_report'])      # no task for R2
        self.assertIn('R2 stays where it is (no goal)', team.message)
        self.assertEqual(team.snapshot()['robots'][1]['task'], 'stays here')
        self.assertTrue(team.run()[0])
        parked, closest = team.robots[1].backend.truth_pose(), 9.0
        deadline = time.monotonic() + 60
        while team.phase == 'running' and time.monotonic() < deadline:
            closest = min(closest, math.dist(team.robots[0].backend.truth_pose()[:2], parked[:2]))
            time.sleep(0.02)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertGreater(closest, 2 * team.body + 0.05)                       # R1 went around it
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], parked[:2]), 0.002)  # R2 did not move
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], (2.6, 0.8)), 0.12)
        self.assertEqual(team.snapshot()['report']['robots'], 1)

    def test_the_arena_can_be_the_map_itself(self):
        """The console's choice: the robots drive in the operator's map, so what is drawn is what they meet."""
        cell = self.occ.world_to_cell((2.0, 1.6))
        team = self.team(APART[0])
        self.assertFalse(team.follows_map)                                      # here: a separate, true arena
        team.set_map(self.occ.edited(cell, True))
        self.assertIs(team.world, self.gt)
        self.assertIs(team.robots[0].backend.world, self.gt)
        same = TeamSim(self.occ, self.occ, team.planner_config, team.nav_config, APART[0], camera=False)
        self.teams.append(same)
        self.assertTrue(same.follows_map)
        painted = self.occ.edited(cell, True)
        same.set_map(painted)                                                   # the operator drew a wall cell
        self.assertIs(same.world, painted)
        for robot in same.robots:
            self.assertIs(robot.backend.world, painted)
            self.assertTrue(robot.backend.world.cells[cell])

    def test_a_robot_can_be_put_somewhere_else_but_not_while_the_team_drives(self):
        team = self.team(APART[0])
        team.plan(APART[1], scan=False)
        team.place(2, (1.2, 1.5, math.pi / 2))
        self.assertEqual(team.phase, 'ready')                       # the plan is gone with the old place
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], (1.2, 1.5)), 0.001)
        self.assertLess(math.dist(team.robots[1].position(), (1.2, 1.5)), 0.01)
        with self.assertRaisesRegex(TeamError, 'too close together'):
            team.place(2, (1.3, 0.4, 0.0))
        with self.assertRaisesRegex(TeamError, 'R2 cannot stand there'):
            team.place(2, (2.95, 1.0, 0.0))
        self.assertLess(math.dist(team.robots[1].position(), (1.2, 1.5)), 0.01)       # still where it was
        team.plan(APART[1], scan=False)
        self.assertTrue(team.run()[0])
        with self.assertRaisesRegex(TeamError, 'Stop the team before moving a robot'):
            team.place(1, (1.5, 0.3, 0.0))
        with self.assertRaisesRegex(TeamError, 'Stop the team before planning again'):
            team.plan(APART[1])

    def test_the_picture_of_the_team_for_the_page(self):
        team = self.team(CROSS[0])
        snap = team.snapshot()
        self.assertEqual((snap['phase'], snap['tasks'], snap['waves']), ('ready', [], []))
        self.assertEqual([(r['label'], r['colour']) for r in snap['robots']], [('R1', COLOURS[0]), ('R2', COLOURS[1])])
        team.plan(CROSS[1])
        snap = team.snapshot()
        self.assertEqual(snap['phase'], 'planned')
        self.assertIn('R2 after R1', snap['message'])
        self.assertEqual(snap['edges'][0]['first'], 1)
        self.assertEqual(snap['waves'][0], ['r1_clear_r2', 'r1_arrive'])
        task = next(t for t in snap['tasks'] if t['name'] == 'r2_arrive')
        self.assertEqual((task['state'], task['needs'], task['robot']), ('PENDING', ['r1_clear_r2'], 2))
        self.assertEqual(snap['robots'][0]['goal'], [2.6, 0.8])
        self.assertGreater(len(snap['planned'][0]['path']), 1)
        self.assertEqual(len(team.poses()), 2)
        self.assertEqual(team.poses()[0][3], 'PLANNED')

    def test_obstacles_drawn_for_the_other_robots(self):
        grid = with_discs(self.occ, [((1.5, 0.5), 0.2)])
        self.assertTrue(grid.cells[grid.world_to_cell((1.5, 0.5))])
        self.assertTrue(grid.cells[grid.world_to_cell((1.65, 0.5))])
        self.assertFalse(grid.cells[grid.world_to_cell((1.8, 0.5))])
        self.assertFalse(self.occ.cells[self.occ.world_to_cell((1.5, 0.5))])      # the operator's map is untouched
        self.assertIs(with_discs(self.occ, []), self.occ)


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class CameraTests(TeamCase):
    def test_every_robot_has_a_camera_and_what_they_see_is_shared(self):
        import cv2
        import numpy as np
        # R1 looks along +x at R2, 60 cm ahead. R2 looks up the map (+y) at a green block 55 cm ahead of it,
        # which is far off to the side for R1's camera.
        team = self.team([(1.2, 0.5, 0.0), (1.8, 0.5, math.pi / 2)], camera=True)
        team.toggle_block(1.8, 1.05, 'green')
        self.assertTrue(wait_for(lambda: team.frame(1, timeout=0.3) is not None
                                 and team.frame(2, timeout=0.3) is not None, 6))
        self.assertTrue(wait_for(lambda: len(team.frame(2, timeout=0.3)[4]) > 0, 6))      # R2's detector has a box
        self.assertTrue(wait_for(lambda: team.snapshot()['findings'], 6))
        time.sleep(1.0)
        finding = team.snapshot()['findings'][0]
        self.assertEqual((finding['id'], finding['by']), (1, ['R2']))                    # R1 does not see it
        self.assertLess(math.hypot(finding['x'] - 1.8, finding['y'] - 1.05), 0.1)
        self.assertIsNotNone(self.said('R2 sees a survivor'))
        # R1's picture shows its team-mate R2 as a box in R2's colour (teal)
        number, jpeg, width, height, boxes = team.frame(1)
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape[:2], (height, width))
        teal = np.array([0xbf, 0xd4, 0x2d])                                              # BGR of #2dd4bf
        self.assertGreater(int((np.abs(image.astype(int) - teal).max(axis=2) < 45).sum()), 400)
        self.assertEqual(boxes, [])                                                      # a robot is not a block
        self.assertEqual(team.snapshot()['blocks'], [[1.8, 1.05, 'green_obstacle']])
        self.assertIn('removed', team.toggle_block(1.81, 1.05, 'green'))
        self.assertEqual(team.snapshot()['blocks'], [])


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class SurvivorTests(TeamCase):
    """With ``approach`` a robot drives up to the survivor it has seen and secures it."""

    def survivors(self, team, count=1, timeout=8):
        self.assertTrue(wait_for(lambda: len(team.snapshot()['findings']) >= count, timeout), team.snapshot()['findings'])
        return team.snapshot()['findings']

    def test_a_robot_drives_up_to_the_survivor_it_found_and_secures_it(self):
        team = self.team([(1.2, 0.5, 0.0), (1.2, 1.3, 0.0)], camera=True)
        team.toggle_block(2.6, 0.9, 'green')                        # both will see it when they look around
        plan = team.plan({1: (2.0, 0.5), 2: (2.0, 1.3)}, scan=True, approach=True)
        self.assertEqual([t.name for t in plan.tasks if t.name.startswith('r1')], ['r1_arrive', 'r1_scan', 'r1_rescue'])
        self.assertTrue(team.run()[0])
        shown = [set(), set()]
        deadline = time.monotonic() + 150
        while team.phase == 'running' and time.monotonic() < deadline:
            for k, robot in enumerate(team.snapshot()['robots']):
                shown[k].add(robot['task'])
            time.sleep(0.05)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertIn('every robot reached its goal. 1 of 1 survivor secured', team.message)
        snap = team.snapshot()
        finding = snap['findings'][0]
        self.assertEqual(finding['status'], 'secured')
        self.assertIn(finding['secured_by'], ('R1', 'R2'))          # whoever had looked around first went
        went = int(finding['secured_by'][1]) - 1
        name, robot, other = finding['secured_by'], team.robots[went], team.robots[1 - went]
        self.assertTrue({'drives to S1', 'secures S1'} <= shown[went], shown)
        self.assertFalse({'drives to S1', 'secures S1'} & shown[1 - went], shown)             # one robot per survivor
        steps = ('%s has looked around', '%s drives to survivor S1', '%s is at survivor S1', '%s secured survivor S1')
        for earlier, later in zip(steps, steps[1:]):
            self.assertLess(self.said(earlier % name), self.said(later % name))
        x, y, yaw = robot.backend.truth_pose()
        self.assertTrue(0.30 < math.hypot(2.6 - x, 0.9 - y) < 0.50)                # one gap in front of it ...
        off = math.atan2(0.9 - y, 2.6 - x) - yaw
        self.assertLess(abs(math.atan2(math.sin(off), math.cos(off))), math.radians(12))     # ... and facing it
        self.assertLess(math.dist(other.backend.truth_pose()[:2], other.goal), 0.12)         # the other stayed at its goal
        self.assertFalse(robot.backend.collided or other.backend.collided)
        notes = {task['name']: (task['state'], task['note']) for task in snap['tasks']}
        self.assertEqual(notes['r%d_rescue' % robot.id], (SUCCEEDED, 'secured S1'))
        self.assertEqual(notes['r%d_rescue' % other.id], (SUCCEEDED, 'it found no survivor to go to'))
        self.assertEqual(snap['report']['survivors'][0]['secured_by'], name)
        self.assertEqual(snap['robots'][went]['task'], 'at survivor S1')
        self.assertEqual(snap['robots'][1 - went]['task'], '')

    def test_without_the_order_to_go_a_survivor_is_only_reported(self):
        team = self.team([(1.2, 0.5, 0.0), (1.2, 1.3, 0.0)], camera=True)
        team.toggle_block(1.9, 0.5, 'green')                        # straight ahead of R1
        self.survivors(team)
        team.plan({1: (1.5, 0.5), 2: (1.5, 1.3)}, scan=False)
        self.assertTrue(team.run()[0])
        self.finish(team, 30)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertNotIn('secured', team.message)
        self.assertEqual(team.snapshot()['findings'][0]['status'], 'found')
        self.assertLess(math.dist(team.robots[0].backend.truth_pose()[:2], (1.5, 0.5)), 0.12)   # it stayed at its goal

    def test_a_survivor_behind_a_wall_is_not_found(self):
        # R1 looks at the wall at y = 0.6 ... 0.7 m; the green block is in the room behind that wall
        team = self.team([(0.5, 0.2, math.pi / 2), (1.6, 0.5, 0.0)], camera=True)
        team.toggle_block(0.62, 1.00, 'green')
        self.assertTrue(wait_for(lambda: team.frame(1, timeout=0.3) is not None, 6))
        time.sleep(2.5)
        self.assertEqual(team.snapshot()['findings'], [])
        self.assertEqual(team.frame(1)[4], [])                      # no box in its picture either
        team.toggle_block(0.5, 0.52, 'green')                       # one on its own side of the wall
        finding = self.survivors(team)[0]
        self.assertEqual(finding['by'], ['R1'])
        self.assertLess(math.hypot(finding['x'] - 0.5, finding['y'] - 0.52), 0.1)
        time.sleep(1.0)
        self.assertEqual(len(team.snapshot()['findings']), 1)       # and still only that one

    def test_it_waits_with_its_survivor_until_the_other_robot_has_passed(self):
        # R1 has a short way and sees a survivor straight ahead; R2's long route runs between R1 and the survivor
        team = self.team([(2.0, 0.3, math.pi / 2), (2.75, 1.1, math.pi)], camera=True)
        team.toggle_block(2.0, 1.7, 'green')
        self.assertIn('R1', self.survivors(team)[0]['by'])
        plan = team.plan({1: (2.0, 0.6), 2: (1.3, 1.1)}, scan=False, approach=True)
        self.assertEqual(plan.edges, ())                            # the routes to the goals are apart
        self.assertTrue(team.run()[0])
        closest, waited = 9.0, False
        deadline = time.monotonic() + 90
        while team.phase == 'running' and time.monotonic() < deadline:
            a, b = self.truth(team)
            closest = min(closest, math.dist(a[:2], b[:2]))
            waited = waited or team.snapshot()['robots'][0]['task'] == 'waits to go to S1'
            time.sleep(0.05)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertTrue(waited)
        self.assertLess(self.said('R1 waits for R2 before it drives to survivor S1'), self.said('R1 drives to survivor S1'))
        self.assertIsNotNone(self.said('R1 secured survivor S1'))
        self.assertGreater(closest, 2 * team.body + 0.05)           # they never came near each other
        self.assertFalse(any(robot.backend.collided for robot in team.robots))
        self.assertLess(math.dist(team.robots[1].backend.truth_pose()[:2], (1.3, 1.1)), 0.12)
        self.assertTrue(0.30 < math.dist(team.robots[0].backend.truth_pose()[:2], (2.0, 1.7)) < 0.50)

    def test_a_survivor_in_the_way_is_gone_to_instead_of_the_goal(self):
        # R1's goal is at the end of a dead end 0.5 m wide (bottom left of ENV1). A survivor stands in its mouth:
        # too far away to be known when the team is planned, and no robot can pass it.
        team = self.team([(2.4, 0.33, math.pi), (2.6, 1.5, -math.pi / 2)], camera=True)
        team.toggle_block(0.75, 0.42, 'green')                      # the survivor, and a red block beside it:
        team.toggle_block(0.75, 0.20, 'red')                        # together they close the dead end
        time.sleep(1.5)
        self.assertEqual(team.snapshot()['findings'], [])
        team.plan({1: (0.38, 0.33), 2: (2.6, 1.0)}, scan=False, approach=True)
        self.assertTrue(team.run()[0])
        self.finish(team, 90)
        self.assertEqual(team.phase, 'done', team.message)          # finding a survivor is not a failure
        self.assertIn('R2 reached its goal; R1 stopped at a survivor on its way. 1 of 1 survivor secured', team.message)
        events = ('R1 sees a survivor', 'It goes to survivor S1, which it found, instead', 'R1 drives to survivor S1',
                  'R1 secured survivor S1')
        for earlier, later in zip(events, events[1:]):
            self.assertLess(self.said(earlier), self.said(later))
        first = team.robots[0]
        self.assertTrue(0.30 < math.dist(first.backend.truth_pose()[:2], (0.75, 0.42)) < 0.50)
        self.assertGreater(math.dist(first.backend.truth_pose()[:2], (0.38, 0.33)), 0.5)     # not at its goal
        self.assertFalse(first.backend.collided)
        snap = team.snapshot()
        states = {task['name']: (task['state'], task['note']) for task in snap['tasks']}
        self.assertEqual(states['r1_arrive'][0], SUCCEEDED)
        self.assertIn('did not get to its goal', states['r1_arrive'][1])
        self.assertEqual(states['r1_rescue'], (SUCCEEDED, 'secured S1'))
        self.assertIn('did not get to its goal', snap['robots'][0]['note'])
        self.assertEqual(snap['findings'][0]['secured_by'], 'R1')

    def test_a_robot_that_can_reach_neither_its_goal_nor_a_survivor_has_failed(self):
        team = self.team([(2.4, 0.33, math.pi), (2.6, 1.5, -math.pi / 2)], camera=True)
        team.toggle_block(0.75, 0.42, 'green')
        team.toggle_block(0.75, 0.20, 'red')
        team.plan({1: (0.38, 0.33), 2: (2.6, 1.0)}, scan=False, approach=True)
        team._wall_between = lambda a, b: True                      # as if no spot in front of the survivor could be used
        self.assertTrue(team.run()[0])
        self.finish(team, 90)
        self.assertEqual(team.phase, 'failed', team.message)
        self.assertIn('R2 reached its goal', team.message)
        self.assertIn('R1 did not get to its goal', team.message)
        self.assertEqual(self.states(team)['r1_rescue'], FAILED)
        self.assertEqual(team.snapshot()['findings'][0]['status'], 'found')

    def test_a_survivor_it_cannot_get_to_does_not_end_the_mission(self):
        team = self.team([(1.2, 0.5, 0.0), (1.2, 1.3, 0.0)], camera=True)
        team.toggle_block(1.9, 0.5, 'green')
        self.survivors(team)
        team.plan({1: (1.5, 0.5), 2: (1.5, 1.3)}, scan=False, approach=True)
        team._wall_between = lambda a, b: True                      # as if a wall stood in front of every spot
        self.assertTrue(team.run()[0])
        self.finish(team, 30)
        self.assertEqual(team.phase, 'done', team.message)
        self.assertIn('0 of 1 survivor secured', team.message)
        self.assertIsNotNone(self.said('R1 does not go to survivor S1: there is no safe way to it'))
        finding = team.snapshot()['findings'][0]
        self.assertEqual((finding['status'], finding['claimed']), ('found', None))
        note = next(t['note'] for t in team.snapshot()['tasks'] if t['name'] == 'r1_rescue')
        self.assertEqual(note, 'did not get to S1')

    def test_stop_on_the_way_to_a_survivor_stops_the_team(self):
        team = self.team([(1.2, 0.5, 0.0), (1.2, 1.3, 0.0)], camera=True)
        team.toggle_block(2.6, 0.5, 'green')
        self.survivors(team)
        team.plan({1: (1.5, 0.5), 2: (1.5, 1.3)}, scan=False, approach=True)
        self.assertTrue(team.run()[0])
        self.assertTrue(wait_for(lambda: self.said('R1 drives to survivor S1') is not None, 20))
        time.sleep(0.8)
        team.stop('Operator STOP')
        self.assertEqual(team.phase, 'stopped')
        self.assertEqual(team.robots[0].nav.state, STOPPED)
        self.assertEqual(self.states(team)['r1_rescue'], CANCELLED)
        self.assertEqual(team.snapshot()['findings'][0]['status'], 'found')
        time.sleep(0.6)
        before = team.robots[0].backend.truth_pose()
        time.sleep(0.5)
        self.assertLess(math.dist(before[:2], team.robots[0].backend.truth_pose()[:2]), 0.002)


if __name__ == '__main__':
    unittest.main()
