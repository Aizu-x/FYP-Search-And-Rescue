"""Team missions, the planning part (ugv_nav/team_plan.py): which robot has to wait for which, the task graph
that follows from it, and how that graph is worked through. Pure geometry and bookkeeping: no simulator, no
robot. The order comes from Kahn's algorithm in the team's scheduler (DAG_scheduler/dag_scheduler/engine.py).
"""
import unittest
from unittest import mock

from ugv_nav import team_plan
from ugv_nav.team_plan import (BLOCKED, CANCELLED, FAILED, PENDING, READY, RUNNING, SUCCEEDED, LiveGraph, Member,
                               TeamError, ahead, build_plan, dag_engine, path_gap, point_gap, precedence)

D = 0.30


def robot(robot_id, *points):
    """A robot whose route is the straight line(s) through ``points``; it starts at the first, ends at the last."""
    return Member(robot_id, tuple(points[0]), tuple(points[-1]), tuple(tuple(p) for p in points))


class GeometryTests(unittest.TestCase):
    def test_distance_from_a_point_to_a_route(self):
        route = [(0, 0), (2, 0), (2, 2)]
        self.assertAlmostEqual(point_gap((1, 0.5), route)[0], 0.5)
        self.assertEqual(point_gap((1, 0.5), route)[1], (1.0, 0.0))
        self.assertAlmostEqual(point_gap((3, 3), route)[0], 2 ** 0.5)          # beyond the end: to the last point
        self.assertAlmostEqual(point_gap((0.3, 0.4), [(0, 0)])[0], 0.5)         # a route of one point

    def test_distance_between_two_routes(self):
        self.assertAlmostEqual(path_gap([(0, 0), (2, 0)], [(0, 1), (2, 1)])[0], 1.0)          # side by side
        gap, where = path_gap([(0, 1), (2, 1)], [(1, 0), (1, 2)])                             # crossing
        self.assertEqual(gap, 0.0)
        self.assertAlmostEqual(where[0], 1.0); self.assertAlmostEqual(where[1], 1.0)
        self.assertAlmostEqual(path_gap([(0, 0), (1, 0)], [(1.4, 0), (3, 0)])[0], 0.4)        # end to end
        self.assertAlmostEqual(path_gap([(0, 0)], [(0, 0.2), (1, 0.2)])[0], 0.2)              # a parked robot

    def test_the_part_of_a_route_still_ahead(self):
        route = [(0, 0), (1, 0), (1, 1), (2, 1)]
        self.assertEqual(ahead(route, (0.4, 0.02)), [(0.4, 0.02), (1, 0), (1, 1), (2, 1)])
        self.assertEqual(ahead(route, (1.02, 0.5)), [(1.02, 0.5), (1, 1), (2, 1)])
        self.assertEqual(ahead(route, (1.5, 1.0), segment=2), [(1.5, 1.0), (2, 1)])
        self.assertEqual(ahead(route, (2, 1), segment=99), [(2, 1), (2, 1)])                  # at the end
        self.assertEqual(ahead([(0, 0)], (0.1, 0)), [(0.1, 0)])
        # a route that comes back past its own start: the robot is at the beginning, not near the end
        loop = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0.05)]
        self.assertEqual(len(ahead(loop, (0, 0))), 5)


class PrecedenceTests(unittest.TestCase):
    def test_routes_far_apart_need_no_order(self):
        self.assertEqual(precedence([robot(1, (0, 0), (2, 0)), robot(2, (0, 1), (2, 1))], D), [])

    def test_crossing_routes_the_lower_number_goes_first(self):
        edges = precedence([robot(2, (1, 0), (1, 2)), robot(1, (0, 1), (2, 1))], D)
        self.assertEqual(len(edges), 1)
        edge = edges[0]
        self.assertEqual((edge.first, edge.then, edge.forced), (1, 2, False))
        self.assertEqual(edge.gap, 0.0)
        self.assertEqual(edge.at, (1.0, 1.0))
        self.assertIn('R1 goes first', edge.why)

    def test_a_robot_standing_on_a_route_has_to_move_away_first(self):
        # R2 stands in the middle of R1's route and drives off to the side
        edges = precedence([robot(1, (0, 1), (3, 1)), robot(2, (1.5, 1.05), (1.5, 2.5))], D)
        self.assertEqual([(e.first, e.then, e.forced) for e in edges], [(2, 1, True)])
        self.assertIn("R1's route passes the place where R2 is standing", edges[0].why)

    def test_a_goal_on_a_route_the_other_passes_before_the_robot_parks_there(self):
        # R1's goal is on R2's route: R2 has to be through before R1 parks
        edges = precedence([robot(1, (1.5, 2.5), (1.5, 1.05)), robot(2, (0, 1), (3, 1))], D)
        self.assertEqual([(e.first, e.then, e.forced) for e in edges], [(2, 1, True)])
        self.assertIn("R1 would be parked at its goal on R2's route", edges[0].why)

    def test_two_robots_that_block_each_other_are_refused(self):
        # each one's goal is the other's start: whoever goes first drives into the one that waits
        with self.assertRaisesRegex(TeamError, 'R1 and R2 block each other.*Move a start or a goal'):
            precedence([robot(1, (0, 0), (2, 0)), robot(2, (2, 0.05), (0, 0.05))], D)

    def test_three_robots_waiting_in_a_circle_are_refused(self):
        # R1 drives up to where R2 stands, R2 up to where R3 stands, R3 up to where R1 stands
        ring = [robot(1, (0, 0), (1.9, 0)), robot(2, (2, 0), (1.05, 1.9)), robot(3, (1, 2), (0.05, 0.1))]
        with self.assertRaisesRegex(TeamError, 'wait for each other in a circle.*R1, R2, R3'):
            precedence(ring, D)

    def test_a_free_order_follows_the_forced_ones(self):
        # R3 stands on R1's route (forced: R3 before R1). R1 and R2 cross and either could go first; Kahn's
        # order of the forced part is R2, R3, R1, so R2 goes before R1 although R1 has the lower number.
        team = [robot(1, (0, 1), (3, 1)), robot(2, (2.5, 0), (2.5, 2.2)), robot(3, (1.5, 1.05), (1.5, 2.5))]
        edges = {(e.first, e.then): e.forced for e in precedence(team, D)}
        self.assertEqual(edges, {(3, 1): True, (2, 1): False})

    def test_the_order_does_not_depend_on_how_the_robots_are_listed(self):
        team = [robot(1, (0, 1), (3, 1)), robot(2, (2.5, 0), (2.5, 2.2)), robot(3, (1.5, 1.05), (1.5, 2.5))]
        first = [(e.first, e.then) for e in precedence(team, D)]
        self.assertEqual([(e.first, e.then) for e in precedence(team[::-1], D)], first)


class PlanTests(unittest.TestCase):
    def crossing(self, scan=True, approach=False):
        return build_plan([robot(1, (0, 1), (2, 1)), robot(2, (1, 0), (1, 2)), robot(3, (0, 3), (2, 3))], D, scan=scan,
                          approach=approach)

    def test_tasks_of_a_team_with_one_crossing(self):
        plan = self.crossing()
        needs = {task.name: task.needs for task in plan.tasks}
        self.assertEqual(needs, {
            'r1_clear_r2': (), 'r1_arrive': (), 'r1_scan': ('r1_arrive',),
            'r2_arrive': ('r1_clear_r2',), 'r2_scan': ('r2_arrive',),
            'r3_arrive': (), 'r3_scan': ('r3_arrive',),
            'team_report': ('r1_scan', 'r2_scan', 'r3_scan')})
        self.assertEqual(plan.about['r1_clear_r2'], dict(robot=1, kind='clear', other=2,
                                                         text="R1 drives clear of R2's route"))
        self.assertEqual(plan.about['team_report']['kind'], 'report')
        self.assertEqual(plan.member(2).goal, (1, 2))

    def test_waves_say_what_can_start_together(self):
        plan = self.crossing()
        self.assertEqual(plan.waves[0], ('r1_clear_r2', 'r1_arrive', 'r3_arrive'))       # R1 and R3 start at once
        self.assertIn('r2_arrive', plan.waves[1])                                        # R2 after R1 is clear
        self.assertEqual(plan.waves[-1], ('team_report',))
        position = {name: k for k, name in enumerate(plan.order)}
        for task in plan.tasks:                                                          # Kahn's order is valid
            for need in task.needs:
                self.assertLess(position[need], position[task.name])

    def test_without_looking_around(self):
        plan = self.crossing(scan=False)
        self.assertNotIn('r1_scan', plan.about)
        self.assertEqual(plan.tasks[-1].needs, ('r1_arrive', 'r2_arrive', 'r3_arrive'))

    def test_driving_to_survivors_is_one_more_task_after_the_others(self):
        plan = self.crossing(approach=True)
        by_name = {task.name: task for task in plan.tasks}
        self.assertEqual(by_name['r1_rescue'].needs, ('r1_scan',))
        self.assertEqual(plan.about['r2_rescue']['kind'], 'rescue')
        self.assertIn('survivors it found', plan.about['r2_rescue']['text'])
        self.assertEqual(plan.tasks[-1].needs, ('r1_rescue', 'r2_rescue', 'r3_rescue'))
        self.assertLess(plan.order.index('r3_scan'), plan.order.index('r3_rescue'))
        self.assertNotEqual(plan.signature, self.crossing().signature)             # another plan to authorize
        plain = self.crossing(scan=False, approach=True)
        self.assertEqual({t.name: t for t in plain.tasks}['r1_rescue'].needs, ('r1_arrive',))
        self.assertNotIn('r1_rescue', self.crossing().about)                       # only when asked for

    def test_one_robot_alone_is_a_plan_too(self):
        plan = build_plan([robot(1, (0, 0), (1, 0))], D)
        self.assertEqual([task.name for task in plan.tasks], ['r1_arrive', 'r1_scan', 'team_report'])
        self.assertEqual(plan.edges, ())

    def test_an_empty_team_and_a_blocked_team_are_refused(self):
        with self.assertRaises(TeamError):
            build_plan([], D)
        with self.assertRaises(TeamError):
            build_plan([robot(1, (0, 0), (2, 0)), robot(2, (2, 0.05), (0, 0.05))], D)

    def test_the_order_comes_from_the_teams_scheduler(self):
        engine = dag_engine()
        self.assertIn('Kahn', engine.topological_order.__doc__)
        self.assertEqual(team_plan.ENGINE_FILE.parts[-3:], ('DAG_scheduler', 'dag_scheduler', 'engine.py'))
        self.assertIsInstance(self.crossing().tasks[0], engine.Task)

    def test_without_the_scheduler_a_team_cannot_be_planned(self):
        with mock.patch.object(team_plan, '_engine', None), \
                mock.patch.object(team_plan, 'ENGINE_FILE', team_plan.ENGINE_FILE.with_name('missing.py')):
            with self.assertRaisesRegex(TeamError, 'DAG scheduler is missing'):
                build_plan([robot(1, (0, 0), (1, 0))], D)


class LiveGraphTests(unittest.TestCase):
    def graph(self):
        plan = build_plan([robot(1, (0, 1), (2, 1)), robot(2, (1, 0), (1, 2)), robot(3, (0, 3), (2, 3))], D)
        return LiveGraph(plan.tasks)

    def test_only_tasks_without_open_prerequisites_are_ready(self):
        graph = self.graph()
        self.assertEqual(graph.names(READY), ['r1_clear_r2', 'r1_arrive', 'r3_arrive'])
        self.assertEqual(graph.state['r2_arrive'], PENDING)
        with self.assertRaises(ValueError):
            graph.start('r2_arrive')                                             # may not start yet

    def test_a_finished_task_releases_the_ones_that_waited_for_it(self):
        graph = self.graph()
        for name in graph.names(READY):
            graph.start(name)
        self.assertEqual(graph.succeed('r1_clear_r2'), ['r2_arrive'])            # R1 is clear: R2 may go
        self.assertEqual(graph.state['r2_arrive'], READY)
        self.assertEqual(graph.succeed('r1_arrive'), ['r1_scan'])
        self.assertEqual(graph.status(), RUNNING)

    def test_working_through_the_whole_graph(self):
        graph = self.graph()
        done = []
        while not graph.finished():
            ready = graph.names(READY)
            self.assertTrue(ready)
            for name in ready:
                graph.start(name)
                graph.succeed(name)
                done.append(name)
        self.assertEqual(graph.status(), SUCCEEDED)
        self.assertEqual(done[-1], 'team_report')
        self.assertEqual(len(done), 8)

    def test_a_failure_blocks_what_depends_on_it_and_nothing_else(self):
        graph = self.graph()
        for name in graph.names(READY):
            graph.start(name)
        blocked = graph.fail('r1_clear_r2', 'R1 stopped in front of an obstacle')
        self.assertEqual(sorted(blocked), ['r2_arrive', 'r2_scan', 'team_report'])
        self.assertEqual(graph.state['r2_arrive'], BLOCKED)
        self.assertIn('r1_clear_r2', graph.note['r2_arrive'])
        self.assertEqual(graph.state['r3_arrive'], RUNNING)                      # R3 has nothing to do with it
        graph.succeed('r3_arrive'); graph.start('r3_scan'); graph.succeed('r3_scan')
        graph.fail('r1_arrive')
        self.assertTrue(graph.finished())
        self.assertEqual(graph.status(), FAILED)
        self.assertEqual(graph.state['r1_scan'], BLOCKED)

    def test_stop_cancels_everything_that_has_not_finished(self):
        graph = self.graph()
        graph.start('r3_arrive'); graph.succeed('r3_arrive')
        graph.cancel('Operator STOP')
        self.assertEqual(graph.state['r3_arrive'], SUCCEEDED)
        self.assertEqual(graph.state['r1_arrive'], CANCELLED)
        self.assertEqual(graph.state['team_report'], CANCELLED)
        self.assertEqual(graph.status(), CANCELLED)

    def test_a_graph_with_a_circle_is_refused(self):
        engine = dag_engine()
        with self.assertRaises(ValueError):
            LiveGraph([engine.Task('a', ('b',)), engine.Task('b', ('a',))])


if __name__ == '__main__':
    unittest.main()
