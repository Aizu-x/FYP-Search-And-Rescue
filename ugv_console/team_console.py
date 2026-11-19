"""The team simulator of the console: 2 to 4 simulated robots in one arena.

This is the part of ``ConsoleApp`` that deals with a team (the class below is mixed into it). The robots, the
order they drive in and the supervisor are in ``ugv_nav/team_sim.py`` and ``ugv_nav/team_plan.py``; here are
the operator's commands and what the page is shown.

SIMULATOR ONLY: a team is only ever built from simulated robots (``TeamSim``). There is no command here that
connects to a real robot, and the team view is only offered in Simulator mode.

How it is used:

* Simulator mode, "Robots in the simulator" = 2, 3 or 4 (``sim_robots``). With 1 the console is the single-robot
  simulator it always was, and nothing in this file is active.
* Every robot gets a starting place (``team_place``) and a goal (``team_goal``). ``team_start`` builds the
  simulated robots, ``team_plan`` plans the routes and the order (Kahn's algorithm on the task graph).
* The arena the team drives in is the operator map as it is drawn, edits included: what the page shows is what
  the robots meet. (The single-robot simulator drives in the data set's ground truth instead, which lets it show
  what a wrong map does; for a team that only hides walls from the operator.)
* Driving needs the operator's authorization, in two steps like the single robot's AUTHORIZE
  (``team_run_prepare`` -> the plan in words + a one-time code, ``team_run_confirm`` -> go).
* STOP stops every robot. One moving machine at a time still holds: no team start while the drone is in the
  air, and the drone does not take off while a team robot moves (``ConsoleApp.is_moving``).
"""
from __future__ import annotations

import dataclasses
import hmac
import math
import secrets
import time

from ugv_nav.navigator import NavigatorConfig
from ugv_nav.path_processing import validate_path
from ugv_nav.team_plan import TeamError, label
from ugv_nav.team_sim import COLOURS, MAX_ROBOTS, TeamSim

from .options import SIM, arg_number

RUN_CODE_S = 60.0                    # the "start the team?" question is valid this long
LATTICE_M = 0.15                     # default starting places are looked for on this raster


def _result(ok, message='', level=None, **extra):
    return dict(ok=bool(ok), message=message, level=level or ('info' if ok else 'error'), **extra)


class TeamConsole:
    # ------------------------------------------------------------------ state
    def _team_init(self):
        self.team = None                # ugv_nav.team_sim.TeamSim while the team simulator runs
        self.team_starts = {}           # robot number -> (x, y, heading in degrees)
        self.team_goals = {}            # robot number -> (x, y)
        self._team_for = None           # the environment those places were chosen for
        self._team_code = None          # (one-time code, expiry, signature of the plan it was shown for)

    def team_selected(self):
        """Is the simulator set to more than one robot?"""
        return self.opts['mode'] == SIM and int(self.opts['sim_robots']) >= 2

    def team_moving(self):
        team = self.team
        return bool(team is not None and team.busy())

    def _team_say(self, text, level='info'):
        """What the team supervisor reports (its own thread): into the activity list."""
        self._log_line(level, text)
        self.revs['team'] += 1

    def _team_close(self):
        team, self.team = self.team, None
        self._team_code = None
        if team is not None:
            team.close()
            self.revs['team'] += 1

    def _team_stop(self, reason):
        """STOP for the team; never waits. -> was a team there to stop?"""
        team = self.team
        self._team_code = None
        if team is None:
            return False
        team.stop(reason)
        self.revs['team'] += 1
        return True

    # ------------------------------------------------------------------ where the robots stand
    def _team_size(self):
        """Make sure every robot of the team has a starting place in this environment. -> how many robots"""
        size = max(2, min(MAX_ROBOTS, int(self.opts['sim_robots'])))
        if self._team_for != self.current_env:
            self.team_starts, self.team_goals, self._team_for = {}, {}, self.current_env
        for number in [n for n in self.team_starts if n > size]:
            del self.team_starts[number]
        for number in [n for n in self.team_goals if n > size]:
            del self.team_goals[number]
        missing = [n for n in range(1, size + 1) if n not in self.team_starts]
        if missing:
            taken = [self.team_starts[n][:2] for n in sorted(self.team_starts)]
            for number, place in zip(missing, self._team_default_places(len(missing), taken)):
                self.team_starts[number] = place
        return size

    def _team_default_places(self, count, taken):
        """``count`` free places far from each other (and from ``taken``), each facing the middle of the arena.

        Looked for on a coarse raster, among the places a robot can stand with some room to spare, in the
        largest part of the arena that hangs together (a robot that starts in a closed-off corner could not
        go anywhere). Greedy: the place farthest from all places chosen so far, starting bottom-left.
        """
        grid = self.grid
        radius = self.planner_config().planning_radius + 0.06
        x0, x1, y0, y1 = grid.extent
        nx, ny = int((x1 - x0) / LATTICE_M), int((y1 - y0) / LATTICE_M)
        point = lambda i, j: (x0 + (i + 0.5) * LATTICE_M, y0 + (j + 0.5) * LATTICE_M)      # noqa: E731
        free = {(i, j) for i in range(nx) for j in range(ny) if validate_path([point(i, j)], grid, radius)[0]}
        best, seen = set(), set()
        for cell in sorted(free):                           # largest connected part
            if cell in seen:
                continue
            part, todo = {cell}, [cell]
            while todo:
                i, j = todo.pop()
                for near in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                    if near in free and near not in part and validate_path([point(i, j), point(*near)], grid, radius)[0]:
                        part.add(near)
                        todo.append(near)
            seen |= part
            if len(part) > len(best):
                best = part
        places = [point(*cell) for cell in sorted(best)]
        chosen, out = [tuple(p) for p in taken], []
        centre = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        for _ in range(count):
            if not places:
                break
            if chosen:
                place = max(places, key=lambda p: (round(min(math.dist(p, c) for c in chosen), 3), -p[0] - p[1]))
            else:
                place = min(places, key=lambda p: (round(p[0] + p[1], 3), p[0]))
            chosen.append(place)
            heading = round(math.degrees(math.atan2(centre[1] - place[1], centre[0] - place[0])) / 45.0) * 45 % 360
            out.append((round(place[0], 3), round(place[1], 3), float(heading)))
        while len(out) < count:                             # an arena with hardly any floor: let the operator place them
            out.append((round(centre[0], 3), round(centre[1], 3), 0.0))
        return out

    # ------------------------------------------------------------------ commands: setting up
    def cmd_team_start(self, args):
        """Build the simulated robots at their starting places."""
        with self._lock:
            if not self.team_selected():
                return self._fail('Choose Simulator mode and 2 to 4 robots first.')
            self._disconnect()                              # the single simulator, or the team that was running
            size = self._team_size()
            o = self.opts
            try:
                base = self.edge_cfg.navigator if self.edge_cfg else NavigatorConfig()
                room_mode = o['lidar_sees'] == 'room'
                # (the same settings the single simulator gets in cmd_connect; a team never continues on a new
                # route by itself, the supervisor decides that)
                localization = dataclasses.replace(base.localization, ignore_drift=o['ignore_drift'])
                nav_cfg = dataclasses.replace(base, safety=self._safety_config(), localization=localization,
                                              auto_continue_after_replan=False, allow_tight_radius=o['allow_tight'],
                                              smooth_path=o['smooth_path'], lidar_correction=o['lidar_correction'],
                                              auto_find=o['auto_find'], lidar_reference='room' if room_mode else 'map')
                starts = [(x, y, math.radians(heading))
                          for x, y, heading in (self.team_starts[n] for n in range(1, size + 1))]
                self.team = TeamSim(self.grid, self.grid, self.planner_config(), nav_cfg, starts,
                                    nav_cfg.safety.half_length_m, nav_cfg.safety.half_width_m, room_mode=room_mode,
                                    speed=int(o['robot_speed']) / 100.0,
                                    min_confidence=o['yolo_min_conf'] / 100.0, say=self._team_say)
            except (TeamError, ValueError) as exc:
                self.team = None
                self._bump('team')
                return self._fail('The team simulator did not start: %s' % exc)
            self.team.env = self.current_env
            self._bump('team', 'map', 'plan', 'settings')
            return self._ok('Team simulator started: %d simulated robots. Give every robot a goal, then plan.' % size,
                            'success')

    def cmd_team_end(self, args):
        with self._lock:
            was = self.team is not None
            self._team_close()
            self._bump('team')
            return self._ok('Team simulator stopped.' if was else 'The team simulator is not running.')

    def _team_robot(self, args):
        number = int(arg_number(args, 'robot', 1, MAX_ROBOTS))
        if number > self._team_size():
            raise ValueError('There is no %s: the team has %d robots.' % (label(number), self._team_size()))
        return number

    def cmd_team_place(self, args):
        """Where a robot starts: x, y and optionally its heading in degrees."""
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        heading = arg_number(args, 'heading', -360, 360, required=False)
        with self._lock:
            if not self.team_selected():
                return self._fail('Choose Simulator mode and 2 to 4 robots first.')
            number = self._team_robot(args)
            if not self.grid.contains(self.grid.world_to_cell((x, y))):
                return self._fail('That point is outside the map.')
            if heading is None:
                heading = self.team_starts[number][2]
            place = (round(x, 3), round(y, 3), float(heading) % 360)
            if self.team is not None:
                try:
                    self.team.place(number, (place[0], place[1], math.radians(place[2])))
                except TeamError as exc:
                    return self._fail(str(exc))
            self.team_starts[number] = place
            self._team_code = None
            self._bump('team')
            return self._ok('%s starts at (%.2f, %.2f), facing %.0f°.' % (label(number), place[0], place[1], place[2]))

    def cmd_team_goal(self, args):
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        with self._lock:
            if not self.team_selected():
                return self._fail('Choose Simulator mode and 2 to 4 robots first.')
            number = self._team_robot(args)
            if self.team_moving():
                return self._fail('The team is driving: stop it before changing a goal.')
            if not self.grid.contains(self.grid.world_to_cell((x, y))):
                return self._fail('That point is outside the map.')
            if self.grid.cells[self.grid.world_to_cell((x, y))]:
                return self._fail('That point is inside a wall.')
            self.team_goals[number] = (round(x, 3), round(y, 3))
            self._team_code = None
            if self.team is not None:
                self.team.forget_plan()
            self._bump('team')
            return self._ok('%s goes to (%.2f, %.2f).' % (label(number), x, y))

    def cmd_team_clear_goals(self, args):
        """Remove the goals: of one robot (``robot``), or of all. A robot without a goal stays where it is."""
        with self._lock:
            if self.team_moving():
                return self._fail('The team is driving: stop it first.')
            number = self._team_robot(args) if args.get('robot') is not None else None
            if number is None:
                self.team_goals = {}
            else:
                self.team_goals.pop(number, None)
            self._team_code = None
            if self.team is not None:
                self.team.forget_plan()
            self._bump('team')
            return self._ok('All goals removed.' if number is None else
                            '%s has no goal: it stays where it is.' % label(number))

    def cmd_team_block(self, args):
        """Place a simulated block for the robots' cameras (or remove the one that is there)."""
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        with self._lock:
            if self.team is None:
                return self._fail('Start the team simulator first.')
            try:
                text = self.team.toggle_block(x, y, self.opts['sim_block_colour'])
            except TeamError as exc:
                return self._fail(str(exc))
            self._bump('team')
            return self._ok(text)

    def cmd_team_clear_findings(self, args):
        with self._lock:
            if self.team is not None:
                self.team.clear_findings()
            self._bump('team')
            return self._ok('The list of survivors was cleared.')

    # ------------------------------------------------------------------ commands: planning and driving
    def cmd_team_plan(self, args):
        """Plan every robot's route and the order of the team."""
        with self._lock:
            team = self.team
            if team is None:
                return self._fail('Start the team simulator first.')
            self._team_code = None
            size = self._team_size()
            try:
                config = self.planner_config()
                if config != team.planner_config:
                    team.set_planner(config)
                plan = team.plan({n: self.team_goals.get(n) for n in range(1, size + 1)},
                                 scan=bool(self.opts['team_scan']), approach=bool(self.opts['team_approach']))
            except (TeamError, ValueError) as exc:
                self._bump('team')
                return self._fail(str(exc))
            self._bump('team')
            return self._ok('%s %d tasks in %d waves. Check the routes, then authorize.'
                            % (team.message, len(plan.tasks), len(plan.waves)), 'success')

    def _team_run_problem(self):
        team = self.team
        if team is None:
            return 'Start the team simulator first.'
        if team.busy():
            return 'The team is already driving.'
        if team.plan_result is None or team.phase != 'planned':
            return 'Plan the team mission first.'
        if self.uav_airborne():
            return 'The drone is in the air: land it before the robots drive.'
        return ''

    def cmd_team_run_prepare(self, args):
        """First step of the authorization: say what will happen and hand out a one-time code."""
        with self._lock:
            problem = self._team_run_problem()
            if problem:
                return self._fail(problem)
            plan = self.team.plan_result
            code = secrets.token_hex(8)
            self._team_code = (code, time.monotonic() + RUN_CODE_S, plan.signature)
            lines = []
            for number, wave in enumerate(plan.waves, 1):
                drives = [label(plan.about[name]['robot']) for name in wave if plan.about[name]['kind'] == 'arrive']
                if drives:
                    waits = sorted({label(e.first) for e in plan.edges if label(e.then) in drives})
                    verb = 'start' if len(drives) > 1 else 'starts'
                    lines.append('%s %s%s' % (' and '.join(drives), verb + ' at once' if number == 1 else verb,
                                              (' when %s %s out of the way' % (' and '.join(waits), 'are' if len(waits) > 1 else 'is'))
                                              if waits else ''))
            text = ('%d simulated robots drive to their goals by themselves%s.%s\n\n%s.\n\n'
                    'The order comes from the task graph (Kahn\'s algorithm). STOP (Esc) stops every robot.'
                    % (len(plan.members), ' and look around there' if self.opts['team_scan'] else '',
                       ' A robot that has seen a survivor then drives up to it by itself.'
                       if self.opts['team_approach'] else '', '.\n'.join(lines)))
            return _result(True, '', nonce=code, title='Start the team', text=text, confirm_label='Authorize + start')

    def cmd_team_run_confirm(self, args):
        with self._lock:
            held, self._team_code = self._team_code, None
            code = args.get('nonce')
            if (held is None or not isinstance(code, str) or not hmac.compare_digest(code.encode(), held[0].encode())
                    or time.monotonic() > held[1]):
                return self._fail('That authorization is no longer valid: press the button again.')
            problem = self._team_run_problem()
            if problem:
                return self._fail(problem)
            ok, text = self.team.run(authorized=held[2])       # only the plan the operator was shown
            self._bump('team')
            if not ok:
                return self._fail(text)
            self._log_line('info', 'Team mission AUTHORIZED by the operator.')
            return self._ok(text, 'success')

    def cmd_team_reset(self, args):
        """Put every robot back at its starting place."""
        with self._lock:
            team = self.team
            if team is None:
                return self._fail('Start the team simulator first.')
            if team.busy():
                return self._fail('The team is driving: stop it first.')
            try:
                for number in range(1, self._team_size() + 1):
                    x, y, heading = self.team_starts[number]
                    team.place(number, (x, y, math.radians(heading)))
            except TeamError as exc:
                self._bump('team')
                return self._fail(str(exc))
            self._bump('team')
            return self._ok('Every robot is back at its starting place.')

    # ------------------------------------------------------------------ hooks called by ConsoleApp
    def _team_options_changed(self, keys, before):
        o, message = self.opts, ''
        if 'sim_robots' in keys and o['sim_robots'] != before['sim_robots']:
            if self.is_moving():
                raise ValueError('a robot is moving: stop it first')
            self._disconnect()                              # another number of robots: the running simulator ends
            self._bump('team', 'map', 'plan')
            if o['mode'] == SIM and o['sim_robots'] >= 2:
                self._team_size()
                message = ('%d robots: put each one where it should start, give each a goal, then start the team '
                           'simulator.' % o['sim_robots'])
            else:
                message = 'One robot: the simulator as it always was.'
        if 'team_scan' in keys or 'team_approach' in keys:
            if self.team_moving():
                raise ValueError('the team is driving: stop it first')
            if self.team is not None:
                self.team.forget_plan()
            self._bump('team')
            if 'team_scan' in keys:
                message = ('Every robot looks around when it has reached its goal.' if o['team_scan'] else
                           'The robots stop at their goals without looking around.')
            else:
                message = ('A robot that has seen a survivor drives up to it after its other tasks.'
                           if o['team_approach'] else 'The robots report survivors and do not drive to them.')
        if 'robot_speed' in keys and self.team is not None:
            self.team.set_speed(int(o['robot_speed']) / 100.0)
        return message

    def _team_tick(self):
        """Housekeeping (console ticker, holding the console lock): follow map and environment changes."""
        team = self.team
        if team is None:
            return
        if getattr(team, 'env', self.current_env) != self.current_env or not self.team_selected():
            self._team_close()                              # another arena, or the team view was left
            self._log_line('info', 'Team simulator stopped (the arena or the mode changed).')
        elif team.grid is not self.grid:
            team.set_map(self.grid)
            self._log_line('info', 'The map changed: the team plans on the new map.')

    def team_frame(self, number, after=None, timeout=1.5):
        """Newest camera picture of one team robot (as VisionWorker.wait_frame), or None."""
        team = self.team
        if team is None:
            return None
        try:
            return team.frame(number, after, timeout)
        except TeamError:
            return None

    # ------------------------------------------------------------------ what the page is shown
    def _team_run_view(self):
        problem = self._team_run_problem() if self.team is not None else 'Start the team simulator first.'
        return dict(enabled=not problem, hint=problem)

    def _view_team(self):
        selected = self.team_selected()
        if not selected and self.team is None:
            return dict(selected=False, size=1, running=False)
        size = self._team_size()
        team = self.team
        out = dict(selected=selected, size=size, running=team is not None, max=MAX_ROBOTS, colours=list(COLOURS[:size]),
                   starts=[list(self.team_starts[n]) for n in range(1, size + 1)],
                   goals=[list(self.team_goals[n]) if n in self.team_goals else None for n in range(1, size + 1)],
                   scan=bool(self.opts['team_scan']), approach=bool(self.opts['team_approach']),
                   run=self._team_run_view(),
                   block_colour=self.opts['sim_block_colour'], env=self.current_env)
        if team is not None:
            out.update(team.snapshot(wait=0.05))
        else:
            out.update(phase='setup', message='', tasks=[], waves=[], edges=[], planned=[], findings=[], blocks=[],
                       report=None, camera=False,
                       robots=[dict(id=n, label=label(n), colour=COLOURS[n - 1],
                                    pose=[self.team_starts[n][0], self.team_starts[n][1],
                                          math.radians(self.team_starts[n][2])],
                                    goal=list(self.team_goals[n]) if n in self.team_goals else None, state='OFF',
                                    motion=None, message='', note='', task='', camera=False, collided=False, path=[],
                                    pins=[]) for n in range(1, size + 1)])
        return self.clean(out)
