"""A team of simulated robots (2 to 4) in one arena, driven in the order a task graph allows.

SIMULATOR ONLY. Every robot here is a ``SimBackend`` on this computer; nothing in this file can reach a real
robot. Each team robot is a complete copy of what the single simulator runs: the real bridge server, the real
edge client, a ``Navigator`` and (with OpenCV) a simulated camera with its detector. The robots are
independent programs that happen to share a floor, which is what several real robots would be.

What the team adds:

* ``plan`` lets every navigator plan its own route on its own copy of the map, in which the places where the
  other robots stand and will park are drawn as obstacles. ``team_plan.build_plan`` then works out which
  routes are in each other's way and orders the robots with Kahn's algorithm.
* ``run`` and the supervisor thread (``step``, 10 Hz) carry the task graph out: a robot is authorized when its
  tasks are released, its tasks succeed when it has driven clear of a waiting robot's route / arrived / looked
  around, and a robot that stops for good fails its tasks, which blocks the robots waiting for it.
* The robots cannot drive through each other (``TeamSimBackend``) and see each other in their cameras
  (``TeamSimCamera``). Their LiDARs do not see each other: like the arena walls, a robot is lower than the
  LiDAR plane of the others. A last check on the laptop stops robots that get too close anyway.
* With ``approach`` a robot that has seen a survivor (a green block) drives up to it after its other tasks,
  faces it and stands there for a moment: the survivor is "secured" (``_rescue_task``). Its way there is
  checked against what the other robots still have to drive, and it waits until that way is free.
* A team does not give up at the first trouble. A robot that stops for something it can get over (an obstacle,
  a path-following fault) plans again and drives on, a few times. When a robot has stopped for good, the robots
  that were waiting for it go on if it is not standing in their way, or take another way around it; only when
  there is no other way are they blocked.

The simulated robots of a team have exact odometry, and the scale calibration of the real robot (console
settings "X scale" / "Y scale") is not applied to them: a team is about who drives when, and a calibration
measured on the real robot's wheels says nothing about simulated ones.
"""
from __future__ import annotations

import dataclasses
import itertools
import math
import threading
import time

import numpy as np

from ugv_bridge.protocol import generate_key_hex, parse_key
from ugv_bridge.server import BridgeConfig, BridgeServer
from ugv_planner.collision import CollisionChecker
from ugv_planner.grid import Grid

from .edge_client import EdgeClient
from .mission import MissionConfig
from .navigator import ARRIVED, IDLE, PLANNED, RUNNING, STOPPED, Navigator
from .sim_backend import SimBackend, simulated_room
from .team_plan import (BLOCKED, CANCELLED, FAILED, READY, SUCCEEDED, LiveGraph, Member, TeamError, ahead,
                        build_plan, label, path_gap, point_gap)
from .team_plan import RUNNING as TASK_RUNNING

MAX_ROBOTS = 4
COLOURS = ('#4da3ff', '#2dd4bf', '#b46bff', '#ff5fd2')        # R1 blue, R2 teal, R3 purple, R4 pink (as on the page)
SIM_CAMERA = dict(pitch_rad=math.radians(15), x_offset_m=0.08)   # the simulated camera's true mounting
SETUP, READY_PHASE, PLANNED_PHASE, RUNNING_PHASE, DONE, FAILED_PHASE, STOPPED_PHASE = (
    'setup', 'ready', 'planned', 'running', 'done', 'failed', 'stopped')
RETRIES = 3                          # how often a robot may plan again and drive on during one mission
CAN_RETRY = ('obstacle', 'replan', 'tracking', 'fault')    # navigator stop kinds a robot can get over by itself
NO_PROGRESS_S = 8.0                  # a driving robot that has not moved or turned for this long is stuck
MISSION = MissionConfig()            # the single robot's search mission: how near to a survivor, how long, how fast to turn
RESCUE_WAIT_S = 90.0                 # a robot gives a survivor up when the way to it has not become free in this time
REPLANS = 4                          # new routes a robot may try from one and the same place before it gives up


def with_discs(grid, discs):
    """The map with round obstacles added: ``discs`` is a list of ((x, y), radius)."""
    if not discs:
        return grid
    cells = np.array(grid.cells, dtype=np.uint8)
    height, width = cells.shape
    xs = grid.origin[0] + (np.arange(width) + 0.5) * grid.resolution
    ys = grid.origin[1] + (np.arange(height) + 0.5) * grid.resolution
    for (x, y), radius in discs:
        cells[(ys[:, None] - y) ** 2 + (xs[None, :] - x) ** 2 <= radius * radius] = 1
    return Grid(cells, grid.resolution, grid.origin)


class TeamSimBackend(SimBackend):
    """A simulated robot that shares the floor: it cannot drive through the other robots of its team."""

    def __init__(self, *args, **kwargs):
        self.others = []            # [(x, y, radius)] where the other robots are right now (set by TeamSim)
        super().__init__(*args, **kwargs)

    def _hits_obstacle(self, pose):
        if any(math.hypot(pose[0] - x, pose[1] - y) <= radius + self.body_radius for x, y, radius in self.others):
            return True
        return super()._hits_obstacle(pose)


def _bgr(colour):
    value = colour.lstrip('#')
    return tuple(int(value[k:k + 2], 16) for k in (4, 2, 0))


def make_team_camera(backend, blocks, camera, mates, rate_hz=10.0):
    """A simulated camera that also shows the other robots: ``mates()`` -> [(label, (x, y), colour)]."""
    from .vision_sim import SimCamera

    class TeamSimCamera(SimCamera):
        def things(self, pose):                             # the blocks, and the team-mates as boxes with a name
            return super().things(pose) + [(x, y, 0.15, 0.11, _bgr(colour), name) for name, (x, y), colour in mates()]

    return TeamSimCamera(backend, blocks, camera, rate_hz)


class TeamRobot:
    """One simulated robot of the team: backend, bridge, client, navigator, camera."""

    def __init__(self, robot_id, world, pose, half_length, half_width, sim_kwargs):
        self.id, self.label, self.colour = robot_id, label(robot_id), COLOURS[robot_id - 1]
        self.goal = None
        self.nav = self.vision = None
        self.released = False           # authorized for this run
        self.spin_from = None           # navigator halt count when its look-around began
        self.replanned_at = None        # when it was first seen waiting with a new route
        self.note = ''
        self.discs = []                 # the other robots' places drawn into its map right now
        self.plan_discs = []            # ... as they were when the team was planned
        self.rescue = None              # what it is doing about the survivors it found (see _rescue_task)
        self.spot = None                # where it stands, or drives to, after it has left its goal
        self.gave_up = set()            # survivors it could not get to in this mission
        self.short = None               # why it will not get to its goal, when it goes to a survivor instead
        self.replan_from = None         # where it last continued on a new route, and how often from there
        self.replans = 0
        self.retries = 0                # times it planned again and drove on in this mission
        self.retry_at = None            # when the next such try is due
        self.failed = False             # it stopped for good in this mission
        self.touched_at = -1e9          # last time it touched something in the simulator
        self.progress = None            # (time, x, y, yaw): where it last made headway
        self.key = parse_key(generate_key_hex())
        values = dict(bind_host='127.0.0.1', port=0, half_length_m=half_length, half_width_m=half_width,
                      max_vx=0.25, max_wz=1.0)
        self.backend = TeamSimBackend(world, pose, body_radius=max(half_length, half_width), **sim_kwargs)
        self.server = BridgeServer(BridgeConfig(values), self.backend, self.key).start()
        self.client = EdgeClient('127.0.0.1', self.server.port, self.key, 'team-sim-%d' % robot_id,
                                 link_timeout=0.6).start()

    def position(self):
        """Where the navigator believes the robot is, (x, y); the true place until it knows."""
        fast = self.nav.fast_state() if self.nav is not None else None
        pose = fast['pose'] if fast and fast['pose'] is not None else self.backend.truth_pose()
        return float(pose[0]), float(pose[1])

    def pose(self):
        fast = self.nav.fast_state() if self.nav is not None else None
        pose = fast['pose'] if fast and fast['pose'] is not None else self.backend.truth_pose()
        return float(pose[0]), float(pose[1]), float(pose[2])

    def close(self):
        if self.vision is not None:
            self.vision.stop()
            self.vision = None
        if self.nav is not None:
            self.nav.emergency_stop('Team simulator closed')
            self.nav.shutdown()
        self.client.close()
        self.server.shutdown()


class TeamSim:
    def __init__(self, world, grid, planner_config, nav_config, starts, half_length=0.08, half_width=0.07,
                 room_mode=False, speed=1.0, camera=True, min_confidence=0.5, say=None, clock=time.monotonic):
        """``starts``: [(x, y, yaw in radians)] for R1, R2, ... ``say(text, level)`` receives what happens."""
        if not 1 <= len(starts) <= MAX_ROBOTS:
            raise TeamError('A team has 1 to %d robots.' % MAX_ROBOTS)
        self.world, self.grid, self.planner_config = world, grid, planner_config
        self.follows_map = world is grid        # the simulated arena IS the operator map (the console's choice)
        self.nav_config = dataclasses.replace(nav_config, auto_continue_after_replan=False)
        self.half_length, self.half_width, self.room_mode = half_length, half_width, room_mode
        self.speed, self.camera, self.min_confidence = speed, camera, min_confidence
        self.say = say or (lambda text, level='info': None)
        self.clock = clock
        self.body = math.hypot(half_length, half_width)
        self.radius = planner_config.planning_radius
        self.distance = 2 * self.radius + 0.08          # routes closer than this are in each other's way
        self.keep = self.radius + 0.12                  # obstacle drawn where another robot stands / will park
        self.apart = self.keep + self.radius + grid.resolution     # how far apart two starts (two goals) must be
        self.too_close = 2 * self.body + 0.02           # the last check: stop robots that come this close
        self.robots = []
        self.blocks = None
        self.plan_result = None
        self.graph = None
        self.phase = SETUP
        self.message = ''
        self.findings = []              # survivors seen by any robot: dict(id, x, y, by, t)
        self.report = None
        self.detours = {}               # clear task -> (robot that stopped, robot that has to get around it)
        self._scan = True
        self._approach = False
        self._last_snapshot = None
        self.rev = 0                    # changes whenever something worth redrawing changed
        self._lock = threading.RLock()
        self._stop_thread = threading.Event()
        self._stopping = None           # the reason of a STOP that is being carried out (see stop)
        self._thread = None
        self._started = None
        self._open(list(starts))

    # ------------------------------------------------------------------ setting up
    def check_places(self, points, what='robots'):
        """Raise TeamError unless all points are inside the map and far enough from each other."""
        for k, point in enumerate(points):
            if point is None:
                continue
            if not self.grid.contains(self.grid.world_to_cell(point[:2])):
                raise TeamError('%s: that point is outside the map.' % label(k + 1))
            for j in range(k):
                if points[j] is not None and math.dist(point[:2], points[j][:2]) < self.apart:
                    raise TeamError('%s and %s are too close together: the %s need %.2f m between them.'
                                    % (label(j + 1), label(k + 1), what, self.apart))

    def _open(self, starts):
        self.check_places(starts, 'starting places')
        sim = dict(odom_scale_error=0.0)     # exact odometry: see the note at the top of this file
        if self.room_mode:              # like the MentorPi: the LiDAR looks over the arena into the room
            sim.update(lidar_world=simulated_room(self.world), scan_noise_m=0.008)
        try:
            for k, start in enumerate(starts):
                self.robots.append(TeamRobot(k + 1, self.world, tuple(start), self.half_length, self.half_width, sim))
            deadline = time.monotonic() + 8.0
            while not all(robot.client.link_ok() for robot in self.robots):
                if time.monotonic() > deadline:
                    raise TeamError('The simulated robots did not all connect.')
                time.sleep(0.05)
            time.sleep(0.25)
            for robot, start in zip(self.robots, starts):
                own = dataclasses.replace(self.nav_config, safety=dataclasses.replace(self.nav_config.safety),
                                          localization=dataclasses.replace(self.nav_config.localization))
                nav = Navigator(robot.client, self.grid, self.planner_config, own)
                nav.set_scale(1.0, 1.0)         # (the real robot's calibration is not for simulated wheels)
                nav.set_speed_fraction(self.speed)
                if self.room_mode:      # the simulated LiDAR sits exactly in the centre: nothing to learn
                    nav.set_lidar_mount((0.0, 0.0, 0.0), True)
                try:
                    nav.set_initial_pose(tuple(start), source='dock')
                except ValueError as exc:
                    raise TeamError('%s cannot stand there: %s' % (robot.label, exc)) from None
                robot.nav = nav
            for robot in self.robots:
                robot.nav.run_in_background()
            if self.camera:
                self._start_cameras()
        except Exception:
            self.close()
            raise
        self.phase = READY_PHASE
        self._thread = threading.Thread(target=self._loop, name='team-supervisor', daemon=True)
        self._thread.start()

    def _start_cameras(self):
        try:
            from .vision import CameraModel, ObstacleTracker, TrackerConfig, VisionWorker
            from .vision_sim import SimBlocks, SimDetector
            import cv2  # noqa: F401
        except ImportError:
            self.camera = False
            self.say('The team cameras need the extra packages (requirements-yolo.txt); driving works without them.',
                     'warn')
            return
        cam = CameraModel(**SIM_CAMERA)
        self.blocks = SimBlocks()
        for robot in self.robots:
            robot.backend.blocks = self.blocks
            robot.backend.camera_source = make_team_camera(robot.backend, self.blocks, cam, self._mates_of(robot))
            detector = SimDetector(robot.backend, self.blocks, cam, noise_px=0.7, seed=robot.id)
            robot.client.request_camera(6)
            tracker = ObstacleTracker(TrackerConfig(min_confidence=self.min_confidence))
            robot.vision = VisionWorker(robot.client, lambda nav=robot.nav: nav, detector, cam, tracker).start()

    def _mates_of(self, robot):
        def mates():
            return [(other.label, other.backend.truth_pose()[:2], other.colour)
                    for other in self.robots if other is not robot]
        return mates

    def close(self):
        self._stop_thread.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2)
        for robot in self.robots:
            try:
                robot.close()
            except Exception:
                pass
        self.robots = []
        self.phase = SETUP

    def robot(self, robot_id):
        for robot in self.robots:
            if robot.id == robot_id:
                return robot
        raise TeamError('There is no %s in this team.' % label(robot_id))

    def moving(self):
        return any(robot.nav is not None and robot.nav.state == RUNNING for robot in self.robots)

    def busy(self):
        return self.phase == RUNNING_PHASE or self.moving()

    def place(self, robot_id, pose):
        """Put a robot somewhere else (x, y, yaw in radians). Not while the team is driving."""
        with self._lock:
            if self.busy():
                raise TeamError('Stop the team before moving a robot.')
            robot = self.robot(robot_id)
            points = [other.position() if other is not robot else tuple(pose[:2]) for other in self.robots]
            self.check_places(points, 'robots')
            robot.nav.set_map(self.grid)                    # the plain map: no obstacles of an old plan
            before = robot.backend.truth_pose()
            robot.backend.place(tuple(pose))
            time.sleep(0.15)                                # the next odometry sample is from the new place
            try:
                robot.nav.set_initial_pose(tuple(pose), source='dock')
            except ValueError as exc:
                robot.backend.place(before)
                time.sleep(0.15)
                robot.nav.set_initial_pose(before, source='dock')
                raise TeamError('%s cannot stand there: %s' % (robot.label, exc)) from None
            robot.backend.collided = False
            self._forget_plan()

    def _forget_plan(self):
        self.plan_result, self.graph, self.report = None, None, None
        self.phase = READY_PHASE
        self.message = ''
        self.rev += 1

    def forget_plan(self):
        """Something the plan was made for has changed (a goal, a setting). Not while the team drives."""
        with self._lock:
            if self.phase != RUNNING_PHASE:
                self._forget_plan()

    def set_map(self, grid):
        """The operator changed the map: every robot plans on the new one. A driving team is stopped."""
        if self.busy():
            self.stop('The map was changed')
        with self._lock:
            self.grid = grid
            if self.follows_map:                    # the arena the robots drive in changes with the map
                self.world = grid
            for robot in self.robots:
                if self.follows_map:
                    robot.backend.world = grid
                    robot.backend.checker = CollisionChecker(grid)
                robot.nav.set_map(grid)
            self._forget_plan()

    def set_planner(self, planner_config):
        """New robot size / clearance settings. Not while the team drives."""
        with self._lock:
            if self.busy():
                raise TeamError('Stop the team before changing the planner settings.')
            self.planner_config = planner_config
            self.radius = planner_config.planning_radius
            self.distance = 2 * self.radius + 0.08
            self.keep = self.radius + 0.12
            self.apart = self.keep + self.radius + self.grid.resolution
            for robot in self.robots:
                robot.nav.set_planner_config(planner_config)
            self._forget_plan()

    def set_speed(self, fraction):
        self.speed = fraction
        for robot in list(self.robots):
            if robot.nav is not None:
                robot.nav.set_speed_fraction(fraction)

    def clear_findings(self):
        """Forget the survivors seen so far (what the cameras still see is found again)."""
        with self._lock:
            self.findings = []
            self.rev += 1

    def toggle_block(self, x, y, colour):
        """Place a simulated block, or remove the one that is already there. -> text"""
        if self.blocks is None:
            raise TeamError('Test blocks need the team cameras (requirements-yolo.txt).')
        for block_id, (bx, by, _cls) in self.blocks.items():
            if math.hypot(bx - x, by - y) < 0.06:
                self.blocks.remove(block_id)
                self.rev += 1
                return 'Block removed at (%.2f, %.2f).' % (bx, by)
        self.blocks.add(x, y, '%s_obstacle' % colour)
        self.rev += 1
        return 'A %s block was placed at (%.2f, %.2f) (click it again to remove it).' % (colour, x, y)

    # ------------------------------------------------------------------ planning
    def plan(self, goals, scan=True, approach=False):
        """Plan the routes and the order of the team. ``goals``: {robot id: (x, y)}; a robot without a goal
        stays where it is and is an obstacle for the others. ``scan``: every robot looks around at its goal.
        ``approach``: after that it drives up to the survivors it has seen. -> TeamPlan"""
        with self._lock:
            if self.busy():
                raise TeamError('Stop the team before planning again.')
            self._forget_plan()
            active = [robot for robot in self.robots if goals.get(robot.id) is not None]
            if not active:
                raise TeamError('Set a goal for at least one robot first.')
            self.check_places([tuple(goals[robot.id]) if goals.get(robot.id) is not None else None
                               for robot in self.robots], 'goals')
            places = {robot.id: robot.position() for robot in self.robots}
            for robot in self.robots:
                robot.goal = tuple(goals[robot.id]) if robot in active else None
                robot.spot, robot.rescue, robot.short = None, None, None
                if robot not in active:
                    robot.nav.set_map(self.grid)
                    robot.discs, robot.plan_discs = [], []
            # Up to four candidate routes per robot. The first treats the places where the other robots stand
            # and will park as obstacles. The others may pass a place another robot will have left (its
            # start), or has not reached yet (its goal), or both: the order of the team then has to make
            # that true, which is exactly what team_plan.precedence checks.
            candidates = {}
            for robot in active:
                routes = [self._route(robot, choice, places, goals) for choice in self.CHOICES]
                if not any(routes):
                    raise TeamError('%s has no route to its goal: %s' % (robot.label, robot.nav.message))
                candidates[robot.id] = routes
            # The combination with the fewest liberties that Kahn's algorithm can put in an order wins.
            best, problem = None, None
            combos = sorted(itertools.product(range(len(self.CHOICES)), repeat=len(active)),
                            key=lambda combo: (sum(combo), combo))
            seen = set()
            for combo in combos:
                paths = tuple(candidates[robot.id][k] for robot, k in zip(active, combo))
                if None in paths or paths in seen:
                    continue
                seen.add(paths)
                members = [Member(robot.id, places[robot.id], robot.goal, path) for robot, path in zip(active, paths)]
                try:
                    best = (combo, build_plan(members, self.distance, scan=scan, approach=approach))
                    break
                except TeamError as exc:
                    problem = problem or exc
            if best is None:
                raise problem or TeamError('No order of the robots works for these starts and goals.')
            combo, plan = best
            # Every navigator now plans the chosen route once more, so that what it holds is what was checked.
            # A camera that has just seen a block changes its robot's map at that very moment, and then the
            # route comes out differently: the order is worked out again for the routes the robots really hold.
            for _ in range(4):
                paths = [self._route(robot, self.CHOICES[k], places, goals) for robot, k in zip(active, combo)]
                if None in paths:
                    lost = active[paths.index(None)]
                    raise TeamError('%s lost its route while the team was being planned: %s' % (lost.label, lost.nav.message))
                if all(path == plan.member(robot.id).path for robot, path in zip(active, paths)):
                    break
                plan = build_plan([Member(robot.id, places[robot.id], robot.goal, path)
                                   for robot, path in zip(active, paths)], self.distance, scan=scan, approach=approach)
            else:
                raise TeamError('The routes keep changing (a camera is still working out what it sees): '
                                'plan again in a moment.')
            self.plan_result, self._scan, self._approach = plan, scan, approach
            self.phase = PLANNED_PHASE
            self.message = self._plan_text(plan)
            parked = [robot.label for robot in self.robots if robot not in active]
            if parked:
                self.message += ' %s %s where %s (no goal).' % (' and '.join(parked), 'stay' if len(parked) > 1 else 'stays',
                                                                 'they are' if len(parked) > 1 else 'it is')
            self.rev += 1
            return plan

    CHOICES = ((True, True), (False, True), (True, False), (False, False))     # (others' starts, others' goals) as obstacles

    def _route(self, robot, choice, places, goals):
        """Let the robot's own navigator plan on its own map. -> the route as a tuple of points, or None"""
        keep_starts, keep_goals = choice
        others = [other for other in self.robots if other is not robot]
        moving = [o for o in others if goals.get(o.id) is not None]
        discs = [(places[o.id], self.keep) for o in others if o not in moving]          # a robot that stays put
        discs += [(places[o.id], self.keep) for o in moving if keep_starts]
        discs += [(tuple(goals[o.id]), self.keep) for o in moving if keep_goals]
        robot.discs, robot.plan_discs = discs, list(discs)
        robot.nav.set_map(with_discs(self.grid, discs))
        robot.nav.set_goal(robot.goal)
        if robot.nav.state == PLANNED and robot.nav.path:
            return tuple((float(x), float(y)) for x, y in robot.nav.path)
        return None

    @staticmethod
    def _plan_text(plan):
        if not plan.edges:
            return 'Planned: no robot is in another one\'s way, they all start together.'
        waits = ', '.join('%s after %s' % (label(e.then), label(e.first)) for e in plan.edges)
        return 'Planned. Order from the task graph: %s.' % waits

    # ------------------------------------------------------------------ running
    def run(self, authorized=None):
        """Carry the plan out. The caller has the operator's authorization; ``authorized`` is the signature of
        the plan the operator was shown (``TeamPlan.signature``), and anything else is refused. -> (ok, text)"""
        with self._lock:
            if self.plan_result is None or self.phase != PLANNED_PHASE:
                return False, 'Plan the team mission first.'
            if not self._plan_still_holds():                # the routes the robots hold right now
                return False, self.message
            plan = self.plan_result
            if authorized is not None and plan.signature != authorized:
                return False, 'The plan changed after you were asked: check it and authorize again.'
            self.graph = LiveGraph(plan.tasks)
            self.report, self.detours = None, {}
            for robot in self.robots:
                robot.released, robot.spin_from, robot.replanned_at, robot.note = False, None, None, ''
                robot.retries, robot.retry_at, robot.failed, robot.progress = 0, None, False, None
                robot.rescue, robot.spot, robot.gave_up, robot.short = None, None, set(), None
                robot.replan_from, robot.replans = None, 0
            for finding in self.findings:                   # nobody is on the way to a survivor yet
                if finding.get('status') != 'secured':
                    finding['claimed'] = None
            self.phase = RUNNING_PHASE
            self._stopping = None
            self._started = self.clock()
            self.message = 'The team is driving.'
            self._release()
            self.rev += 1
            return True, self.message

    def stop(self, reason='Operator STOP'):
        """Stop every robot now. Does not wait for the supervisor.

        The supervisor may be in the middle of a step. It is told first that this is a STOP, so that it does
        not take the robots it finds stopped for robots that failed."""
        self._stopping = reason
        for robot in list(self.robots):
            if robot.nav is not None:
                robot.nav.emergency_stop(reason)
        with self._lock:
            self._finish_stop()
            self.rev += 1

    def _finish_stop(self):
        """(holding the lock) A STOP was asked for: the mission is stopped, not failed."""
        reason, self._stopping = self._stopping, None
        if reason is not None and self.graph is not None and self.phase == RUNNING_PHASE:
            self.graph.cancel(reason)
            self.phase = STOPPED_PHASE
            self.message = 'Stopped: %s. Plan again to continue.' % reason
            self.rev += 1

    def _tasks_of(self, robot, *states):
        about = self.plan_result.about
        return [name for name in self.graph.names(*states) if about[name]['robot'] == robot.id]

    def _release(self):
        """Authorize every robot whose drive tasks the graph has just released."""
        for robot in self.robots:
            arrive = 'r%d_arrive' % robot.id
            if robot.released or self.graph.state.get(arrive) != READY:
                continue
            robot.released = True
            ok, text = robot.nav.authorize()
            drive = [n for n in self._tasks_of(robot, READY) if self.plan_result.about[n]['kind'] in ('clear', 'arrive')]
            if ok:
                for name in drive:
                    self.graph.start(name)
                waited = [label(e.first) for e in self.plan_result.edges if e.then == robot.id]
                clear = ' (%s %s out of its way)' % (' and '.join(waited), 'are' if len(waited) > 1 else 'is') if waited else ''
                self._say('%s starts%s.' % (robot.label, clear))
            else:
                self._robot_failed(robot, 'could not start: %s' % text)

    def _say(self, text, level='info'):
        self.rev += 1
        try:
            self.say(text, level)
        except Exception:
            pass

    def _fail(self, robot, names, why):
        """These tasks of a robot have failed; what depends on them is blocked."""
        if self._stopping is not None:          # a STOP is being carried out: that is not a failure
            return
        robot.note = why
        blocked = []
        for name in names:
            if self.graph.state[name] in (READY, TASK_RUNNING):
                blocked += self.graph.fail(name, why)
        waiting = sorted({label(self.plan_result.about[n]['robot']) for n in blocked
                          if self.plan_result.about[n]['robot'] not in (None, robot.id)})
        self._say('%s %s%s' % (robot.label, why, ' %s waited for it and cannot start.' % ' and '.join(waiting)
                               if waiting else ''), 'warn')

    def _robot_failed(self, robot, why):
        """A robot has stopped for good. Its own tasks fail. The robots that were waiting for it are not
        simply blocked: if it is not standing in their way they may go, and if it is, a way around it is
        looked for when their turn comes (``_try_detours``)."""
        if self._stopping is not None:          # a STOP is being carried out: that is not a failure
            return
        robot.note, robot.failed, robot.retry_at = why, True, None
        own = self._let_the_others_go(robot)
        self._say('%s %s' % (robot.label, why), 'warn')
        for name in own:
            if self.graph.state[name] in (READY, TASK_RUNNING):
                self.graph.fail(name, why)

    def _let_the_others_go(self, robot):
        """This robot will not drive its route any further. The robots that waited for it to get clear may go
        if it is not standing in their way; otherwise a way around it is looked for when their turn comes.
        -> its own open tasks that are not about another robot"""
        position = robot.position()
        own = []
        for name in self._tasks_of(robot, READY, TASK_RUNNING):
            about = self.plan_result.about[name]
            if about['kind'] != 'clear':
                own.append(name)
                continue
            if self.graph.state[name] == READY:
                self.graph.start(name)
            other = self.plan_result.member(about['other'])
            if point_gap(position, other.path)[0] >= self.distance:
                self.graph.succeed(name, '%s stopped, but not in the way of %s' % (robot.label, label(other.id)))
                self._say('%s is not in the way of %s, which can go on.' % (robot.label, label(other.id)))
            else:
                self.detours[name] = (robot.id, other.id)
        return own

    def _ends_its_drive(self, robot, why):
        """A robot will not get to its goal. If it is still fit to drive and has found a survivor, it goes
        to that survivor instead (a survivor standing in a narrow passage blocks the way like any block, and
        finding one is what the team is out for). Otherwise it has failed."""
        finding = None
        if self._approach and self._stopping is None and not robot.nav.safety.estop:
            finding = self._next_survivor(robot)
        if finding is None:
            self._robot_failed(robot, why)
            return
        robot.note = robot.short = 'did not get to its goal: %s' % why
        robot.retry_at = None
        for name in self._let_the_others_go(robot):
            if self.plan_result.about[name]['kind'] == 'arrive' and self.graph.state[name] in (READY, TASK_RUNNING):
                if self.graph.state[name] == READY:
                    self.graph.start(name)
                self.graph.succeed(name, '%s; it goes to survivor S%d instead' % (robot.short, finding['id']))
        self._say('%s %s. It goes to survivor S%d, which it found, instead.'
                  % (robot.label, why.rstrip('.'), finding['id']), 'warn')

    def _discs_now(self, robot):
        """The places this robot keeps away from, as the team stands now.

        A robot that will not drive its route any more (it arrived, has no goal, stopped for good or cannot
        start) is an obstacle where it stands, or where it is going if it is on its way to a survivor; the
        place it started from is free. For a robot that still has to drive, the places the plan chose stay.
        A robot that has finished its own route (it is going to a survivor) also keeps away from where the
        others are waiting and from where they will park."""
        mine_done = self.graph.state.get('r%d_arrive' % robot.id) == SUCCEEDED
        planned = {tuple(disc[0]) for disc in robot.plan_discs}
        discs = []
        for other in self.robots:
            if other is robot:
                continue
            arrive = self.graph.state.get('r%d_arrive' % other.id)
            if other.failed or other.short or other.goal is None or arrive in (SUCCEEDED, BLOCKED, CANCELLED):
                place = other.position() if other.failed or other.spot is None else other.spot
                discs.append((tuple(place), self.keep))
                continue
            member = self.plan_result.member(other.id)
            if tuple(member.start) in planned:
                discs.append((tuple(member.start), self.keep))
            elif mine_done and not other.released:
                discs.append((other.position(), self.keep))
            if tuple(member.goal) in planned or mine_done:
                discs.append((tuple(member.goal), self.keep))
        return discs

    def _try_detours(self):
        """A robot stands for good on the route of a robot that waited for it. When it is that robot's turn
        (nothing else it waits for is still open), let it plan another way, around the one that stands."""
        for name in list(self.detours):
            if name not in self.detours:
                continue
            _stuck, other_id = self.detours[name]
            mine = [n for n, (_s, o) in self.detours.items() if o == other_id]
            arrive = 'r%d_arrive' % other_id
            if any(self.graph.state[n] != SUCCEEDED for n in self.graph.needs[arrive] if n not in mine):
                continue
            other = self.robot(other_id)
            stuck = [self.robot(self.detours[n][0]) for n in mine]
            names = ' and '.join(r.label for r in stuck)
            discs = self._discs_now(other)
            other.nav.set_map(with_discs(self.grid, discs))
            other.nav.set_goal(other.goal)
            route = [tuple(p) for p in other.nav.path] if other.nav.state == PLANNED else None
            trouble = self._too_close_to(other, route) if route else None
            for n in mine:
                del self.detours[n]
            if route is None or trouble is not None:
                why = ('there is no way around %s' % names if route is None
                       else 'the only way around %s comes too close to %s' % (names, trouble))
                other.note = 'cannot start: %s' % why
                for n in mine:
                    self.graph.fail(n, why)
                self._say('%s cannot start: %s.' % (other.label, why), 'warn')
                continue
            other.discs = discs
            path = tuple((float(x), float(y)) for x, y in route)
            plan = self.plan_result                         # the waiting robot's route in the plan is the new one
            self.plan_result = dataclasses.replace(plan, members=tuple(
                dataclasses.replace(m, path=path) if m.id == other_id else m for m in plan.members))
            for n in mine:
                self.graph.succeed(n, '%s takes another way around %s' % (other.label, names))
            self._say('%s takes another way, around %s.' % (other.label, names), 'warn')

    def _too_close_to(self, robot, route):
        """Would this (new) route of a robot bring it too close to another one? -> that robot's label, or None"""
        mine_done = self.graph.state.get('r%d_arrive' % robot.id) == SUCCEEDED     # (on its way to a survivor)
        for other in self.robots:
            if other is robot:
                continue
            arrive = self.graph.state.get('r%d_arrive' % other.id)
            # a robot that waits for this one stands still while this one drives to its goal; afterwards it drives
            waits_for_me = not mine_done and any(e.first == robot.id and e.then == other.id
                                                 for e in self.plan_result.edges)
            if other.nav.state == RUNNING and other.nav.motion == 'path':
                tracker = other.nav.tracker
                gap = path_gap(route, ahead(other.nav.path, other.position(), getattr(tracker, 'segment', None)))[0]
            elif arrive is None or arrive in (BLOCKED, CANCELLED) or other.released or other.failed or waits_for_me:
                gap = point_gap(other.position(), route)[0]         # it stands still (parked, done, stopped, or waiting)
            else:                                                   # it will drive later, independently of this one
                gap = path_gap(route, self.plan_result.member(other.id).path)[0]
            if gap < self.distance:
                return other.label
        return None

    # ------------------------------------------------------------------ the supervisor
    def _loop(self):
        while not self._stop_thread.wait(0.1):
            try:
                self.step()
            except Exception as exc:          # a bug must stop the robots, not leave them driving unwatched
                for robot in list(self.robots):
                    if robot.nav is not None:
                        robot.nav.emergency_stop('Team supervisor error')
                with self._lock:
                    if self.graph is not None and self.phase == RUNNING_PHASE:
                        self.graph.cancel('internal error')
                        self.phase = FAILED_PHASE
                    self.message = 'Stopped: internal error in the team supervisor (%r).' % exc
                    self._say(self.message, 'error')

    def step(self):
        """One look at the whole team: keep the robots apart, move the task graph on."""
        with self._lock:
            robots = list(self.robots)
            if not robots or any(robot.nav is None for robot in robots):
                return
            if self._stopping is not None:
                self._finish_stop()
                return
            truth = {robot.id: robot.backend.truth_pose() for robot in robots}
            for robot in robots:        # the simulated robots cannot drive through each other
                robot.backend.others = [(truth[o.id][0], truth[o.id][1], self.body) for o in robots if o is not robot]
            self._keep_apart(robots)
            self._collect_findings(robots)
            if self.phase == PLANNED_PHASE and self.plan_result is not None:
                self._plan_still_holds()
            if self.phase != RUNNING_PHASE or self.graph is None:
                return
            for robot in robots:
                self._drive_tasks(robot)
                self._scan_task(robot)
                self._rescue_task(robot)
            if self._stopping is None:
                self._try_detours()
            if self.graph.state['team_report'] == READY:
                self.graph.start('team_report')
                self.report = self._write_report()
                self.graph.succeed('team_report')
            self._release()
            if self._stopping is not None:      # STOP came while this step was looking at the robots
                self._finish_stop()
                return
            if self.graph.finished():
                status = self.graph.status()
                self.phase = {SUCCEEDED: DONE, CANCELLED: STOPPED_PHASE}.get(status, FAILED_PHASE)
                seconds = self.clock() - self._started
                if status == SUCCEEDED:
                    short = [robot.label for robot in robots if robot.short]
                    there = [label(m.id) for m in self.plan_result.members if not self.robot(m.id).short]
                    reached = ('every robot reached its goal.' if not short else '%s%s stopped at a survivor on %s way.' % (
                        '%s reached %s; ' % (' and '.join(there), 'their goals' if len(there) > 1 else 'its goal')
                        if there else '', ' and '.join(short), 'their' if len(short) > 1 else 'its'))
                    self.message = 'Team mission complete in %.0f s: %s%s' % (seconds, reached, self._survivor_text())
                    self._say(self.message, 'success')
                elif status == FAILED:
                    self.message = self._end_text()
                    self._say(self.message, 'warn')

    def _plan_still_holds(self):
        """Between planning and the operator's go-ahead the robots' navigators keep their routes up to date
        (a camera that refines where a block is shifts a route a little). Keep the plan in step with the routes
        the robots really hold. -> True while there is a plan.

        The same order of the team: the plan is quietly brought up to date. Another order, or no order any
        more: that is a different plan, the operator is told, and an authorization that was being asked for
        no longer fits (``TeamPlan.signature``)."""
        plan = self.plan_result
        paths = {}
        for member in plan.members:
            robot = self.robot(member.id)
            state, _kind, message, route, _segment = self._look(robot.nav)
            if state != PLANNED or not route:
                self._forget_plan()
                self.message = '%s has no planned route any more (%s): plan again.' % (robot.label, message)
                self._say(self.message, 'warn')
                return False
            paths[member.id] = tuple((float(x), float(y)) for x, y in route)
        if all(paths[m.id] == m.path for m in plan.members):
            return True
        try:
            fresh = build_plan([Member(m.id, m.start, m.goal, paths[m.id]) for m in plan.members], self.distance,
                               scan=self._scan, approach=self._approach)
        except TeamError as exc:
            self._forget_plan()
            self.message = 'A route changed and the team can no longer be ordered: %s' % exc
            self._say(self.message, 'warn')
            return False
        self.plan_result = fresh
        self.rev += 1
        if fresh.signature != plan.signature:
            self.message = 'A route changed, and with it the order of the team. ' + self._plan_text(fresh)
            self._say(self.message, 'warn')
        return True

    def _end_text(self):
        """A mission that did not go all the way, in words: who arrived, who did not and why."""
        arrived, stopped, waiting = [], [], []
        for member in self.plan_result.members:
            robot = self.robot(member.id)
            state = self.graph.state['r%d_arrive' % member.id]
            if state == SUCCEEDED and robot.short:
                stopped.append('%s %s' % (robot.label, robot.note or robot.short))
            elif state == SUCCEEDED:
                arrived.append(robot.label)
                later = [self.graph.state.get('r%d_%s' % (member.id, kind)) for kind in ('scan', 'rescue')]
                if FAILED in later:                         # it got there, and then stopped for good
                    stopped.append('%s then %s' % (robot.label, robot.note or 'stopped'))
            elif state == FAILED:
                stopped.append('%s %s' % (robot.label, robot.note or 'stopped'))
            else:
                waiting.append('%s (%s)' % (robot.label, robot.note or 'it waited for a robot that stopped'))
        parts = []
        if arrived:
            parts.append('%s reached %s.' % (' and '.join(arrived), 'their goals' if len(arrived) > 1 else 'its goal'))
        parts += ['%s.' % text.rstrip('.') for text in stopped]
        if waiting:
            parts.append('Did not start: %s.' % ', '.join(waiting))
        return 'Team mission ended early. ' + ' '.join(parts) + ' Plan again to go on from here.'

    def _keep_apart(self, robots):
        """The last check: two robots closer than they should ever be. Whoever is driving stops. Also here:
        a robot that touched something in the simulator (reported), and one that makes no headway (stopped)."""
        now = self.clock()
        for k, first in enumerate(robots):
            for second in robots[:k]:
                gap = math.dist(first.position(), second.position())
                if gap >= self.too_close:
                    continue
                for robot, other in ((first, second), (second, first)):
                    if robot.nav.state == RUNNING and robot.nav.motion == 'path':
                        robot.nav.emergency_stop('Too close to %s (%.2f m)' % (other.label, gap))
                        self._say('%s stopped: too close to %s (%.2f m).' % (robot.label, other.label, gap), 'warn')
        for robot in robots:
            if robot.backend.collided:              # the simulator did not let it move into a wall, block or robot
                robot.backend.collided = False
                if now - robot.touched_at > 3.0:
                    self._say('%s touched something in the simulator.' % robot.label, 'warn')
                robot.touched_at = now
            if robot.nav.state != RUNNING or robot.nav.motion != 'path':
                robot.progress = None
                continue
            x, y, yaw = robot.backend.truth_pose()
            mark = robot.progress
            if mark is None or math.hypot(x - mark[1], y - mark[2]) > 0.02 or abs(yaw - mark[3]) > 0.1:
                robot.progress = (now, x, y, yaw)
            elif now - mark[0] > NO_PROGRESS_S:
                robot.progress = None
                robot.nav.halt_motion('Stopped: %s made no headway for %.0f s (stuck).' % (robot.label, NO_PROGRESS_S),
                                      kind='tracking')

    @staticmethod
    def _look(nav):
        """(state, why it stopped, message, route, segment) of a navigator, read in one piece. The navigator
        stops and replans in one go under its lock; looking without the lock could catch it in between and
        take "stopped for a moment, new route coming" for "stopped for good"."""
        with nav._lock:
            tracker = nav.tracker
            return (nav.state, nav.stop_kind, nav.message, [tuple(p) for p in nav.path],
                    getattr(tracker, 'segment', None))

    def _drive_tasks(self, robot):
        graph, nav = self.graph, robot.nav
        arrive = 'r%d_arrive' % robot.id
        if graph.state.get(arrive) != TASK_RUNNING:        # (a robot without a goal has no tasks)
            return
        state, _kind, message, route, segment = self._look(nav)
        if state == RUNNING or state == ARRIVED:
            position = robot.position()
            rest = [position] if state == ARRIVED else ahead(route, position, segment)
            for name in self._tasks_of(robot, TASK_RUNNING):
                about = self.plan_result.about[name]
                if about['kind'] != 'clear':
                    continue
                other = self.plan_result.member(about['other'])
                if path_gap(rest, other.path)[0] >= self.distance:
                    graph.succeed(name)
                    self._say("%s is clear of %s's route." % (robot.label, label(other.id)))
                elif state == ARRIVED:
                    self._fail(robot, [name], "is parked on %s's route" % label(other.id))
            if state == ARRIVED:
                robot.replanned_at = None
                graph.succeed(arrive)
                self._say('%s reached its goal.' % robot.label, 'success')
        elif state == PLANNED:
            self._continue_after_replan(robot)
        elif state in (STOPPED, IDLE):
            self._stopped(robot, _kind, message)

    def _stopped(self, robot, kind, message):
        """A driving robot has stopped. For something it can get over, it plans again and drives on (a few
        times, after a moment's rest). An operator STOP, a lost link or a lost position end its mission."""
        nav, now = robot.nav, self.clock()
        if kind not in CAN_RETRY or nav.safety.estop or robot.retries >= RETRIES:
            again = ' (after %d new tries)' % robot.retries if robot.retries else ''
            self._ends_its_drive(robot, 'stopped%s: %s' % (again, message))
            return
        if robot.retry_at is None:
            robot.retry_at = now + 1.5
            return
        if now < robot.retry_at or not nav.robot_still():
            return
        robot.retries += 1
        robot.retry_at = None
        if any(other.failed or other.short or other.spot is not None for other in self.robots if other is not robot):
            robot.discs = self._discs_now(robot)            # around the robots that are not where the plan has them
            nav.set_map(with_discs(self.grid, robot.discs))
        nav.set_goal(robot.goal)                            # a fresh route from where it stands now
        if nav.state != PLANNED:
            self._ends_its_drive(robot, 'found no way on (%s)' % nav.message.rstrip('.'))
            return
        self._say('%s stopped (%s) and plans again: try %d of %d.' % (robot.label, message.rstrip('.'), robot.retries,
                                                                    RETRIES), 'warn')
        robot.replanned_at = now - 1.0                      # (the route is checked and driven in the next step)

    def _continue_after_replan(self, robot):
        """The navigator met an obstacle, planned a new route and waits. Continue if the new route is no
        trouble for the others; the operator authorized the mission, not one particular route."""
        nav, now = robot.nav, self.clock()
        if robot.replanned_at is None:
            robot.replanned_at = now
            return
        if now - robot.replanned_at < 1.0 or not nav.robot_still():
            return
        if math.dist(robot.position(), nav.path[0]) > 0.03:        # it rolled on a little: plan from where it is
            nav.set_goal(robot.goal)
            if nav.state != PLANNED:
                return                                              # (the next step deals with a robot without a route)
        here = robot.position()
        if robot.replan_from is None or math.dist(here, robot.replan_from) > 0.10:
            robot.replan_from, robot.replans = here, 0      # it got somewhere since the last new route
        if robot.replans >= REPLANS:        # the same thing stops it every time (its safety stop, a gap too narrow)
            robot.replanned_at = None
            self._ends_its_drive(robot, 'cannot get past what is in front of it (%d new routes tried)' % robot.replans)
            return
        trouble = self._too_close_to(robot, [tuple(p) for p in nav.path])
        if trouble is not None:
            nav.emergency_stop('New route too close to %s' % trouble)
            self._robot_failed(robot, 'stopped: its new route would come too close to %s' % trouble)
            return
        ok, text = nav.authorize()
        robot.replanned_at = None
        if ok:
            robot.replans += 1
            self._say('%s continues on a new route.' % robot.label, 'warn')
        else:
            self._robot_failed(robot, 'could not continue: %s' % text)

    def _scan_task(self, robot):
        name = 'r%d_scan' % robot.id
        graph, nav = self.graph, robot.nav
        state = graph.state.get(name)
        if state == READY and robot.short:
            graph.start(name)
            graph.succeed(name, 'not at its goal: it goes to the survivor it found')
        elif state == READY:
            heading = robot.pose()[2]
            turns = [heading + math.radians(a) for a in (90, 180, 270, 360)]
            before = nav.halt_seq
            ok, text = nav.start_spin(turns, dwell_s=0.4, max_rate=MISSION.scan_turn_rate,
                                      note='%s looks around at its goal.' % robot.label)
            if ok:
                robot.spin_from = before
                graph.start(name)
            else:
                graph.start(name)
                self._fail(robot, [name], 'could not look around: %s' % text)
        elif state == TASK_RUNNING:
            with nav._lock:
                nav_state, halts, message = nav.state, nav.halt_seq, nav.message
            if nav_state == ARRIVED and halts > robot.spin_from:
                graph.succeed(name)
                self._say('%s has looked around.' % robot.label)
            elif nav_state in (STOPPED, IDLE, PLANNED):
                self._fail(robot, [name], 'stopped while looking around: %s' % message)

    # ------------------------------------------------------------------ driving up to survivors
    def _finding(self, finding_id):
        return next((f for f in self.findings if f['id'] == finding_id), None)

    def _next_survivor(self, robot):
        """The nearest survivor this robot has seen itself that nobody has secured or is going to."""
        position = robot.position()
        mine = [f for f in self.findings
                if f.get('status', 'found') == 'found' and f.get('claimed') in (None, robot.id)
                and robot.id in f['by'] and f['id'] not in robot.gave_up]
        return min(mine, key=lambda f: math.dist(position, (f['x'], f['y'])), default=None)

    @staticmethod
    def _survivor_place(robot, finding):
        """Where the survivor is by this robot's own camera (its latest view), else as it was first reported."""
        seen = [(math.hypot(float(o['x']) - finding['x'], float(o['y']) - finding['y']), float(o['x']), float(o['y']))
                for o in list(robot.nav.vision_obstacles) if str(o.get('cls', '')).startswith('green')]
        near = min(seen, default=None)
        return (near[1], near[2]) if near is not None and near[0] < 0.2 else (finding['x'], finding['y'])

    def _wall_between(self, a, b):
        """Is there a wall of the arena on the straight line from a to b?"""
        world = self.world
        steps = max(2, int(math.dist(a, b) / (world.resolution / 2.0)))
        for k in range(1, steps):
            cell = world.world_to_cell((a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps))
            if world.contains(cell) and world.cells[cell]:
                return True
        return False

    def _rescue_task(self, robot):
        """After its other tasks a robot drives up to the survivors it has seen itself: to a free spot in front
        of the survivor, where it turns to face it and stands for a moment ("secured"), as the single robot's
        search mission does. It only sets off when its way is clear of what the other robots still have to
        drive; until then it waits where it is."""
        name = 'r%d_rescue' % robot.id
        graph = self.graph
        state = graph.state.get(name)
        if state == READY:
            graph.start(name)
            robot.rescue = dict(stage='pick', done=[], missed=[])
            state = TASK_RUNNING
        if state != TASK_RUNNING or robot.rescue is None:
            return
        job, nav, now = robot.rescue, robot.nav, self.clock()
        if job['stage'] == 'pick':
            finding = self._next_survivor(robot)
            if finding is None:
                parts = []
                if job['done']:
                    parts.append('secured %s' % ' and '.join('S%d' % n for n in job['done']))
                if job['missed']:
                    parts.append('did not get to %s' % ' and '.join('S%d' % n for n in job['missed']))
                note = '; '.join(parts) or 'it found no survivor to go to'
                job['stage'] = 'over'
                if robot.short and not job['done']:         # neither at its goal nor at a survivor
                    self._rescue_failed(robot, name, robot.short)
                    return
                graph.succeed(name, note)
                self.rev += 1
                return
            finding['claimed'] = robot.id
            job.update(stage='plan', finding=finding['id'], since=now, next=now, tries=0, waiting_for=None, replans=0)
            self.rev += 1
        if job['stage'] == 'plan':
            if now >= job['next'] and nav.robot_still():
                self._plan_approach(robot, job, name)
        elif job['stage'] == 'drive':
            self._watch_approach(robot, job, name)
        elif job['stage'] == 'face':
            self._watch_securing(robot, job, name)

    def _rescue_failed(self, robot, name, why):
        if self._stopping is not None:          # a STOP is being carried out: that is not a failure
            return
        robot.failed = True                     # it stands wherever it stopped: an obstacle there for the others
        self._fail(robot, [name], why)

    def _give_up_survivor(self, robot, job, why):
        number = job['finding']
        finding = self._finding(number)
        if finding is not None and finding.get('claimed') == robot.id:
            finding['claimed'] = None
        robot.gave_up.add(number)
        job['missed'].append(number)
        job['stage'] = 'pick'
        self._say('%s does not go to survivor S%d: %s.' % (robot.label, number, why), 'warn')

    def _plan_approach(self, robot, job, name):
        nav, now = robot.nav, self.clock()
        number = job['finding']
        finding = self._finding(number)
        if finding is None or finding.get('status') == 'secured':       # the list was emptied meanwhile
            job['stage'] = 'pick'
            return
        target = self._survivor_place(robot, finding)
        position = robot.position()
        ring = MISSION.approach_gap_m + self.half_length + MISSION.object_size_m / 2
        if math.dist(position, target) <= ring + 0.08 and not self._wall_between(position, target):
            self._face_survivor(robot, job, name, target)               # close enough already: just face it
            return
        discs = self._discs_now(robot)
        grid = with_discs(self.grid, discs)
        checker = CollisionChecker(grid)
        spots = []
        for k in range(16):
            angle = 2 * math.pi * k / 16
            spot = (target[0] + ring * math.cos(angle), target[1] + ring * math.sin(angle))
            if not grid.contains(grid.world_to_cell(spot)) or not checker.is_clear(spot, spot, self.radius):
                continue
            if self._wall_between(spot, target):                        # it has to see the survivor from there
                continue
            roomy = checker.is_clear(spot, spot, self.radius + MISSION.spot_clearance_m)
            spots.append((not roomy, math.dist(spot, position), spot))  # roomy spots first, then the nearest
        nav.set_map(grid)
        robot.discs = discs
        waits_for = None
        for _crowded, _far, spot in sorted(spots)[:6]:
            goal = tuple(grid.cell_to_world(grid.world_to_cell(spot)))
            nav.set_goal(goal)
            if nav.state != PLANNED or not nav.path:
                continue
            trouble = self._too_close_to(robot, [tuple(p) for p in nav.path])
            if trouble is not None:
                waits_for = waits_for or trouble
                continue
            ok, text = nav.authorize()
            if not ok:
                self._rescue_failed(robot, name, 'could not set off to survivor S%d: %s' % (number, text))
                return
            job.update(stage='drive', spot=goal, target=target, waiting_for=None, replanned_at=None)
            robot.spot, robot.progress = goal, None
            self._say('%s drives to survivor S%d.' % (robot.label, number))
            return
        if waits_for is None:
            self._give_up_survivor(robot, job, 'there is no safe way to it')
        elif now - job['since'] > RESCUE_WAIT_S:
            self._give_up_survivor(robot, job, 'the way to it did not become free (%s)' % waits_for)
        else:
            if job.get('waiting_for') != waits_for:
                self._say('%s waits for %s before it drives to survivor S%d.' % (robot.label, waits_for, number))
            job.update(waiting_for=waits_for, next=now + 2.0)

    def _watch_approach(self, robot, job, name):
        nav, now = robot.nav, self.clock()
        number = job['finding']
        state, kind, message, route, _segment = self._look(nav)
        if state == RUNNING:
            job['replanned_at'] = None
            return
        if state == ARRIVED:
            self._face_survivor(robot, job, name, job['target'])
            return
        if state == PLANNED:                    # it met an obstacle, has a new route and waits for the go-ahead
            if job.get('replanned_at') is None:
                job['replanned_at'] = now
                return
            if now - job['replanned_at'] < 1.0 or not nav.robot_still():
                return
            job['replanned_at'] = None
            if job.get('replans', 0) >= REPLANS:                # the same thing stops it every time
                self._give_up_survivor(robot, job, 'it cannot get past what is in front of it')
                robot.spot = robot.position()
                return
            trouble = self._too_close_to(robot, route)
            if trouble is None:
                ok, message = nav.authorize()
                if ok:
                    job['replans'] = job.get('replans', 0) + 1
                    self._say('%s continues to survivor S%d on a new route.' % (robot.label, number), 'warn')
                    return
            else:
                message = 'its new route would come too close to %s' % trouble
        elif nav.safety.estop or kind not in CAN_RETRY:
            self._rescue_failed(robot, name, 'stopped on the way to survivor S%d: %s' % (number, message))
            return
        job['tries'] += 1                       # something it can get over: plan again from where it stands
        robot.spot = robot.position()
        if job['tries'] > RETRIES:
            self._give_up_survivor(robot, job, 'it stopped %d times on the way (%s)' % (job['tries'], message.rstrip('.')))
        else:
            job.update(stage='plan', next=now + 1.5, since=now)

    def _face_survivor(self, robot, job, name, target):
        nav = robot.nav
        number = job['finding']
        x, y = robot.position()
        before = nav.halt_seq
        ok, text = nav.start_spin([math.atan2(target[1] - y, target[0] - x)], dwell_s=MISSION.secure_hold_s,
                                  note='%s secures survivor S%d.' % (robot.label, number))
        if not ok:
            self._rescue_failed(robot, name, 'could not turn to survivor S%d: %s' % (number, text))
            return
        robot.spot = (x, y)
        job.update(stage='face', spin_from=before, target=target)
        self._say('%s is at survivor S%d.' % (robot.label, number))

    def _watch_securing(self, robot, job, name):
        nav = robot.nav
        number = job['finding']
        with nav._lock:
            state, halts, message = nav.state, nav.halt_seq, nav.message
        if state == ARRIVED and halts > job['spin_from']:
            finding = self._finding(number)
            if finding is not None:
                finding.update(status='secured', secured_by=robot.id, claimed=robot.id)
            job['done'].append(number)
            job['stage'] = 'pick'
            self._say('%s secured survivor S%d.' % (robot.label, number), 'success')
        elif state in (STOPPED, IDLE, PLANNED):
            self._rescue_failed(robot, name, 'stopped while securing survivor S%d: %s' % (number, message))

    def _survivor_text(self):
        if not self._approach or not self.findings:
            return ''
        secured = sum(1 for f in self.findings if f.get('status') == 'secured')
        return ' %d of %d survivor%s secured.' % (secured, len(self.findings), '' if len(self.findings) == 1 else 's')

    def _collect_findings(self, robots):
        """Survivors (green blocks) seen by any robot's camera go into one list for the whole team."""
        for robot in robots:
            for seen in list(robot.nav.vision_obstacles):
                if not str(seen.get('cls', '')).startswith('green'):
                    continue
                x, y = float(seen['x']), float(seen['y'])
                known = next((f for f in self.findings if math.hypot(f['x'] - x, f['y'] - y) < 0.15), None)
                if known is None:
                    self.findings.append(dict(id=len(self.findings) + 1, x=round(x, 3), y=round(y, 3),
                                              by=[robot.id], t=round(self.clock() - (self._started or self.clock()), 1),
                                              status='found', claimed=None, secured_by=None))
                    self._say('%s sees a survivor at (%.2f, %.2f): S%d.' % (robot.label, x, y, len(self.findings)), 'success')
                elif robot.id not in known['by']:
                    known['by'].append(robot.id)
                    self.rev += 1

    def _write_report(self):
        return dict(seconds=round(self.clock() - self._started, 1), robots=len(self.plan_result.members),
                    survivors=[dict(id=f['id'], x=f['x'], y=f['y'], by=[label(i) for i in f['by']],
                                    status=f.get('status', 'found'),
                                    secured_by=label(f['secured_by']) if f.get('secured_by') else None)
                               for f in self.findings],
                    waited=[dict(robot=label(e.then), on=label(e.first), why=e.why) for e in self.plan_result.edges])

    # ------------------------------------------------------------------ for displays
    def poses(self):
        """[(x, y, yaw, navigator state)] for R1, R2, ...: cheap, for the 20 Hz picture."""
        out = []
        for robot in list(self.robots):
            if robot.nav is None:
                continue
            x, y, yaw = robot.pose()
            out.append((round(x, 4), round(y, 4), round(yaw, 4), robot.nav.state))
        return out

    def frame(self, robot_id, after=None, timeout=1.5):
        robot = self.robot(robot_id)
        return robot.vision.wait_frame(after, timeout) if robot.vision is not None else None

    def snapshot(self, wait=None):
        """Everything a display needs. ``wait`` (seconds): do not wait longer than this for the team's lock
        (planning holds it for a moment); the previous picture is returned instead, so a page never freezes."""
        if not self._lock.acquire(timeout=-1 if wait is None else wait):
            return self._last_snapshot or dict(phase=self.phase, message=self.message, rev=self.rev, robots=[],
                                               tasks=[], waves=[], edges=[], planned=[], findings=[], blocks=[],
                                               report=None, distance=round(self.distance, 3),
                                               apart=round(self.apart, 3), camera=bool(self.camera))
        try:
            self._last_snapshot = self._snapshot()
            return self._last_snapshot
        finally:
            self._lock.release()

    def _snapshot(self):
        plan, graph = self.plan_result, self.graph
        robots = []
        for robot in self.robots:
            nav = robot.nav
            x, y, yaw = robot.pose()
            task = ''
            if graph is not None and 'r%d_arrive' % robot.id in graph.state:
                waiting = [n for n in graph.needs['r%d_arrive' % robot.id] if graph.state[n] != SUCCEEDED]
                arrive = graph.state['r%d_arrive' % robot.id]
                if arrive in ('PENDING', READY) and waiting:
                    task = 'waits for ' + ' and '.join(dict.fromkeys(label(plan.about[n]['robot']) for n in waiting))
                elif arrive == BLOCKED:
                    task = 'cannot start'
            elif plan is not None and robot.goal is None:
                task = 'stays here'
            job = robot.rescue
            if graph is not None and job and graph.state.get('r%d_rescue' % robot.id) == TASK_RUNNING:
                words = dict(plan='waits to go to S%d' if job.get('waiting_for') else 'plans the way to S%d',
                             drive='drives to S%d', face='secures S%d').get(job['stage'])
                if words:
                    task = words % job['finding']
            elif graph is not None and job and job.get('done') and robot.spot is not None and not robot.failed:
                task = 'at survivor S%d' % job['done'][-1]          # (where it stands now, not at its goal)
            robots.append(dict(
                id=robot.id, label=robot.label, colour=robot.colour, pose=[round(x, 3), round(y, 3), round(yaw, 3)],
                goal=list(robot.goal) if robot.goal else None, state=nav.state if nav else IDLE,
                motion=nav.motion if nav else None, message=nav.message if nav else '', note=robot.note,
                task=task, camera=robot.vision is not None, retries=robot.retries,
                collided=self.clock() - robot.touched_at < 3.0,
                path=[[round(px, 3), round(py, 3)] for px, py in (nav.path if nav else [])],
                pins=[dict(x=round(float(p['x']), 3), y=round(float(p['y']), 3), cls=str(p.get('cls', '')))
                      for p in (nav.vision_obstacles if nav else [])]))
        tasks = []
        if plan is not None:
            for name in plan.order:
                about = plan.about[name]
                tasks.append(dict(name=name, text=about['text'], kind=about['kind'], robot=about['robot'],
                                  other=about['other'], needs=list(next(t.needs for t in plan.tasks if t.name == name)),
                                  state=graph.state[name] if graph else 'PENDING',
                                  note=graph.note[name] if graph else ''))
        return dict(
            phase=self.phase, message=self.message, rev=self.rev, robots=robots, tasks=tasks,
            waves=[list(wave) for wave in plan.waves] if plan else [],
            edges=[dict(first=e.first, then=e.then, forced=e.forced, gap=e.gap, at=list(e.at), why=e.why)
                   for e in plan.edges] if plan else [],
            planned=[dict(id=m.id, path=[[round(x, 3), round(y, 3)] for x, y in m.path]) for m in plan.members]
            if plan else [],
            findings=[dict(f, by=[label(i) for i in f['by']], status=f.get('status', 'found'),
                           claimed=label(f['claimed']) if f.get('claimed') else None,
                           secured_by=label(f['secured_by']) if f.get('secured_by') else None)
                      for f in self.findings],
            blocks=[[round(x, 3), round(y, 3), cls] for _, (x, y, cls) in self.blocks.items()] if self.blocks else [],
            report=self.report, distance=round(self.distance, 3), apart=round(self.apart, 3),
            camera=bool(self.camera))
