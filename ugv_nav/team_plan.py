"""Team missions: several robots in one arena, and who may drive when.

Every robot has a start, a goal and a planned route. Two robots are in each other's way when their routes
come closer than a safe distance. For every such pair one of the two has to go first: that is a dependency
("R2 may start when R1 has driven clear of R2's route"). All dependencies of a team form a directed graph
without circles, a DAG, and Kahn's algorithm turns it into an order:

* robots that are not in anybody's way start at once and drive at the same time,
* a robot that has to wait starts when the tasks it depends on have succeeded,
* when a robot fails, the robots that were waiting for it are blocked instead of driving into it,
* a circle ("R1 waits for R2, R2 waits for R1") means the setup cannot be driven, and is refused.

Kahn's algorithm and the task type are the ones of the team's scheduler, ``DAG_scheduler/dag_scheduler/
engine.py`` (``topological_order``, ``Task``). That scheduler itself runs one task at a time in separate
processes, which suits a pipeline of programs but not several robots moving at once; so the live part is
``LiveGraph`` below, which keeps the scheduler's rules (only a succeeded task releases the tasks that need
it; a failed task blocks them) and lets every released task run.

Nothing in this file moves a robot or knows about the simulator: it is geometry and bookkeeping.
"""
from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path

ENGINE_FILE = Path(__file__).resolve().parents[1] / 'DAG_scheduler' / 'dag_scheduler' / 'engine.py'
_engine = None

PENDING, READY, RUNNING, SUCCEEDED, FAILED, BLOCKED, CANCELLED = (
    'PENDING', 'READY', 'RUNNING', 'SUCCEEDED', 'FAILED', 'BLOCKED', 'CANCELLED')


class TeamError(ValueError):
    """The team setup cannot be planned or driven; the text says why, for the operator."""


def dag_engine():
    """The team's DAG scheduler module (Kahn's algorithm lives there). Loaded from its file, once."""
    global _engine
    if _engine is None:
        if not ENGINE_FILE.is_file():
            raise TeamError('The DAG scheduler is missing (%s): a team mission is ordered by its Kahn algorithm '
                            'and cannot be planned without it.' % ENGINE_FILE)
        spec = importlib.util.spec_from_file_location('team_dag_engine', ENGINE_FILE)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module             # dataclasses look their module up while it is being loaded
        try:
            spec.loader.exec_module(module)
        except Exception:
            del sys.modules[spec.name]
            raise
        _engine = module
    return _engine


def label(robot_id):
    return 'R%d' % robot_id


# ---------------------------------------------------------------------------- geometry
def _point_segment(p, a, b):
    """(distance, closest point) from point p to the segment a-b."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(p[0] - cx, p[1] - cy), (cx, cy)


def _crossing(a, b, c, d):
    """Do the segments a-b and c-d properly cross?"""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    s1, s2, s3, s4 = side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b)
    return (s1 > 0) != (s2 > 0) and (s3 > 0) != (s4 > 0) and 0.0 not in (s1, s2, s3, s4)


def _segment_segment(a, b, c, d):
    """(distance, a point where they are closest) between the segments a-b and c-d."""
    if _crossing(a, b, c, d):
        # where they cross: solve a + t (b - a) = c + u (d - c)
        r = (b[0] - a[0], b[1] - a[1])
        s = (d[0] - c[0], d[1] - c[1])
        t = ((c[0] - a[0]) * s[1] - (c[1] - a[1]) * s[0]) / (r[0] * s[1] - r[1] * s[0])
        return 0.0, (a[0] + t * r[0], a[1] + t * r[1])
    best = None
    for p, q, r in ((a, c, d), (b, c, d), (c, a, b), (d, a, b)):
        dist, closest = _point_segment(p, q, r)
        if best is None or dist < best[0]:
            best = (dist, ((p[0] + closest[0]) / 2.0, (p[1] + closest[1]) / 2.0))
    return best


def _segments(path):
    path = list(path)
    return list(zip(path, path[1:])) if len(path) > 1 else [(path[0], path[0])]


def point_gap(point, path):
    """(smallest distance from the point to the route, the route point where that is)."""
    return min((_point_segment(point, a, b) for a, b in _segments(path)), key=lambda item: item[0])


def path_gap(first, second):
    """(smallest distance between two routes, a point between them where that is)."""
    return min((_segment_segment(a, b, c, d) for a, b in _segments(first) for c, d in _segments(second)),
               key=lambda item: item[0])


def ahead(path, position, segment=None):
    """The part of a route that is still in front of a robot standing at ``position``.

    ``segment`` is the index of the route segment the robot is on, when its path follower knows it; without
    it the closest segment is taken (the first of equally close ones, so a route that passes the same place
    twice is not cut short).
    """
    path = [tuple(p) for p in path]
    if len(path) < 2:
        return [tuple(position)]
    if segment is None:
        segment = min(range(len(path) - 1), key=lambda k: (round(_point_segment(position, path[k], path[k + 1])[0], 4), k))
    segment = max(0, min(int(segment), len(path) - 2))
    return [tuple(position)] + path[segment + 1:]


# ---------------------------------------------------------------------------- who goes first
@dataclass(frozen=True)
class Member:
    """One robot's part of a team plan: where it stands, where it goes, and its planned route."""
    id: int
    start: tuple
    goal: tuple
    path: tuple


@dataclass(frozen=True)
class Edge:
    """``first`` has to drive clear of ``then``'s route before ``then`` may start."""
    first: int
    then: int
    forced: bool            # False: either order would have worked, the order was chosen
    gap: float              # how close the two routes come (m)
    at: tuple               # ... and where (x, y)
    why: str


def _may_go_first(a, b, distance):
    """Can ``a`` drive first while ``b`` waits at its start? -> (possible, why not).

    Not when ``b`` is standing on ``a``'s route, and not when ``a`` would end up parked on ``b``'s route.
    """
    if point_gap(b.start, a.path)[0] < distance:
        return False, "%s's route passes the place where %s is standing" % (label(a.id), label(b.id))
    if point_gap(a.goal, b.path)[0] < distance:
        return False, "%s would be parked at its goal on %s's route" % (label(a.id), label(b.id))
    return True, ''


def precedence(members, distance):
    """Which robot has to wait for which: a list of ``Edge``. Raises ``TeamError`` when it cannot be done.

    Routes that stay at least ``distance`` apart need no order. For a pair that comes closer, it is checked
    which of the two could go first. One possibility: that order is forced. Both: the order is chosen so that
    it agrees with all forced orders (Kahn's algorithm gives that order), then by the lower number. None, or
    forced orders that run in a circle: the robots block each other.
    """
    members = sorted(members, key=lambda m: m.id)
    engine = dag_engine()
    pairs, needs = [], {m.id: [] for m in members}
    for a, b in combinations(members, 2):
        gap, at = path_gap(a.path, b.path)
        if gap >= distance:
            continue
        a_first, a_problem = _may_go_first(a, b, distance)
        b_first, b_problem = _may_go_first(b, a, distance)
        if not a_first and not b_first:
            raise TeamError('%s and %s block each other: %s, and %s. Move a start or a goal.'
                            % (label(a.id), label(b.id), a_problem, b_problem))
        pairs.append((a, b, gap, at, a_first, b_first, a_problem or b_problem))
        if a_first != b_first:
            first, then = (a, b) if a_first else (b, a)
            needs[then.id].append(first.id)
    try:        # Kahn's algorithm on the forced orders: an order of the robots that respects all of them
        order = engine.topological_order([engine.Task('r%d' % m.id, tuple('r%d' % n for n in needs[m.id]))
                                          for m in members])
    except engine.ConfigurationError:
        stuck = ', '.join(label(m.id) for m in members if needs[m.id])
        raise TeamError('These robots wait for each other in a circle and none could start: %s. '
                        'Move a start or a goal.' % stuck) from None
    rank = {int(name[1:]): position for position, name in enumerate(order)}
    edges = []
    for a, b, gap, at, a_first, b_first, problem in pairs:
        if a_first and b_first:
            first, then = sorted((a, b), key=lambda m: (rank[m.id], m.id))
            why = 'their routes come within %.2f m of each other; %s goes first' % (gap, label(first.id))
        else:
            first, then = (a, b) if a_first else (b, a)
            why = '%s, so %s has to go first' % (problem, label(first.id))
        edges.append(Edge(first.id, then.id, not (a_first and b_first), round(gap, 3),
                          (round(at[0], 3), round(at[1], 3)), why))
    return edges


# ---------------------------------------------------------------------------- the task graph
@dataclass(frozen=True)
class TeamPlan:
    members: tuple
    edges: tuple
    tasks: tuple            # the scheduler's Task objects (name + the names it needs)
    about: dict             # task name -> dict(robot, kind, other, text)
    order: tuple            # Kahn's order of the task names
    waves: tuple            # the same, grouped: wave 1 can start at once, wave 2 after wave 1, ...
    distance: float

    def member(self, robot_id):
        return next(m for m in self.members if m.id == robot_id)

    @property
    def signature(self):
        """What the operator's authorization is given for: the goals, who waits for whom, and the tasks.
        Not the exact routes: a robot's navigator shifts its route a little whenever its camera refines where
        a block is, and that must not undo an authorization as long as the order of the team stays the same."""
        return (tuple((m.id, tuple(round(v, 3) for v in m.goal)) for m in self.members),
                tuple(sorted((e.first, e.then) for e in self.edges)), tuple(task.name for task in self.tasks))


def build_plan(members, distance, scan=True, approach=False):
    """The team's task graph, ordered with Kahn's algorithm. Raises ``TeamError`` if it cannot be driven.

    Tasks of a robot ``i``:
      ``r<i>_clear_r<j>``  i has driven so far that the rest of its route is clear of j's route
                           (one for every robot j that has to wait for i)
      ``r<i>_arrive``      i has reached its goal
      ``r<i>_scan``        i has looked around at its goal (turns on the spot), if ``scan``
      ``r<i>_rescue``      i has driven up to the survivors it found and secured them, if ``approach``
    The first two need ``r<k>_clear_r<i>`` of every robot k that goes before i; the others follow one after
    the other. ``team_report`` needs every robot's last task.
    """
    members = tuple(sorted(members, key=lambda m: m.id))
    if not members:
        raise TeamError('There is no robot in the team.')
    engine = dag_engine()
    edges = tuple(precedence(members, distance))
    tasks, about, last = [], {}, []
    for m in members:
        needs = tuple('r%d_clear_r%d' % (e.first, m.id) for e in edges if e.then == m.id)
        for e in edges:
            if e.first == m.id:
                name = 'r%d_clear_r%d' % (m.id, e.then)
                tasks.append(engine.Task(name, needs))
                about[name] = dict(robot=m.id, kind='clear', other=e.then,
                                   text="%s drives clear of %s's route" % (label(m.id), label(e.then)))
        arrive = 'r%d_arrive' % m.id
        tasks.append(engine.Task(arrive, needs))
        about[arrive] = dict(robot=m.id, kind='arrive', other=None, text='%s drives to its goal' % label(m.id))
        final = arrive
        if scan:
            name = 'r%d_scan' % m.id
            tasks.append(engine.Task(name, (final,)))
            about[name] = dict(robot=m.id, kind='scan', other=None, text='%s looks around at its goal' % label(m.id))
            final = name
        if approach:
            name = 'r%d_rescue' % m.id
            tasks.append(engine.Task(name, (final,)))
            about[name] = dict(robot=m.id, kind='rescue', other=None,
                               text='%s drives to the survivors it found' % label(m.id))
            final = name
        last.append(final)
    tasks.append(engine.Task('team_report', tuple(last)))
    about['team_report'] = dict(robot=None, kind='report', other=None, text='Team report')
    try:
        order = engine.topological_order(tasks)             # Kahn: also the proof that nothing waits in a circle
    except engine.ConfigurationError as exc:
        raise TeamError('The team tasks cannot be ordered: %s' % exc) from None
    level, by_name = {}, {task.name: task for task in tasks}
    for name in order:
        level[name] = 1 + max((level[n] for n in by_name[name].needs), default=0)
    waves = tuple(tuple(n for n in order if level[n] == k) for k in range(1, max(level.values()) + 1))
    return TeamPlan(members, edges, tuple(tasks), about, tuple(order), waves, distance)


# ---------------------------------------------------------------------------- running it
class LiveGraph:
    """The state of a task graph while it is carried out; the rules are the scheduler's.

    A task is READY when every task it needs has SUCCEEDED (Kahn's algorithm, one step at a time: finishing
    a task lowers the count of open prerequisites of its successors, and a count of zero releases the task).
    A FAILED task makes everything that depends on it BLOCKED. Unlike the scheduler, every READY task may run
    at once: these tasks are robots driving, not programs sharing one processor.
    """

    def __init__(self, tasks):
        engine = dag_engine()
        engine.topological_order(tasks)                     # refuses a graph with a circle
        self.tasks, self.successors, self.open = engine._graph(tasks)
        self.needs = {task.name: tuple(dict.fromkeys(task.needs)) for task in self.tasks}
        self.state = {task.name: PENDING for task in self.tasks}
        self.note = {task.name: '' for task in self.tasks}
        for name, count in self.open.items():
            if count == 0:
                self.state[name] = READY

    def names(self, *states):
        return [task.name for task in self.tasks if self.state[task.name] in states]

    def start(self, name):
        if self.state[name] != READY:
            raise ValueError('%s is %s, not READY' % (name, self.state[name]))
        self.state[name] = RUNNING

    def succeed(self, name, note=''):
        """-> the names of the tasks this released."""
        if self.state[name] not in (READY, RUNNING):
            raise ValueError('%s is %s' % (name, self.state[name]))
        self.state[name], self.note[name] = SUCCEEDED, note
        released = []
        for successor in self.successors[name]:
            self.open[successor] -= 1
            if self.open[successor] == 0 and self.state[successor] == PENDING:
                self.state[successor] = READY
                released.append(successor)
        return released

    def fail(self, name, note=''):
        """-> the names of the tasks that are blocked by this failure."""
        if self.state[name] not in (READY, RUNNING):
            raise ValueError('%s is %s' % (name, self.state[name]))
        self.state[name], self.note[name] = FAILED, note
        blocked, waiting = [], list(self.successors[name])
        while waiting:
            successor = waiting.pop()
            if self.state[successor] in (PENDING, READY):
                self.state[successor] = BLOCKED
                self.note[successor] = 'waits for %s, which failed' % name
                blocked.append(successor)
                waiting.extend(self.successors[successor])
        return blocked

    def cancel(self, note=''):
        """Operator STOP: nothing that has not finished will run."""
        for name, state in self.state.items():
            if state in (PENDING, READY, RUNNING):
                self.state[name], self.note[name] = CANCELLED, note

    def finished(self):
        return not self.names(PENDING, READY, RUNNING)

    def status(self):
        if not self.finished():
            return RUNNING
        if all(state == SUCCEEDED for state in self.state.values()):
            return SUCCEEDED
        return CANCELLED if CANCELLED in self.state.values() else FAILED
