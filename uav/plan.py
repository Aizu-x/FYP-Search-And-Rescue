"""Flight plans for the drone: a list of steps it flies by itself, and the runner that flies them.

A plan is either written by the operator, step by step ("up 20 cm, turn right 360 degrees, forward 40 cm,
land"), or it is a flight that was flown by hand and remembered ("keys forward at 35 % for 1.8 s, hover 0.6 s
..."). Both are the same thing here: ``dict(name, speed, steps)`` with steps like

    {'do': 'up', 'cm': 20}          forward / back / left / right / up / down: a distance, 20-500 cm
    {'do': 'cw', 'deg': 360}        cw = turn right, ccw = turn left: 1-360 degrees
    {'do': 'wait', 's': 2.0}        hover
    {'do': 'photo'}                 save a photo
    {'do': 'speed', 'cm_s': 50}     the speed of the following distance moves, 10-100 cm/s
    {'do': 'sticks', 'rc': [0, 35, 0, 0], 's': 1.8}     stick values for a time (from a flight flown by hand)
    {'do': 'land'}                  only as the last step

How the steps are flown (``PlanRunner``):

* distances and turns are the Tello SDK's own movement commands ("up 20", "cw 360"); the drone measures them
  itself and answers "ok" when it has done one. Only then is the next step sent. Any other answer, or none in
  time, stops the plan: the drone hovers and the operator has the keys;
* long distances are flown in pieces of at most 1 m, so that LAND never has to wait long for a move to end
  (whether the drone accepts "land" in the middle of a move is not known; it is sent at once and repeated);
* "sticks" and "wait" steps send stick values 20 times a second, exactly like the keys do. A remembered flight
  is therefore the same stick movements with the same timing, not the same path: the drone does not know where
  it is, so it ends up close to where it was flown, not exactly there;
* the plan stops at once when the operator presses STOP PLAN or any flight key (the drone hovers), when LAND is
  pressed, when nothing is heard from the drone, or when the battery falls below 15 % (then it lands).

Starting a plan is the console's business (a two-step confirmation that shows the whole plan); this file never
starts one by itself.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

from .tello import FLYING, READY, TAKING_OFF, ZERO, TelloError

MOVES = ('forward', 'back', 'left', 'right', 'up', 'down')
TURNS = ('cw', 'ccw')
KINDS = MOVES + TURNS + ('wait', 'photo', 'speed', 'sticks', 'land')
LABELS = {'forward': 'Forward', 'back': 'Back', 'left': 'Left', 'right': 'Right', 'up': 'Up', 'down': 'Down',
          'cw': 'Turn right', 'ccw': 'Turn left', 'wait': 'Hover', 'photo': 'Photo', 'speed': 'Speed',
          'sticks': 'Keys', 'land': 'Land'}
CM = (20, 500)                       # what the drone accepts for one distance move
DEG = (1, 360)
WAIT_S = (0.2, 120.0)
STICKS_S = (0.05, 120.0)
SPEED = (10, 100)
DEFAULT_SPEED = 30
MAX_STEPS = 200
MAX_PIECE_CM = 100                   # a distance is flown in pieces of at most this
LOW_BATTERY = 15                     # below this a running plan lands
FLIGHT_NAME = 'Last manual flight'   # the plan that every flight flown by hand is saved as
LAST_HOVER_S = 1.0                   # of a remembered flight's hover before landing, at most this much is kept
WORKING = '_working'                 # the plan on the page, kept between runs of the console
_RESERVED = {'con', 'prn', 'aux', 'nul'} | {'com%d' % k for k in range(1, 10)} | {'lpt%d' % k for k in range(1, 10)}


# ============================================================================================ plans as data
def _number(raw, key, limits, what, number, whole=True):
    value = raw.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Step %d: %s is missing.' % (number, what))
    if whole:
        if abs(value - round(value)) > 1e-6:
            raise ValueError('Step %d: %s must be a whole number.' % (number, what))
        value = int(round(value))
    else:
        value = round(float(value), 2)
    if not limits[0] <= value <= limits[1]:
        raise ValueError('Step %d: %s must be between %g and %g.' % (number, what, limits[0], limits[1]))
    return value


def clean_step(raw, number=1):
    """One step with exactly the fields it needs, or ValueError (the message names the step)."""
    if not isinstance(raw, dict) or raw.get('do') not in KINDS:
        raise ValueError('Step %d: unknown kind of step.' % number)
    do = raw['do']
    if do in MOVES:
        return dict(do=do, cm=_number(raw, 'cm', CM, 'the distance (cm; the drone cannot do less than 20)', number))
    if do in TURNS:
        return dict(do=do, deg=_number(raw, 'deg', DEG, 'the angle (degrees)', number))
    if do == 'wait':
        return dict(do=do, s=_number(raw, 's', WAIT_S, 'the time (seconds)', number, whole=False))
    if do == 'speed':
        return dict(do=do, cm_s=_number(raw, 'cm_s', SPEED, 'the speed (cm/s)', number))
    if do == 'sticks':
        rc = raw.get('rc')
        if (not isinstance(rc, (list, tuple)) or len(rc) != 4
                or any(isinstance(v, bool) or not isinstance(v, int) or abs(v) > 100 for v in rc) or not any(rc)):
            raise ValueError('Step %d: the stick values are not valid.' % number)
        return dict(do=do, rc=[int(v) for v in rc],
                    s=_number(raw, 's', STICKS_S, 'the time (seconds)', number, whole=False))
    return dict(do=do)                                   # photo, land


def clean_plan(raw):
    """A checked copy of a plan: dict(name, speed, steps). ValueError with a message for the operator."""
    if not isinstance(raw, dict) or not isinstance(raw.get('steps'), list):
        raise ValueError('That is not a flight plan.')
    if len(raw['steps']) > MAX_STEPS:
        raise ValueError('A plan can have at most %d steps.' % MAX_STEPS)
    steps = [clean_step(step, number) for number, step in enumerate(raw['steps'], 1)]
    for number, step in enumerate(steps, 1):
        if step['do'] == 'land' and number != len(steps):
            raise ValueError('Step %d: "Land" has to be the last step.' % number)
    speed = raw.get('speed', DEFAULT_SPEED)
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not SPEED[0] <= speed <= SPEED[1]:
        raise ValueError('The speed of the moves must be between %d and %d cm/s.' % SPEED)
    name = raw.get('name', '')
    return dict(name=check_name(name) if name else '', speed=int(round(speed)), steps=steps)


def check_name(name):
    """A plan's name as it is used for its file, or ValueError."""
    if not isinstance(name, str):
        raise ValueError('The plan needs a name.')
    name = ' '.join(name.split())
    if not 1 <= len(name) <= 40 or any(not (ch.isalnum() or ch in ' -_') for ch in name):
        raise ValueError('A plan name is 1 to 40 letters, digits, spaces, - or _.')
    if name[0] in '_-' or name.lower() in _RESERVED:
        raise ValueError('That name cannot be used: start it with a letter or a digit.')
    return name


def _sticks_text(rc):
    words = [word for value, (plus, minus) in zip(rc, (('right', 'left'), ('forward', 'back'), ('up', 'down'),
                                                      ('turn right', 'turn left'))) if value
             for word in [plus if value > 0 else minus]]
    return '%s at %d %%' % (' + '.join(words), max(abs(v) for v in rc))


def describe(step):
    """One step in words: 'Up 20 cm', 'Turn right 360°', 'Keys: forward + turn right at 35 % for 1.8 s'."""
    do = step['do']
    if do in MOVES:
        return '%s %d cm' % (LABELS[do], step['cm'])
    if do in TURNS:
        return '%s %d°' % (LABELS[do], step['deg'])
    if do == 'wait':
        return 'Hover %g s' % step['s']
    if do == 'speed':
        return 'Speed %d cm/s' % step['cm_s']
    if do == 'sticks':
        return 'Keys: %s for %g s' % (_sticks_text(step['rc']), step['s'])
    return LABELS[do]


def estimate_seconds(plan, takeoff=True):
    """Roughly how long the plan takes (the drone's own pace is not known exactly)."""
    total, speed = (6.0 if takeoff else 0.0), float(plan['speed'])
    for step in plan['steps']:
        do = step['do']
        if do in MOVES:
            total += step['cm'] / speed + 1.5 * len(pieces(step['cm'], speed))
        elif do in TURNS:
            total += step['deg'] / 70.0 + 1.5
        elif do in ('wait', 'sticks'):
            total += step['s']
        elif do == 'speed':
            speed = float(step['cm_s'])
            total += 0.3
        elif do == 'land':
            total += 5.0
    return total


def pieces(cm, speed):
    """A distance as the pieces it is flown in: at most 1 m and about 3 s each, never less than 20 cm."""
    limit = max(2 * CM[0], min(MAX_PIECE_CM, int(speed * 3)))
    count = -(-int(cm) // limit)
    base, extra = divmod(int(cm), count)
    return [base + (1 if k < extra else 0) for k in range(count)]


def steps_from_flight(flight, shortest_hover=0.15):
    """A flight flown by hand (``TelloLink.take_flight()``) as plan steps: the same stick values for the same
    times, the photos where they were taken, and "Land" at the end."""
    steps = []
    for what, seconds in flight.get('segments', []):
        if isinstance(what, str):
            if what == 'photo':
                steps.append(dict(do='photo'))
            continue
        rc, seconds = [int(v) for v in what], round(float(seconds), 2)
        last = steps[-1] if steps else None
        if not any(rc):
            if seconds < shortest_hover:
                continue
            if last is not None and last['do'] == 'wait':
                last['s'] = round(min(WAIT_S[1], last['s'] + seconds), 2)
            else:
                steps.append(dict(do='wait', s=min(WAIT_S[1], max(WAIT_S[0], seconds))))
        elif seconds >= 0.03:
            if last is not None and last['do'] == 'sticks' and last['rc'] == rc:
                last['s'] = round(min(STICKS_S[1], last['s'] + seconds), 2)
            else:
                steps.append(dict(do='sticks', rc=rc, s=min(STICKS_S[1], max(STICKS_S[0], seconds))))
    if steps and steps[-1]['do'] == 'wait':               # the hover before the landing: long enough to come to
        steps[-1]['s'] = min(steps[-1]['s'], LAST_HOVER_S)    # rest, but not the time it took to find the LAND key
    steps.append(dict(do='land'))
    if len(steps) > MAX_STEPS:
        raise ValueError('That flight has %d movements: too many for one plan (at most %d).' % (len(steps), MAX_STEPS))
    return steps


# ============================================================================================ plans as files
class PlanStore:
    """Saved plans: one small JSON file per plan in one folder."""

    def __init__(self, folder):
        self.folder = Path(folder)

    def _path(self, name, internal=False):
        return self.folder / ((name if internal else check_name(name)) + '.json')

    def names(self):
        try:
            found = [p.stem for p in self.folder.glob('*.json') if not p.stem.startswith('_')]
        except OSError:
            return []
        return sorted(found, key=str.lower)

    def load(self, name, internal=False):
        path = self._path(name, internal)
        try:
            if path.stat().st_size > 300000:
                raise ValueError('The plan file "%s" is too large.' % path.name)
            raw = json.loads(path.read_text(encoding='utf-8'))
        except FileNotFoundError:
            raise ValueError('There is no saved plan "%s".' % name)
        except (OSError, ValueError) as exc:
            raise ValueError('The plan "%s" cannot be read (%s).' % (name, exc))
        kept = raw.get('name', '') if isinstance(raw, dict) else ''
        plan = clean_plan(dict(raw, name='') if isinstance(raw, dict) else raw)
        if internal:                                     # the plan on the page remembers which saved plan it was
            try:
                plan['name'] = check_name(kept) if kept else ''
            except ValueError:
                plan['name'] = ''
        else:
            plan['name'] = name
        return plan

    def save(self, plan, name, internal=False):
        path = self._path(name, internal)
        text = json.dumps(dict(name=plan.get('name', '') if internal else name, speed=plan['speed'],
                               steps=plan['steps']), indent=1)
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            temp = path.with_name(path.name + '.tmp')
            temp.write_text(text, encoding='utf-8')
            os.replace(temp, path)
        except OSError as exc:
            raise ValueError('The plan could not be saved (%s).' % (exc.strerror or exc))
        return path

    def delete(self, name):
        try:
            self._path(name).unlink()
        except FileNotFoundError:
            raise ValueError('There is no saved plan "%s".' % name)
        except OSError as exc:
            raise ValueError('The plan could not be deleted (%s).' % (exc.strerror or exc))


# ============================================================================================ flying a plan
class _Over(Exception):
    def __init__(self, state, message):
        super().__init__(message)
        self.state, self.message = state, message


class PlanRunner:
    """Flies one plan with one ``TelloLink``, in its own thread.

    ``state``: ready, running, then done / stopped (by the operator) / failed. ``index``: the step being flown
    (-1 while taking off). ``stop(reason)`` makes the drone hover and gives the keys back; it cannot cut short a
    movement command that is already running (at most one piece of 1 m)."""

    def __init__(self, link, plan, take_photo=None, clock=time.monotonic, low_battery=LOW_BATTERY):
        self.link, self.plan = link, plan
        self.steps = plan['steps']
        self.take_photo = take_photo
        self.clock = clock
        self.low_battery = low_battery
        self.speed = plan['speed']
        self.state, self.index, self.message = 'ready', -1, ''
        self.took_off = False
        self.started = self.ended = None
        self._stop = threading.Event()
        self._reason = ''
        self._thread = None

    # ------------------------------------------------------------------ control
    def start(self):
        self.state, self.started = 'running', self.clock()
        self._thread = threading.Thread(target=self._run, name='uav-plan', daemon=True)
        self._thread.start()
        return self

    def stop(self, reason='STOP PLAN pressed'):
        if not self._stop.is_set():
            self._reason = reason
            self._stop.set()

    @property
    def active(self):
        return self.state == 'running'

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)

    def snapshot(self):
        index, total = self.index, len(self.steps)
        text = 'Take off' if index < 0 else describe(self.steps[index]) if index < total else ''
        end = self.ended if self.ended is not None else self.clock()
        return dict(state=self.state, index=index, total=total, text=text, message=self.message,
                    elapsed=0.0 if self.started is None else end - self.started, name=self.plan.get('name', ''))

    # ------------------------------------------------------------------ the flight
    def _run(self):
        link = self.link
        link.plan_begin()
        try:
            if link.state == READY:
                self._stop.wait(min(3.0, link.quiet_s()))         # (just landed: answers are still settling)
                if self._stop.is_set():
                    raise _Over('stopped', 'Plan stopped before the take-off (%s).' % self._reason)
                link.takeoff()
                self.took_off = True
                end = self.clock() + link.cfg.takeoff_timeout_s + 3.0
                while link.state == TAKING_OFF and self.clock() < end:
                    self._stop.wait(0.05)
                if link.state != FLYING:
                    raise _Over('failed', 'The drone did not take off: the plan was not flown.')
                self._hover(1.0)                              # let it settle above the take-off point
            elif link.state != FLYING:
                raise _Over('failed', 'The drone is not ready: the plan was not flown.')
            self._check()
            self._command('speed %d' % self.speed, 4.0)
            for index, step in enumerate(self.steps):
                self._check()                                 # (still names the step that was just flown)
                self.index = index
                self._fly(step)
                following = self.steps[index + 1]['do'] if index + 1 < len(self.steps) else ''
                if step['do'] == 'sticks' and following != 'sticks':
                    link.plan_sticks(ZERO)
            landed = bool(self.steps) and self.steps[-1]['do'] == 'land'
            self.index = len(self.steps)
            self.message = ('Plan finished: the drone has landed.' if landed else
                            'Plan finished: the drone hovers. You have control (keys), or press LAND.')
            self.state = 'done'
        except _Over as over:
            self.state, self.message = over.state, over.message
        except TelloError as exc:
            self.state, self.message = 'failed', 'The plan was not flown: %s' % exc
        except Exception as exc:                              # a bug must end in a hover, not in a moving drone
            self.state, self.message = 'failed', 'Plan error (%s): the plan was stopped.' % exc
        finally:
            link.plan_end()
            self.ended = self.clock()

    def _where(self):
        return 'take-off' if self.index < 0 else 'step %d (%s)' % (self.index + 1, describe(self.steps[self.index]))

    def _check(self):
        """Raise when the plan must not go on."""
        link = self.link
        if link.state != FLYING:
            raise _Over('stopped', 'Plan ended at %s: the drone is landing.' % self._where())
        if self._stop.is_set():
            raise _Over('stopped', 'Plan stopped at %s (%s). The drone hovers: you have control.'
                        % (self._where(), self._reason))
        if link.takeover:
            raise _Over('stopped', 'Plan stopped at %s: you pressed a flight key. You have control.' % self._where())
        battery = link.battery()
        if battery is not None and battery < self.low_battery:
            link.land('battery %d %% during the plan' % battery)
            raise _Over('failed', 'Battery %d %%: the plan was stopped at %s and the drone is landing.'
                        % (battery, self._where()))
        if link.snapshot()['silent']:
            raise _Over('failed', 'Nothing is heard from the drone: the plan was stopped at %s.' % self._where())

    def _hover(self, seconds):
        end = self.clock() + seconds
        self.link.plan_sticks(ZERO)
        while True:
            self._check()
            left = end - self.clock()
            if left <= 0:
                return
            self._stop.wait(min(0.05, left))

    def _command(self, text, timeout, again=True):
        answer = self.link.plan_command(text, timeout)
        if answer is None:
            self._check()                                     # LAND, a key or STOP PLAN explains it
            raise _Over('failed', 'No answer from the drone to "%s" at %s. Plan stopped: the drone hovers, you '
                                  'have control.' % (text, self._where()))
        low = answer.strip().lower()
        if low == 'ok':
            return
        if again and 'joystick' in low:                       # it still had stick values in mind: once more
            self._stop.wait(1.0)
            self._check()
            return self._command(text, timeout, again=False)
        raise _Over('failed', 'The drone answered "%s" to "%s" at %s. Plan stopped: the drone hovers, you have '
                              'control.' % (answer.strip()[:40], text, self._where()))

    def _fly(self, step):
        link, do = self.link, step['do']
        if do in MOVES:
            for piece in pieces(step['cm'], self.speed):
                self._check()
                self._command('%s %d' % (do, piece), piece / float(self.speed) * 2.5 + 6.0)
        elif do in TURNS:
            self._command('%s %d' % (do, step['deg']), step['deg'] / 30.0 + 6.0)
        elif do == 'wait':
            self._hover(step['s'])
        elif do == 'sticks':
            end = self.clock() + step['s']
            if not link.plan_sticks(step['rc']):
                self._check()
            while True:
                self._check()
                left = end - self.clock()
                if left <= 0:
                    break
                self._stop.wait(min(0.02, left))
        elif do == 'photo':
            if self.take_photo is not None:
                self.take_photo()
        elif do == 'speed':
            self.speed = step['cm_s']
            self._command('speed %d' % self.speed, 4.0)
        elif do == 'land':
            link.land('last step of the plan')
            end = self.clock() + link.cfg.land_timeout_s + 5.0
            while link.state != READY and self.clock() < end:
                time.sleep(0.05)
            if link.state != READY:
                raise _Over('failed', 'The landing at the end of the plan was not confirmed. Press LAND.')
