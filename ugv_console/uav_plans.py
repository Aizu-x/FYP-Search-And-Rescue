"""Planned flight in the UAV section: the flight plan on the page, saved plans, and flying a plan.

Second half of the drone part of ``ConsoleApp`` (the first is ``uav_console.py``). ``uav/plan.py`` knows what a
plan is and flies it; here are the operator's commands.

* The plan on the page is edited as a whole (``uav_plan_set``), checked on every change, and kept between runs
  of the console. Named plans are small files in ``data/uav_plans``.
* Every flight flown with the keys alone is remembered as the plan "Last manual flight", so it can be flown
  again, changed, or saved under its own name.
* Flying a plan is autonomous movement: it needs the same two-step confirmation as a take-off, and the
  confirmation shows the whole plan. The plan that is flown is the one that was shown.
* STOP PLAN, any flight key, LAND / Esc, a closed page and a silent drone all end a plan (``uav/plan.py``,
  ``server.py``).
"""
from __future__ import annotations

import copy
import hmac
import secrets
import time

from ugv_nav.recorder import RecorderError, check_folder
from uav.plan import (CM, DEFAULT_SPEED, DEG, FLIGHT_NAME, MAX_STEPS, SPEED, WAIT_S, WORKING, PlanRunner, PlanStore,
                      check_name, clean_plan, describe, estimate_seconds, steps_from_flight)
from uav.tello import AIRBORNE, FLYING, READY

from .options import arg_bool

PLAN_CODE_S = 45.0                   # the "fly this plan?" question is valid this long
SHOWN_STEPS = 14                     # steps listed in that question


def _result(ok, message='', level=None, **extra):
    return dict(ok=bool(ok), message=message, level=level or ('info' if ok else 'error'), **extra)


def _clock(seconds):
    seconds = int(round(seconds))
    return '%d s' % seconds if seconds < 60 else '%d min %02d s' % (seconds // 60, seconds % 60)


class UavPlans:
    def _plan_init(self):
        self.uav_plans = PlanStore(self.edit_dir.parent / 'uav_plans')
        self.uav_plan = dict(name='', speed=DEFAULT_SPEED, steps=[])     # the plan on the page
        self.uav_plan_dirty = False          # changed since it was saved or loaded
        self.uav_run = None                  # PlanRunner while a plan is flown
        self.uav_run_last = None             # what became of the last one
        self.uav_flight = None               # dict(steps, seconds) about the last flight flown by hand
        self.uav_flights = 0
        self._plan_code = None               # (one-time code, expires, the plan that was shown)
        self._plan_names = self.uav_plans.names()
        try:
            self.uav_plan = self.uav_plans.load(WORKING, internal=True)
        except ValueError:
            pass                             # nothing kept yet (or the file is damaged): start with an empty plan

    # ------------------------------------------------------------------ small helpers
    def uav_plan_running(self):
        run = self.uav_run
        return run is not None and run.active

    def _plan_free(self):
        if self.uav_plan_running():
            raise ValueError('A plan is being flown: stop it first.')

    def _plan_changed(self, plan, dirty):
        self.uav_plan, self.uav_plan_dirty, self._plan_code = plan, dirty, None
        if not self.uav_plan_running():
            self.uav_run_last = None             # what the page shows about the last run was about the old plan
        try:
            self.uav_plans.save(plan, WORKING, internal=True)
        except ValueError:
            pass                             # (the plan still works; it is only not kept for the next start)
        self._bump('uavplan', 'uav')

    def _plan_unsaved(self, args, what):
        """The question to ask before the plan on the page is replaced, or None."""
        if not self.uav_plan_dirty or not self.uav_plan['steps'] or arg_bool(args, 'confirm'):
            return None
        return _result(False, 'The plan on the page has changes that are not saved. %s' % what, 'warn',
                       need_confirm='uav_plan_discard')

    # ------------------------------------------------------------------ editing and the saved plans
    def cmd_uav_plan_set(self, args):
        """The whole plan as it is on the page now (sent after every change)."""
        raw = args.get('plan')
        with self._lock:
            self._plan_free()
            if not isinstance(raw, dict):
                raise ValueError('That is not a flight plan.')
            plan = clean_plan(dict(raw, name=self.uav_plan['name']))
            if plan != self.uav_plan:
                self._plan_changed(plan, True)
            return _result(True)

    def cmd_uav_plan_new(self, args):
        with self._lock:
            self._plan_free()
            ask = self._plan_unsaved(args, 'Start an empty plan anyway?')
            if ask:
                return ask
            self._plan_changed(dict(name='', speed=self.uav_plan['speed'], steps=[]), False)
            return self._ok('Empty plan. Add steps, then RUN PLAN.')

    def cmd_uav_plan_save(self, args):
        name = check_name(args.get('name'))
        with self._lock:
            if name.lower() == FLIGHT_NAME.lower():
                raise ValueError('"%s" is where the console keeps your last flight by hand: choose another name.'
                                 % FLIGHT_NAME)
            if not self.uav_plan['steps']:
                raise ValueError('The plan is empty: nothing to save.')
            taken = [n for n in self._plan_names if n.lower() == name.lower()]
            if taken and taken[0].lower() != self.uav_plan['name'].lower() and not arg_bool(args, 'confirm'):
                return _result(False, 'A plan called "%s" exists already. Replace it?' % taken[0], 'warn',
                               need_confirm='uav_plan_overwrite')
            name = taken[0] if taken else name
            self.uav_plans.save(self.uav_plan, name)
            self._plan_names = self.uav_plans.names()
            self._plan_changed(dict(self.uav_plan, name=name), False)
            return self._ok('Plan saved as "%s".' % name, 'success')

    def _plan_load(self, args, name, missing):
        self._plan_free()
        if name not in self._plan_names:
            raise ValueError(missing)
        ask = self._plan_unsaved(args, 'Load "%s" anyway?' % name)
        if ask:
            return ask
        plan = self.uav_plans.load(name)
        self._plan_changed(plan, False)
        return self._ok('Plan "%s" loaded: %d steps, about %s.'
                        % (name, len(plan['steps']), _clock(estimate_seconds(plan))))

    def cmd_uav_plan_load(self, args):
        name = check_name(args.get('name'))
        with self._lock:
            return self._plan_load(args, name, 'There is no saved plan "%s".' % name)

    def cmd_uav_plan_from_flight(self, args):
        """The last flight flown by hand becomes the plan on the page."""
        with self._lock:
            return self._plan_load(args, FLIGHT_NAME, 'No flight by hand is remembered yet: fly one in Manual '
                                   'control first (take off, fly with the keys, land).')

    def cmd_uav_plan_delete(self, args):
        name = check_name(args.get('name'))
        with self._lock:
            if name not in self._plan_names:
                raise ValueError('There is no saved plan "%s".' % name)
            if not arg_bool(args, 'confirm'):
                return _result(False, 'Delete the saved plan "%s"? (The plan on the page stays.)' % name, 'warn',
                               need_confirm='uav_plan_delete')
            self.uav_plans.delete(name)
            self._plan_names = self.uav_plans.names()
            if self.uav_plan['name'] == name:
                self.uav_plan = dict(self.uav_plan, name='')
                self.uav_plan_dirty = bool(self.uav_plan['steps'])
            self._bump('uavplan')
            return self._ok('Saved plan "%s" deleted.' % name)

    # ------------------------------------------------------------------ flying the plan
    def _plan_problem(self):
        link = self.uav
        if link is None:
            return 'Connect the drone first.'
        if self.opts['section'] != 'uav':
            return 'Switch to the UAV section first.'
        if self.uav_plan_running():
            return 'A plan is being flown.'
        if not self.uav_plan['steps']:
            return 'The plan is empty: add steps first.'
        if self.is_moving():
            return 'The robot is moving: stop it before the drone flies.'
        if link.state not in (READY, FLYING):
            return 'The drone is taking off or landing: wait a moment.'
        snap = link.snapshot()
        if snap['silent']:
            return 'Nothing is heard from the drone right now.'
        if snap['battery'] is None:
            return 'No battery reading from the drone yet: wait a moment.'
        if snap['battery'] < link.cfg.min_battery:
            return 'Battery %d %%: too low to fly a plan (at least %d %% is needed).' % (snap['battery'],
                                                                                       link.cfg.min_battery)
        if link.state == FLYING and any(snap['sticks']):
            return 'Let go of the flight keys first.'
        return ''

    def cmd_uav_plan_prepare(self, args):
        with self._lock:
            problem = self._plan_problem()
            if problem:
                return self._fail(problem)
            link, plan = self.uav, copy.deepcopy(self.uav_plan)
            on_ground = link.state == READY
            lines = (['Take off'] if on_ground else []) + [describe(step) for step in plan['steps']]
            listing = '\n'.join('%2d.  %s' % (k, line) for k, line in enumerate(lines[:SHOWN_STEPS], 1))
            if len(lines) > SHOWN_STEPS:
                listing += '\n      … and %d more steps' % (len(lines) - SHOWN_STEPS)
            ending = ('It lands at the end.' if plan['steps'][-1]['do'] == 'land' else
                      'At the end it HOVERS and you have the keys.')
            copied = any(step['do'] == 'sticks' for step in plan['steps'])
            text = ('The %s flies this plan BY ITSELF%s:\n\n%s\n\nAbout %s, battery %d %%. %s%s\n\n'
                    'Check first: free space along the whole route, nobody in it.\n'
                    'Esc = LAND.  STOP PLAN or any flight key = it hovers and you have control.'
                    % ('simulated drone' if self.uav_sim is not None else 'drone',
                       '' if on_ground else ', starting from where it hovers now', listing,
                       _clock(estimate_seconds(plan, takeoff=on_ground)), link.snapshot()['battery'], ending,
                       '\nA copied flight repeats your key presses with the same timing: it ends up close to '
                       'where you flew, not exactly there.' if copied else ''))
            code = secrets.token_hex(8)
            self._plan_code = (code, time.monotonic() + PLAN_CODE_S, plan)
            return _result(True, '', nonce=code, title='Fly the plan', text=text, confirm_label='Fly the plan')

    def cmd_uav_plan_confirm(self, args):
        code = args.get('nonce')
        with self._lock:
            asked, self._plan_code = self._plan_code, None
            if (not asked or not isinstance(code, str) or not hmac.compare_digest(asked[0], code)
                    or time.monotonic() > asked[1]):
                return self._fail('The question expired: press RUN PLAN again.')
            problem = self._plan_problem()
            if problem:
                return self._fail(problem)
            plan = asked[2]
            if plan != self.uav_plan:
                return self._fail('The plan changed since you were asked: press RUN PLAN again.')
            self._uav_code, self._uav_key_t = None, None
            self.uav_run_last = None
            self.uav_run = PlanRunner(self.uav, plan, take_photo=self._plan_photo).start()
            self._uav_say('Flying the plan%s by itself: %d steps. Esc = LAND, any flight key = you take over.'
                          % (' "%s"' % plan['name'] if plan['name'] else '', len(plan['steps'])), 'warn')
            self._bump('uavplan')
            return _result(True, self.uav_note, 'success')

    def cmd_uav_plan_stop(self, args):
        """STOP PLAN: the drone hovers, the keys work again. Never waits for a lock."""
        run = self.uav_run
        if run is None or not run.active:
            return _result(True, 'No plan is being flown.')
        run.stop(str(args.get('reason') or 'STOP PLAN pressed')[:80])
        self._bump('uav')
        return _result(True, 'Stopping the plan: the drone hovers after the move it is doing.', 'warn')

    def _plan_photo(self):
        """A "Photo" step (called by the plan runner)."""
        video = self.uav_video
        if video is None or video.dead:
            return
        try:
            video.photo(check_folder(self._uav_folder()))
        except RecorderError:
            pass

    # ------------------------------------------------------------------ housekeeping (from the ticker)
    def _plan_tick(self, link, state, was):
        run = self.uav_run
        if run is not None and not run.active and run.state != 'ready':        # the plan is over: say how it ended
            self.uav_run, self.uav_run_last = None, run.snapshot()
            self._uav_say(run.message, {'done': 'success', 'stopped': 'warn'}.get(run.state, 'error'))
            self._bump('uavplan')
        if state == READY and was in AIRBORNE:                                 # a flight has just ended
            flight = link.take_flight()
            if flight and flight.get('by_hand'):
                self._plan_remember(flight)

    def _plan_remember(self, flight):
        """Keep a flight that was flown with the keys alone as the plan "Last manual flight"."""
        try:
            steps = steps_from_flight(flight)
            if not any(step['do'] == 'sticks' for step in steps):
                return                                                         # it only hovered: nothing to repeat
            self.uav_plans.save(dict(speed=self.uav_plan['speed'], steps=steps), FLIGHT_NAME)
        except ValueError as exc:
            self._log_line('warn', 'This flight was not remembered as a plan: %s' % exc)
            return
        self._plan_names = self.uav_plans.names()
        self.uav_flight = dict(steps=len(steps), seconds=int(round(flight['seconds'])))
        self.uav_flights += 1
        self._log_line('info', 'This flight is remembered as the plan "%s" (%d steps). Planned flight > My last '
                               'flight flies it again.' % (FLIGHT_NAME, len(steps)))
        self._bump('uavplan')

    def _plan_drop(self):
        run, self.uav_run = self.uav_run, None
        if run is not None and run.active:
            run.stop('the drone was disconnected')
        self.uav_run_last = None
        self._plan_code = None

    # ------------------------------------------------------------------ what the page shows
    def _plan_run_view(self):
        run = self.uav_run
        info = run.snapshot() if run is not None else self.uav_run_last
        if info is None:
            return None
        return dict(state=info['state'], index=info['index'], total=info['total'], text=info['text'],
                    message=info['message'], seconds=int(info['elapsed']), name=info['name'])

    def _view_uavplan(self):
        plan = self.uav_plan
        steps = [dict(step, text=describe(step)) for step in plan['steps']]
        return self.clean(dict(
            name=plan['name'], speed=plan['speed'], steps=steps, dirty=self.uav_plan_dirty,
            saved=list(self._plan_names), seconds=int(round(estimate_seconds(plan))),
            lands=bool(steps) and steps[-1]['do'] == 'land', flight=self.uav_flight, flights=self.uav_flights,
            flight_name=FLIGHT_NAME, has_flight=FLIGHT_NAME in self._plan_names,
            limits=dict(cm=list(CM), deg=list(DEG), wait=list(WAIT_S), speed=list(SPEED), steps=MAX_STEPS)))
