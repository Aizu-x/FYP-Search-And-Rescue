"""The UAV section of the console: fly a Tello from the keyboard, watch its camera, keep its footage.

This is the part of ``ConsoleApp`` that deals with the drone (the class below is mixed into it). The drone link
(``uav/tello.py``), the video worker (``uav/video.py``) and the simulated drone (``uav/sim.py``) do the work; here
are the operator's commands and what the page is shown.

Rules:

* Take-off is a two-step handshake like the robot's AUTHORIZE (``uav_takeoff_prepare`` -> summary + one-time
  code, ``uav_takeoff_confirm`` -> go): a single stray request can never start the motors.
* LAND never waits for anything, and STOP / Esc lands the drone as well.
* One moving machine at a time: no take-off while the robot drives, no robot authorization while the drone is
  in the air, and the section cannot be left while the drone is in the air.
* The keys only steer while the page keeps reporting them; a page that goes quiet means hover, and a page that
  is gone for a few seconds means land (``server.py``).
"""
from __future__ import annotations

import hmac
import secrets
import threading
import time
from pathlib import Path

from ugv_nav.recorder import RecorderError, check_folder
from uav.sim import SimProcess
from uav.tello import (AIRBORNE, DISCONNECTED, FLYING, KEYS, LANDING, READY, TAKING_OFF, TelloConfig, TelloError,
                       TelloLink, check_host)
from uav.video import VIDEO_PORT, VideoError, VideoFeed, default_folder

from .options import arg_bool, arg_choice

SECTIONS = ('ugv', 'uav')
ROOT = Path(__file__).resolve().parents[1]
TAKEOFF_CODE_S = 30.0                # the take-off question is valid this long
RECORD_TAIL_S = 2.0                  # filmed after the landing
LOW_BATTERY = 25
STATE_TEXT = {DISCONNECTED: 'Not connected', 'connecting': 'Connecting…', READY: 'On the ground',
              TAKING_OFF: 'Taking off', FLYING: 'Flying', LANDING: 'Landing'}
FIREWALL_HINT = ('Windows Firewall is probably blocking it: allow Python for Private AND Public networks '
                 '(Windows Security > Firewall > Allow an app), then reconnect.')


def _result(ok, message='', level=None, **extra):
    return dict(ok=bool(ok), message=message, level=level or ('info' if ok else 'error'), **extra)


class UavConsole:
    def _uav_init(self):
        self.uav = None                      # TelloLink while connected
        self.uav_video = None                # VideoFeed (the worker process)
        self.uav_sim = None                  # SimProcess when the simulated drone is used
        self.uav_connecting = False
        self.uav_note, self.uav_note_level = '', 'info'
        self.uav_video_note = ''             # why there is no video
        self.uav_rec = None                  # dict(auto, started, stop_at) while footage is being recorded
        self.uav_last = None                 # dict about the newest saved footage
        self.uav_made = 0
        self.uav_last_photo = None
        self.uav_photos = 0
        self._uav_code = None                # (one-time code, expires) of the take-off question
        self._uav_gen = 0                    # connection attempts; a late connect thread must not install itself
        self._uav_was = DISCONNECTED
        self._uav_key_t = None
        self._uav_stream = dict(since=None, restarts=0, busy=False)

    # ------------------------------------------------------------------ small helpers
    def _uav_say(self, text, level='info', log=True):
        self.uav_note, self.uav_note_level = text, level
        if log:
            self._log_line(level, text)
        self._bump('uav')

    def uav_airborne(self):
        link = self.uav
        return link is not None and link.state in AIRBORNE

    def _uav_folder(self):
        custom = str(self.opts['uav_dir']).strip()
        if not custom:
            return default_folder()
        folder = Path(custom).expanduser()
        return folder if folder.is_absolute() else ROOT / folder

    # ------------------------------------------------------------------ section
    def cmd_set_section(self, args):
        section = arg_choice(args, 'section', SECTIONS)
        with self._lock:
            if section == self.opts['section']:
                return _result(True)
            if section == 'uav' and self.is_moving():
                return self._fail('The robot is moving: stop it before you switch to the drone.')
            if section == 'ugv' and self.uav_airborne():
                return self._fail('The drone is in the air: land it before you leave the UAV section.')
            self.opts['section'] = section
            self.sim.pause()
            self._bump('settings', 'uav', 'offline')
            self._settings_dirty = True
            return self._ok('UAV section: connect the drone, take off, fly with W A S D and the arrow keys.'
                            if section == 'uav' else 'UGV section.')

    # ------------------------------------------------------------------ connecting
    def cmd_uav_connect(self, args):
        with self._lock:
            if self.uav is not None:
                return self._ok('The drone is already connected.')
            if self.uav_connecting:
                return _result(True, 'Still connecting…')
            sim = bool(self.opts['uav_sim'])
            host = '127.0.0.1' if sim else check_host(self.opts['uav_ip'])
            self.uav_connecting = True
            self._uav_gen += 1
            generation = self._uav_gen
            self.uav_video_note = ''
            self._uav_say('Starting the simulated drone…' if sim else 'Connecting to the drone at %s…' % host)
        threading.Thread(target=self._uav_connect, args=(generation, sim, host), name='uav-connect',
                         daemon=True).start()
        return _result(True, self.uav_note)

    def _uav_connect(self, generation, sim, host):
        link = video = process = None
        video_note = ''

        def cancelled():
            return generation != self._uav_gen or self._closing.is_set()
        try:
            if sim:
                process = SimProcess().start()
                link = TelloLink(TelloConfig(host=host, cmd_port=process.cmd_port, state_port=0, local_ip=host))
                link.bind()
                process.set_ports(link.state_port, 0)
            else:
                link = TelloLink(TelloConfig(host=host))
                link.bind()
            link.handshake()
            if not cancelled():                    # the drone answers: now its camera
                try:
                    video = VideoFeed(link.local_ip, 0 if sim else VIDEO_PORT, host,
                                      command_port=link.cfg.cmd_port).start()
                    if sim:
                        process.set_ports(link.state_port, video.port)
                    if not link.stream(True):
                        video_note = 'The drone did not accept "streamon", so there is no video.'
                except VideoError as exc:
                    video, video_note = None, 'No video: %s.' % exc
        except (TelloError, RuntimeError, OSError) as exc:
            self._uav_release(link, video, process)
            with self._lock:
                if generation == self._uav_gen:
                    self.uav_connecting = False
                    self._uav_say(str(exc), 'warn')
            return
        with self._lock:
            if cancelled():
                self._uav_release(link, video, process)
                return
            self.uav, self.uav_video, self.uav_sim = link, video, process
            self.uav_connecting = False
            self.uav_video_note = video_note
            self._uav_was = link.state
            self._uav_stream = dict(since=time.monotonic(), restarts=0, busy=False)
            self._uav_say('%s connected. Press TAKE OFF when the space around it is clear.'
                          % ('Simulated drone' if sim else 'Drone at %s' % host), 'success')
            if video_note:
                self._log_line('warn', video_note)

    @staticmethod
    def _uav_release(link, video, process):
        for closer in ((lambda: video.stop(wait=True)) if video is not None else None,
                       (lambda: link.close(land=True)) if link is not None else None,
                       process.stop if process is not None else None):
            try:
                if closer is not None:
                    closer()
            except Exception:
                pass

    def cmd_uav_disconnect(self, args):
        with self._lock:
            if self.uav is None and not self.uav_connecting:
                return self._ok('The drone is not connected.')
            if self.uav_airborne():
                return self._fail('The drone is in the air: land it first (LAND or Esc).')
            self._uav_drop()
            self._uav_say('Drone disconnected.')
            return _result(True, self.uav_note)

    def _uav_drop(self):
        """Let go of the drone, the video worker and the simulated drone (the footage is saved first)."""
        self._uav_gen += 1                     # cancels a connect that is still running
        self.uav_connecting = False
        link, video, process = self.uav, self.uav_video, self.uav_sim
        self.uav = self.uav_video = self.uav_sim = None
        self._uav_code = None
        self._uav_was = DISCONNECTED
        self._uav_release(link, video, process)
        if video is not None:
            self._uav_video_events(video)      # "footage saved: ..."
        self.uav_rec = None
        self.uav_video_note = ''
        self._bump('uav')

    def _uav_close(self):
        """Program exit: a drone in the air is told to land, the footage is saved."""
        link = self.uav
        if link is not None and link.state in AIRBORNE:
            link.land('the console is closing')
            print('The console is closing while the drone is in the air: LAND sent.')
        self._uav_drop()

    # ------------------------------------------------------------------ flying
    def _uav_takeoff_problem(self):
        link = self.uav
        if link is None:
            return 'Connect the drone first.'
        if self.opts['section'] != 'uav':
            return 'Switch to the UAV section first.'
        if self.is_moving():
            return 'The robot is moving: stop it before the drone takes off.'
        if link.state != READY:
            return {FLYING: 'The drone is already flying.', TAKING_OFF: 'The drone is already taking off.',
                    LANDING: 'The drone is landing: wait until it is down.'}.get(link.state, 'The drone is not ready.')
        snap = link.snapshot()
        if snap['silent']:
            return 'Nothing is heard from the drone right now: no take-off.'
        if snap['battery'] is None:
            return 'No battery reading from the drone yet: wait a moment.'
        if snap['battery'] < link.cfg.min_battery:
            return 'Battery %d %%: too low to take off (at least %d %% is needed).' % (snap['battery'], link.cfg.min_battery)
        return ''

    def cmd_uav_takeoff_prepare(self, args):
        with self._lock:
            problem = self._uav_takeoff_problem()
            if problem:
                return self._fail(problem)
            snap = self.uav.snapshot()
            video = self.uav_video.status() if self.uav_video is not None else {}
            live = video.get('frame_age') is not None and video['frame_age'] < 2.0
            code = secrets.token_hex(8)
            self._uav_code = (code, time.monotonic() + TAKEOFF_CODE_S)
            filming = ('The flight is filmed and saved in %s.' % self._uav_folder() if self.opts['uav_record'] and live
                       else 'The flight is NOT filmed (no live video).' if self.opts['uav_record']
                       else 'The flight is not filmed (recording is switched off).')
            text = ('The %s starts its motors, rises and hovers.\n\nBattery %d %%.  %s\n\n'
                    'Check first: nobody near the drone, free space around and above it, it stands level.\n'
                    'LAND (Esc) brings it down at any time.'
                    % ('simulated drone' if self.uav_sim is not None else 'drone', snap['battery'], filming))
            return _result(True, '', nonce=code, title='Take off', text=text, confirm_label='Take off')

    def cmd_uav_takeoff_confirm(self, args):
        code = args.get('nonce')
        with self._lock:
            asked, self._uav_code = self._uav_code, None
            if (not asked or not isinstance(code, str) or not hmac.compare_digest(asked[0], code)
                    or time.monotonic() > asked[1]):
                return self._fail('The take-off question expired: press TAKE OFF again.')
            problem = self._uav_takeoff_problem()
            if problem:
                return self._fail(problem)
            try:
                self.uav.takeoff()
            except TelloError as exc:
                return self._fail(str(exc))
            self._uav_key_t = None
            if self.uav_video is not None:
                self.uav_video.set_airborne(True)      # (if the console dies now, the video worker lands the drone)
            if self.opts['uav_record'] and self.uav_rec is None:
                self._uav_record_start(auto=True)
            self._uav_say('Taking off…', 'warn')
            return _result(True, self.uav_note, 'success')

    def cmd_uav_land(self, args):
        """LAND: never waits for a lock or a slow operation."""
        link = self.uav
        self._uav_code = None
        if link is None:
            return _result(False, 'The drone is not connected.', 'warn')
        was = link.state
        link.land(str(args.get('reason') or 'LAND pressed')[:80])
        self._bump('uav')
        if was in AIRBORNE or link.state == LANDING:
            self._uav_say('LAND: the drone is coming down.', 'warn', log=False)
        else:
            self._uav_say('The drone is on the ground (LAND was sent anyway).', log=False)
        return _result(True, self.uav_note, 'warn')

    def _uav_stop(self, reason):
        """Part of STOP / Esc: a drone in the air lands. True when a landing was commanded."""
        link = self.uav
        self._uav_code = None
        if link is None:
            return False
        was = link.state
        link.land(reason)
        self._bump('uav')
        return was in AIRBORNE or link.state == LANDING

    def cmd_uav_emergency(self, args):
        link = self.uav
        if link is None:
            return self._fail('The drone is not connected.')
        if not arg_bool(args, 'confirm'):
            return _result(False, 'All motors stop at once and the drone FALLS from where it is. Only for a drone '
                           'that is caught in something or flying away. Normal way down: LAND. Cut the motors?',
                           'warn', need_confirm='uav_emergency')
        link.emergency()
        self._uav_code = None
        self._uav_say('EMERGENCY: motors cut.', 'error', log=False)
        return _result(True, self.uav_note, 'warn')

    def cmd_uav_keys(self, args):
        """The keys held on the page right now. Called several times a second while a key is down; never waits."""
        keys = args.get('keys')
        if not isinstance(keys, list) or len(keys) > len(KEYS) or any(k not in KEYS for k in keys):
            raise ValueError('Unknown key.')
        stamp = args.get('t')
        if stamp is not None:                      # reports can overtake each other: only the newest one counts
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
                raise ValueError('Bad key report.')
            last = self._uav_key_t
            if last is not None and stamp <= last and last - stamp < 10000:
                return _result(True, stale=True)
            self._uav_key_t = stamp
        link = self.uav
        if link is None:
            return _result(True)
        if self.opts['section'] != 'uav':
            keys = []
        link.set_keys(keys, self.opts['uav_speed'])
        return _result(True)

    # ------------------------------------------------------------------ footage
    def _uav_live(self):
        video = self.uav_video
        if video is None or video.dead:
            return False
        age = video.status().get('frame_age')
        return age is not None and age < 2.0

    def _uav_record_start(self, auto):
        video = self.uav_video
        if video is None or video.dead:
            self._uav_say('The flight is NOT filmed: there is no video from the drone.', 'warn')
            return False
        try:
            folder = check_folder(self._uav_folder())
        except RecorderError as exc:
            self._uav_say('The flight is NOT filmed: %s.' % exc, 'warn')
            return False
        video.record_start(folder, 'sim' if self.uav_sim is not None else '')
        self.uav_rec = dict(auto=bool(auto), started=time.monotonic(), stop_at=None)
        self._bump('uav')
        return True

    def _uav_record_stop(self, reason, tail=0.0):
        rec, video = self.uav_rec, self.uav_video
        if rec is None or video is None:
            return
        if tail > 0:
            if rec['stop_at'] is None:
                rec['stop_at'], rec['reason'] = time.monotonic() + tail, reason
        else:
            video.record_stop(reason)
            rec['stop_at'], rec['sent'] = time.monotonic(), True
        self._bump('uav')

    def cmd_uav_record(self, args):
        on = arg_bool(args, 'on')
        with self._lock:
            if self.uav is None:
                return self._fail('Connect the drone first.')
            if on:
                if self.uav_rec is not None:
                    return self._ok('Already recording.')
                if not self._uav_live():
                    return self._fail('There is no live video from the drone to record.')
                if not self._uav_record_start(auto=False):
                    return _result(False, self.uav_note, 'warn')
                return self._ok('Recording the drone camera. Press Stop recording (or land) to save it.')
            if self.uav_rec is None:
                return self._ok('Nothing is being recorded.')
            self._uav_record_stop('stopped by the operator')
            return self._ok('Saving the footage…')

    def cmd_uav_photo(self, args):
        with self._lock:
            video = self.uav_video
            if video is None or not self._uav_live():
                return self._fail('There is no live picture from the drone.')
            try:
                folder = check_folder(self._uav_folder())
            except RecorderError as exc:
                return self._fail('Cannot save a photo: %s.' % exc)
            video.photo(folder)
            return _result(True)

    def cmd_uav_open_folder(self, args):
        try:
            folder = check_folder(self._uav_folder())
            self._open_folder(folder)
        except RecorderError as exc:
            return self._fail('Cannot open the footage folder: %s.' % exc)
        except OSError as exc:
            return self._fail('Could not open the folder: %s' % exc)
        return self._ok('Opened %s.' % folder)

    def _uav_video_events(self, video):
        for event in video.take_events():
            kind = event[0]
            if kind == 'recorded':
                result, fault = event[1], event[2]
                self.uav_rec = None
                if result is not None:
                    self.uav_last = result.as_dict()
                    self.uav_made += 1
                    seconds = int(round(result.seconds))
                    self._uav_say('Drone footage saved: %s (%d:%02d, %.1f MB).'
                                  % (result.path, seconds // 60, seconds % 60, result.size_bytes / 1024 ** 2), 'success')
                elif not fault:
                    fault = 'no picture arrived from the drone, so nothing was saved'
                if fault:
                    self._uav_say('Drone footage: %s.' % fault, 'warn')
            elif kind == 'record_failed':
                self.uav_rec = None
                self._uav_say('The flight is NOT filmed: %s.' % event[1], 'warn')
            elif kind == 'photo':
                if event[1]:
                    self.uav_last_photo = dict(path=event[1], name=Path(event[1]).name)
                    self.uav_photos += 1
                    self._uav_say('Photo saved: %s' % event[1], 'success')
                else:
                    self._uav_say('No photo: %s.' % event[2], 'warn')
            elif kind == 'gone':
                self.uav_rec = None
                self.uav_video_note = 'The video worker stopped unexpectedly: reconnect the drone to get the picture back.'
                self._uav_say(self.uav_video_note, 'error')
            self._bump('uav')

    # ------------------------------------------------------------------ housekeeping (from the ticker)
    def _uav_tick(self):
        link = self.uav
        if link is None:
            return
        for level, text in link.take_events():
            self._uav_say(text, level)
        state = link.state
        if state != self._uav_was:
            was, self._uav_was = self._uav_was, state
            if state == READY and was in AIRBORNE:                 # down again: save the footage of this flight
                rec = self.uav_rec
                if rec is not None and rec['auto']:
                    self._uav_record_stop('landed', tail=RECORD_TAIL_S)
            if self.uav_video is not None:
                self.uav_video.set_airborne(state in AIRBORNE)
            self._bump('uav')
        video, rec = self.uav_video, self.uav_rec
        if video is None:
            return
        self._uav_video_events(video)
        now = time.monotonic()
        if rec is not None and rec.get('stop_at') is not None and not rec.get('sent') and now >= rec['stop_at']:
            rec['sent'] = True
            video.record_stop(rec.get('reason', 'stopped'))
        # datagrams arrive but no picture comes out of them: ask the drone to start its video again
        stream = self._uav_stream
        status = video.status()
        if status.get('frame_age') is not None and status['frame_age'] < 2.0:
            stream['since'], stream['restarts'] = now, 0
        elif (not stream['busy'] and stream['restarts'] < 2 and stream['since'] is not None
              and now - stream['since'] > 5.0 and state in (READY, FLYING) and not video.dead):
            stream['busy'], stream['since'] = True, now
            stream['restarts'] += 1
            self._log_line('warn', 'No picture from the drone yet: asking it to start its video again.')
            threading.Thread(target=self._uav_restart_stream, args=(link, stream), name='uav-stream', daemon=True).start()

    @staticmethod
    def _uav_restart_stream(link, stream):
        try:
            link.stream(False)
            link.stream(True)
        finally:
            stream['busy'] = False

    # ------------------------------------------------------------------ what the page shows
    def _view_uav(self):
        o = self.opts
        link, video = self.uav, self.uav_video
        state = 'connecting' if self.uav_connecting else DISCONNECTED
        data = dict(section=o['section'], sim=bool(o['uav_sim']), ip=o['uav_ip'], speed=int(o['uav_speed']),
                    state=state, connected=False, airborne=False, simulated=self.uav_sim is not None,
                    battery=None, height_cm=None, tof_cm=None, flight_s=None, temp_c=None, yaw=None,
                    source='none', keys=[], sticks=[0, 0, 0, 0], warnings=[], note=self.uav_note,
                    note_level=self.uav_note_level, takeoff_hint='Connect the drone first.',
                    video=dict(on=False, live=False, w=0, h=0, fps=0.0, kbps=0, note=self.uav_video_note),
                    rec=dict(active=False, seconds=0, mb=0.0, auto=False, saving=False),
                    record_on=bool(o['uav_record']), folder=str(self._uav_folder()), last=self.uav_last,
                    made=self.uav_made, photo=self.uav_last_photo, photos=self.uav_photos, low_battery=LOW_BATTERY)
        if link is None:
            data['state_text'] = STATE_TEXT[state]
            return self.clean(data)
        snap = link.snapshot()
        state = snap['state']
        whole = lambda v: None if v is None else int(round(v))             # noqa: E731
        warnings = []
        battery, height = whole(snap['battery']), whole(snap['height_cm'])
        if snap['silent']:
            warnings.append(dict(level='bad', text='Nothing is heard from the drone. Out of range, or switched off?'))
        if battery is not None and battery < link.cfg.min_battery:
            warnings.append(dict(level='bad', text='Battery %d %%: %s' % (
                battery, 'LAND NOW.' if state in AIRBORNE else 'too low to take off.')))
        elif battery is not None and battery < LOW_BATTERY:
            warnings.append(dict(level='warn', text='Battery %d %%: land soon.' % battery))
        if snap['status_port_error']:
            warnings.append(dict(level='warn', text=snap['status_port_error']))
        elif snap['status_blocked']:
            warnings.append(dict(level='warn', text='The drone answers, but its status messages do not arrive. '
                                                    + FIREWALL_HINT))
        if state == FLYING and height is not None and height <= 0 and snap['since'] > 6.0:
            warnings.append(dict(level='warn', text='The altitude reads 0: if the drone has landed by itself, press '
                                                    'LAND to tell the console.'))
        if state == READY and height is not None and height >= 30:
            warnings.append(dict(level='warn', text='The drone reports %d cm although the console did not start a '
                                                    'flight: if it is in the air, press LAND.' % height))
        status = video.status() if video is not None else {}
        frame_age = status.get('frame_age')
        live = video is not None and not video.dead and frame_age is not None and frame_age < 2.0
        video_note = self.uav_video_note
        if video is not None and not live and not video_note and snap['since'] > 4.0:
            packet_age = status.get('packet_age')
            video_note = ('Video data arrives but no picture can be made from it yet.' if packet_age is not None
                          and packet_age < 2.0 else 'No video arrives from the drone. ' + FIREWALL_HINT)
        rec, recording = self.uav_rec, status.get('recording') or {}
        problem = self._uav_takeoff_problem()
        data.update(
            state=state, state_text=STATE_TEXT.get(state, state), connected=True, airborne=state in AIRBORNE,
            battery=battery, height_cm=height, tof_cm=whole(snap['tof_cm']), flight_s=whole(snap['flight_s']),
            temp_c=whole(snap['temp_c']), yaw=whole(snap['yaw']), source=snap['source'], keys=snap['keys'],
            sticks=snap['sticks'], warnings=warnings, takeoff_hint=problem,
            video=dict(on=video is not None and not video.dead, live=live, w=status.get('width', 0),
                       h=status.get('height', 0), fps=status.get('fps', 0.0), kbps=status.get('kbps', 0),
                       note=video_note),
            rec=dict(active=rec is not None, seconds=int(recording.get('seconds', 0.0)),
                     mb=round(recording.get('bytes', 0) / 1024 ** 2, 1), auto=bool(rec and rec['auto']),
                     saving=bool(rec and rec.get('sent'))))
        return self.clean(data)
