"""The operator console without any GUI toolkit.

``ConsoleApp`` owns everything the old Tk console owned (map, planner settings, the connection to
the simulated or real UGV, navigator, survivor search, camera/YOLO worker, calibration runs, saved
settings) and exposes it as validated commands (``command(name, args)``) plus state "views" that a
display can poll or stream (``views.py``). The web console (``server.py`` + ``web/``) is one such
display; tests drive the same commands directly.

Rules kept from the original console:

* Nothing moves without an operator authorization. Authorization is a two-step handshake
  (``authorize_prepare`` -> summary + one-time code, ``authorize_confirm`` -> go) so a single stray
  request can never start motion.
* STOP never waits for anything.
* Original map files are never written; edited maps go to ``data/edited`` (with backups).
"""
from __future__ import annotations

import collections
import csv
import dataclasses
import hmac
import math
import os
import secrets
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

from ugv_bridge.protocol import load_key
from ugv_planner.config import EXAMPLES, Footprint, PlannerConfig
from ugv_planner.evaluation import evaluate, simulation_preflight
from ugv_planner.grid import Grid
from ugv_planner.simulation import SimulatedUGV

from ugv_nav.config import DEFAULT_CONFIG, load_config
from ugv_nav.edge_client import EdgeClient
from ugv_nav.fast_planner import RoutePlanner
from ugv_nav.localization import compute_axis_scale, validate_scale
from ugv_nav.map_store import (CORNER_NAMES, CORNERS, EDIT_DIR, _atomic_save_npy, corner_pose, edited_path,
                               example_points, inner_box, load_edited, load_env, paint, revert_to_original,
                               save_edited, validate_field_scale)
from ugv_nav.mission import MissionConfig, SearchMission
from ugv_nav.navigator import PLANNED, RUNNING, Navigator, NavigatorConfig
from ugv_nav.path_processing import path_length, remove_collinear, validate_path
from ugv_nav.recorder import MissionRecorder, RecorderError, check_folder, default_folder
from ugv_nav.safety_monitor import SafetyConfig
from ugv_nav.settings_store import load_settings, save_settings
from ugv_nav.sim_backend import simulated_room
from ugv_nav.sim_session import SimulatedUGVSession
from ugv_nav.smoothing import smooth_route
from uav.tello import check_host

from . import options as O
from .options import OFFLINE, SIM, arg_bool, arg_choice, arg_number, arg_points
from .uav_console import UavConsole
from .views import Views

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS = ROOT / 'config' / 'console_settings.json'
OLD_SETTINGS = ROOT / 'config' / 'gui_settings.json'          # previous console: imported once if present
EXPORT_DIR = ROOT / 'exports'
SIM_HALF_L, SIM_HALF_W = 0.08, 0.07       # fallback only if config/navigation.toml has no robot dimensions
SIM_TRUE_CAMERA = dict(pitch_rad=math.radians(15.0), x_offset_m=0.08)   # mounting of the SIMULATED camera
VERSION = '2.0'


def result(ok, message='', level=None, **extra):
    return dict(ok=bool(ok), message=message, level=level or ('info' if ok else 'error'), **extra)


class ConsoleApp(UavConsole, Views):
    VERSION = VERSION

    def __init__(self, config_path=DEFAULT_CONFIG, settings_path=DEFAULT_SETTINGS, edit_dir=EDIT_DIR,
                 export_dir=EXPORT_DIR, start_ticker=True):
        self._lock = threading.RLock()
        self.settings_path = settings_path        # None = do not remember settings (tests)
        self.edit_dir = Path(edit_dir)
        self.export_dir = Path(export_dir)
        self.opts = O.defaults()
        self.config_error = ''
        try:
            self.edge_cfg = load_config(config_path)
        except Exception as exc:                  # console still usable for planning / simulator
            self.edge_cfg = None
            self.config_error = str(exc)
        self.planner = RoutePlanner('fast')
        self.sim = SimulatedUGV()
        self.nav = self.mission = self.session = self.client = self.vision = None
        self.sim_blocks = None
        self.cam_samples = []
        self.cal = None                            # active scale-calibration run
        self.pending_pose = None
        self.dock = None                           # corner start waiting for Confirm pose: dict(corner, pose)
        self.room_dir = self.edit_dir.parent / 'rooms'     # what the LiDAR learnt about the room (per environment)
        self.field_scale = {}                      # environment -> (x, y): real field size / size in the data file
        self.field_samples = []                    # corners measured with the robot (cmd_field_sample)
        self.field_measured = None                 # real inner size (x, y) from those corners; None = not measured
        self.lidar_mount, self.lidar_mount_ok = (0.0, 0.0, 0.0), False     # learnt on the REAL robot
        self._room_saved = None
        self._room_check = 0.0
        self._found_seen = 0
        self.recorder = None                       # MissionRecorder while a mission is being filmed
        self.last_recording = None                 # dict about the newest saved video (shown on the page)
        self.recording_note = ''                   # why the last mission was not filmed / the video stopped early
        self.recordings_made = 0
        self.current_env = None
        self.map_source, self.map_unsaved, self.edited = 'original', False, False
        self.notice = ''
        self.activity = collections.deque(maxlen=300)   # dict(id, t, level, text)
        self._activity_lock = threading.Lock()          # written from several threads (commands, vision, ticker)
        self._activity_id = 0
        self._nav_log_seen = 0
        self._auth = None
        self._settings_dirty = False
        self._camera_timer = None
        self._closing = threading.Event()
        self.revs = collections.Counter()          # bumped to tell the display that something changed
        self._view_cache = {}
        self.result = self.report = self.active_config = None
        self.dispatch_allowed, self.dispatch_message = False, ''
        self._gt_guard = None
        self._stroke_dirty = False
        self._uav_init()                           # the UAV section (uav_console.py)
        self._restore_settings()
        self._ticker = None
        if start_ticker:
            self._ticker = threading.Thread(target=self._tick_loop, name='console-ticker', daemon=True)
            self._ticker.start()

    # ------------------------------------------------------------------ small helpers
    def _say(self, text, level='info', log=True):
        self.notice = text
        if log:
            self._log_line(level, text)

    def _log_line(self, level, text):
        with self._activity_lock:
            self._activity_id += 1
            self.activity.append(dict(id=self._activity_id, t=time.strftime('%H:%M:%S'), level=level, text=text))
            self.revs['log'] += 1

    def log_lines(self, count=150):
        with self._activity_lock:
            return list(self.activity)[-count:]

    def _ok(self, text='', level='info', **extra):
        if text:
            self._say(text, level)
        return result(True, text, level, **extra)

    def _fail(self, text, level='warn', **extra):
        self._say(text, level, log=level == 'error')
        return result(False, text, level, **extra)

    def _bump(self, *names):
        for name in names:
            self.revs[name] += 1

    def nav_active(self):
        return self.opts['mode'] != OFFLINE

    def _safety_config(self):
        cfg = self.edge_cfg.navigator.safety if self.edge_cfg else SafetyConfig()
        if self.opts['mode'] == SIM and not (cfg.half_length_m and cfg.half_width_m):
            cfg = dataclasses.replace(cfg, half_length_m=SIM_HALF_L, half_width_m=SIM_HALF_W)
        return cfg

    def robot_half_size(self):
        cfg = self._safety_config()
        return (cfg.half_length_m or 0.11), (cfg.half_width_m or 0.09)

    def planner_config(self, record_search=False):
        """PlannerConfig from the console fields (raises ValueError with the planner's own message)."""
        o = self.opts
        cfg = PlannerConfig(int(o['connectivity']), Footprint(o['radius'], float(o['margin']), bool(o['verified'])),
                            bool(o['inflation']), record_search, float(o['clearance_weight']),
                            float(o['extra_clearance']))
        if o['mode'] == SIM and cfg.footprint.radius_m is None:
            safety = self._safety_config()
            half_l, half_w = safety.half_length_m, safety.half_width_m
            radius = math.ceil(math.hypot(half_l, half_w) * 1000) / 1000          # round UP
            margin = max(cfg.footprint.margin_m, safety.rotate_clearance_m)
            cfg = dataclasses.replace(cfg, footprint=Footprint(radius, margin, True))
            o['radius'], o['margin'], o['verified'] = radius, margin, True
            self._bump('settings')
        return cfg

    def _full_speed(self):
        base = self.nav.base_controller if self.nav else (
            self.edge_cfg.navigator.controller if self.edge_cfg else NavigatorConfig().controller)
        return base.max_v, base.max_w

    def _mission_config(self):
        o = self.opts
        return MissionConfig(approach_gap_m=o['mission_gap_cm'] / 100.0, secure_hold_s=o['mission_hold_s'],
                             end_when_secured=o['mission_end'], same_survivor_m=o['mission_same_cm'] / 100.0,
                             scan_step_deg=o['mission_step_deg'], scan_dwell_s=o['mission_dwell_s'],
                             scan_turn_rate=o['mission_turn_rate'], view_range_m=o['mission_range_m'])

    # ------------------------------------------------------------------ settings
    def _restore_settings(self):
        saved = load_settings(self.settings_path)
        first_run = not saved
        if first_run and self.settings_path is not None and Path(self.settings_path).name == DEFAULT_SETTINGS.name:
            old = load_settings(Path(self.settings_path).with_name(OLD_SETTINGS.name))
            if old:                                # settings of the previous console, copied next to ours
                saved, first_run = old, False
        if self.edge_cfg is not None:
            net, nav = self.edge_cfg.network, self.edge_cfg.navigator
            self.opts.update(host=net.ugv_host, port=int(net.ugv_port),
                             auto_continue=bool(nav.auto_continue_after_replan), smooth_path=bool(nav.smooth_path),
                             lidar_correction=bool(nav.lidar_correction), auto_find=bool(nav.auto_find),
                             ignore_drift=bool(nav.localization.ignore_drift),
                             lidar_sees='room' if nav.lidar_reference == 'room' else 'walls',
                             scale_x=float(nav.localization.scale_x), scale_y=float(nav.localization.scale_y))
            if first_run:                          # start from the robot values in config/navigation.toml
                fp = self.edge_cfg.planner.footprint
                self.opts.update(radius=fp.radius_m, margin=float(fp.margin_m), verified=fp.radius_m is not None)
        self.opts.update(O.from_saved(saved))
        scales = saved.get('field_scale')
        for env, value in (scales.items() if isinstance(scales, dict) else []):
            try:
                if env in O.SCHEMA['env'].choices:
                    self.field_scale[env] = validate_field_scale(value)
            except (TypeError, ValueError):
                pass
        mount = saved.get('lidar_mount')
        if (isinstance(mount, list) and len(mount) == 3
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in mount)
                and abs(mount[0]) <= 0.3 and abs(mount[1]) <= 0.3):
            self.lidar_mount = tuple(float(v) for v in mount)
            self.lidar_mount_ok = saved.get('lidar_mount_ok') is True
        with self._lock:
            self._load_env()
        self._settings_dirty = True

    def _save_settings(self):
        self._settings_dirty = False
        try:
            save_settings(self.settings_path, dict(self.opts, field_scale={k: list(v) for k, v in self.field_scale.items()},
                                                   lidar_mount=list(self.lidar_mount),
                                                   lidar_mount_ok=bool(self.lidar_mount_ok)))
        except OSError as exc:
            self._say('Could not save settings: %s' % exc, 'warn')

    def cmd_set_options(self, args):
        values = args.get('values')
        if not isinstance(values, dict) or not values or len(values) > 40:
            raise ValueError('No settings given.')
        clean = {}
        for key, value in values.items():
            if key in ('mode', 'env', 'scale_x', 'scale_y', 'section'):
                raise ValueError('"%s" has its own command.' % key)
            clean[key] = O.coerce(key, value)
        with self._lock:
            if clean.get('allow_tight') and not self.opts['allow_tight'] and not args.get('confirm'):
                sc = self._safety_config()
                turn = (' (turning circle %.3f m)' % (math.hypot(sc.half_length_m, sc.half_width_m)
                                                      + sc.rotate_clearance_m)
                        if sc.half_length_m and sc.half_width_m else '')
                return result(False, 'A* may route the robot through gaps where it can drive but cannot turn on '
                              'the spot%s. The robot will not crash because of this: the UGV refuses any turn with '
                              'a wall inside its turning circle, so it STOPS at such places instead. Allow it?'
                              % turn, 'warn', need_confirm='allow_tight')
            if clean.get('ignore_drift') and not self.opts['ignore_drift'] and not args.get('confirm'):
                limit = (self.nav.estimator.cfg.max_sigma_m if self.nav else
                         self.edge_cfg.navigator.localization.max_sigma_m if self.edge_cfg else 0.15)
                return result(False, 'The robot normally stops when its estimated position error passes %.0f cm. '
                              'Without that limit it keeps driving on its own estimate however far it is from '
                              'reality, so it can end up where the picture says it is not, for example next to a '
                              'wall. STOP, the LiDAR stop zone, link loss and stale odometry still work. Watch the '
                              'robot and keep a hand on STOP. Ignore drift alerts?' % (100 * limit),
                              'warn', need_confirm='ignore_drift')
            before = dict(self.opts)
            self.opts.update(clean)
            try:
                message = self._options_changed(set(clean), before)
            except ValueError as exc:              # e.g. a planner setting the planner itself rejects
                self.opts.clear(); self.opts.update(before)
                self._bump('settings')
                raise ValueError('Not applied: %s' % exc)
            self._bump('settings')
            self._settings_dirty = True
            return self._ok(message) if message else result(True)

    def _options_changed(self, keys, before):
        o, message = self.opts, ''
        if keys & set(O.PLANNER_KEYS):
            self.planner_config()                  # validate first (raises ValueError)
            if 'smooth_path' in keys and self.nav:
                self.nav.cfg.smooth_path = o['smooth_path']
            self._planner_changed()
            message = self.notice
        if 'auto_continue' in keys:
            if self.nav:
                self.nav.cfg.auto_continue_after_replan = o['auto_continue']
                if not o['auto_continue']:
                    self.nav._cancel_auto()
            message = 'Auto-continue after replan %s.' % (
                'ON: the robot drives new routes around unmapped obstacles by itself (STOP and faults still need '
                'authorization)' if o['auto_continue'] else 'OFF')
        if 'allow_tight' in keys:
            if self.nav:
                self.nav.cfg.allow_tight_radius = o['allow_tight']
            message = 'Radius below turning circle %s.' % ('ALLOWED' if o['allow_tight'] else 'not allowed')
        if 'record_dir' in keys:
            try:
                folder = check_folder(self._recording_folder())             # created and proven writable now
            except RecorderError as exc:
                raise ValueError('the video folder: %s' % exc)               # the setting goes back to what it was
            message = 'Mission videos are saved in %s.' % folder
        if 'record_overlay' in keys:
            message = ('The next video shows the time, mission phase and detections on the picture.' if o['record_overlay']
                       else 'The next video is the plain camera picture.')
        if 'record_mission' in keys:
            if o['record_mission']:
                if self.mission is not None and self.mission.active and self.recorder is None:
                    self._record_start()                                     # switched on during a mission: film from now
                message = 'Missions are filmed: the video is saved when the mission ends.'
            else:
                self._record_stop('video switched off', wait=True)           # what was filmed so far is saved
                message = 'Missions are no longer filmed.'
            self._bump('recording')
        if keys & {'uav_sim', 'uav_ip'}:
            if self.uav is not None or self.uav_connecting:
                raise ValueError('disconnect the drone first')
            o['uav_ip'] = check_host(o['uav_ip'])                            # ValueError: the setting goes back
            message = ('Simulated drone: a practice drone on this laptop, nothing real moves.' if o['uav_sim'] else
                       'Real drone at %s. Join its Wi-Fi (TELLO-...) before you connect.' % o['uav_ip'])
            self._bump('uav')
        if 'uav_dir' in keys:
            try:
                folder = check_folder(self._uav_folder())
            except RecorderError as exc:
                raise ValueError('the footage folder: %s' % exc)
            message = 'Drone footage is saved in %s.' % folder
            self._bump('uav')
        if 'uav_record' in keys:
            message = ('Every flight is filmed from take-off to landing.' if o['uav_record'] else
                       'Flights are no longer filmed by themselves (the Record button still works).')
            self._bump('uav')
        if 'uav_speed' in keys:
            message = 'Stick strength %d %%: how hard a held key pushes the drone.' % o['uav_speed']
            self._bump('uav')
        if 'ignore_drift' in keys:
            if self.nav:
                self.nav.set_ignore_drift(o['ignore_drift'])
            message = ('Drift alerts IGNORED: the robot no longer stops for "drift too high". STOP, the LiDAR stop '
                       'zone and link loss still work.' if o['ignore_drift'] else
                       'Drift alerts ON again: the robot stops when the estimated drift passes its limit.')
        if 'lidar_correction' in keys:
            if self.nav:
                self.nav.cfg.lidar_correction = o['lidar_correction']
            message = ('Live LiDAR tracking ON: the position follows what the LiDAR sees.'
                       if o['lidar_correction'] else 'Live LiDAR tracking OFF: the position comes from odometry only.')
        if 'auto_find' in keys and self.nav:
            self.nav.cfg.auto_find = o['auto_find']
        if 'lidar_sees' in keys and o['lidar_sees'] != before['lidar_sees']:
            was = self.nav is not None
            self._disconnect()
            self._bump('map', 'plan')
            message = ('The LiDAR looks over the arena walls: it sees the room, and the console follows the robot in '
                       'that room.' if o['lidar_sees'] == 'room' else
                       'The LiDAR sees the arena walls: the scan is compared with the map.')
            message += ' Connect again.' if was else ''
        if 'corner_gap_cm' in keys and self.dock:
            self._corner_start(self.dock['corner'], turn=False)
            message = self.notice
        if 'heading' in keys:
            self.dock = None
        if 'robot_speed' in keys:
            message = self._apply_speed()
        if keys & set(O.MISSION_KEYS) and self.mission is not None:
            self.mission.cfg = self._mission_config()
            message = 'Search settings applied.'
        if 'search_at_goal' in keys:
            message = ('Search ON: after reaching the goal the robot looks for green objects (survivors) on its own '
                       ': it drives to the first one it sees, stands in front of it and marks it secured. Camera + YOLO must be on.' if o['search_at_goal']
                       else 'Search OFF: the robot stops at the goal.')
        if 'camera_on' in keys and o['camera_on'] != before['camera_on']:
            message = self._start_camera() if o['camera_on'] else self._stop_camera()
        if self.vision is not None:
            self.vision.auto_tilt, self.vision.shape_filter = o['auto_tilt'], o['shape_filter']
            self.vision.colour_check = o['colour_check']
            self.vision.tracker.cfg.min_confidence = o['yolo_min_conf'] / 100.0
        if keys & set(O.CAMERA_MOUNT_KEYS):
            cam = self._camera_model()
            if self.vision:
                self.vision.set_camera(cam)
            message = ('Camera mount applied: height %.1f cm, tilt %.1f deg, %.1f cm ahead, yaw %.1f deg.'
                       % (cam.height_m * 100, math.degrees(cam.pitch_rad), cam.x_offset_m * 100,
                          math.degrees(cam.yaw_offset_rad)))
        if 'heading' in keys:
            self._bump('plan')
        return message

    def _planner_changed(self):
        """Footprint / clearance / smoothing changed: replan (offline) or hand the settings to the navigator."""
        self._bump('map', 'plan')
        if not self.nav_active():
            self._recalculate()
            return
        nav = self.nav
        if nav is not None and nav.state != RUNNING:
            nav.set_planner_config(self.planner_config())
            if nav.goal is not None and nav.estimator.pose() is not None:
                nav.set_goal(nav.goal)
                self._say(nav.message, log=False)

    def _apply_speed(self):
        pct = int(self.opts['robot_speed'])
        full_v, full_w = self._full_speed()
        if self.nav:
            self.nav.set_speed_fraction(pct / 100.0)
        return 'Robot speed %d %% = %.3f m/s, turning %.2f rad/s%s.' % (
            pct, full_v * pct / 100.0, full_w * pct / 100.0, '' if self.nav else ' (applies when connected)')

    # ------------------------------------------------------------------ mode / environment / map
    def cmd_set_mode(self, args):
        mode = arg_choice(args, 'mode', O.MODES)
        with self._lock:
            if self.opts['section'] == 'uav':          # a mode button in the top bar also leaves the UAV section
                if self.uav_airborne():
                    return self._fail('The drone is in the air: land it before you leave the UAV section.')
                self.opts['section'] = 'ugv'
                self._bump('settings', 'uav')
                self._settings_dirty = True
                if mode == self.opts['mode']:          # back to where the UGV was: nothing is disconnected
                    return self._ok('%s mode.' % O.MODE_LABELS[mode])
            self._disconnect()
            self.sim.pause()
            self.opts['mode'] = mode
            self.pending_pose = None
            self._bump('settings', 'map', 'plan')
            self._settings_dirty = True
            if mode == OFFLINE:
                g = self.grid
                self.start_point = g.cell_to_world(g.world_to_cell(self.start_point))
                self._recalculate(use_current=False)
                return self._ok('Planner mode: plan and play back A* routes on the map (no robot).')
            return self._ok('%s mode: press %s, place the robot (Start + heading), Confirm pose, then set a goal.'
                            % (O.MODE_LABELS[mode], 'Start simulator' if mode == SIM else 'Connect'))

    def cmd_set_env(self, args):
        env = arg_choice(args, 'env', tuple(EXAMPLES))
        with self._lock:
            if self.map_unsaved and not arg_bool(args, 'confirm'):
                return result(False, 'Discard the unsaved map edits for %s?' % self.current_env, 'warn',
                              need_confirm='discard_edits')
            if self.session is not None and env != self.current_env:
                self._disconnect()                 # the simulated world belongs to the old environment
                note = ' Simulator stopped: press Start simulator again.'
            else:
                note = ''
            self.opts['env'] = env
            self._settings_dirty = True
            self._load_env()
            return self._ok('%s loaded (%s map).%s' % (env, 'your saved edited' if self.map_source == 'saved'
                                                      else 'original', note))

    def _scale(self, env=None):
        return tuple(self.field_scale.get(env or self.opts['env'], (1.0, 1.0)))

    def _load_env(self):
        env = self.opts['env']
        scale = self._scale(env)
        grid, gt = load_env(env, scale)
        self.grid, self.gt = grid, gt
        self.edited = False
        self.start_point = grid.cell_to_world(grid.world_to_cell(example_points(env, scale)[0]))
        self.goal_point = None                     # the operator chooses the goal: never preset
        self.pending_pose = None
        self.dock = None
        self.field_samples, self.field_measured = [], None
        self.sim.set_route([])
        self.map_unsaved, self.map_source = False, 'original'
        try:
            saved = load_edited(env, self.edit_dir, scale)
        except (ValueError, OSError) as exc:
            saved = None
            self._say('Saved map for %s could not be loaded (%s); using the original.' % (env, exc), 'warn')
        if saved is not None:
            self.grid, self.map_source = saved, 'saved'
        env_changed = self.current_env is not None and env != self.current_env
        self.current_env = env
        if self.nav:
            if env_changed:                        # camera pins belong to the previous environment
                if self.vision:
                    self.vision.clear_obstacles()
                self.nav.set_vision_obstacles([], reason='environment changed')
            self._push_map_to_nav(keep_lidar=False)
        self.result = None
        if not self.nav_active():
            self._recalculate(use_current=False)
        self._bump('map', 'plan', 'settings')

    def _push_map_to_nav(self, replan=True, keep_lidar=True):
        """Give the operator's map to the navigator (the single path for every map change while connected)."""
        nav = self.nav
        if not nav:
            return
        goal = nav.goal
        nav.set_map(self.grid, keep_lidar=keep_lidar)
        if replan and goal is not None and nav.state != RUNNING and nav.estimator.pose() is not None:
            nav.set_goal(goal)                     # keep the route: replan on the changed map

    def cmd_paint(self, args):
        """Brush stroke (or part of one): points in metres, brush radius in cells, value 1 = obstacle, 0 = free."""
        points = arg_points(args, 'points')
        brush = int(arg_number(args, 'brush', 0, 10, self.opts['brush']))
        value = bool(int(arg_number(args, 'value', 0, 1)))
        end = arg_bool(args, 'end')
        with self._lock:
            if self.nav and self.nav.state == RUNNING:
                return self._fail('Stop the robot before editing the map.')
            grid = self.grid
            for point in points:
                grid = paint(grid, point, brush, value)
            if grid is not self.grid:
                self.grid = grid
                self.edited = self.map_unsaved = True
                self._stroke_dirty = True
                self._bump('map')
            if end and self._stroke_dirty:
                self._stroke_dirty = False
                return self._map_changed()
            return result(True)

    def _map_changed(self):
        self._bump('map', 'plan')
        if self.nav:
            self._push_map_to_nav()
            return self._ok('Map updated for the edge planner%s.' % ('; route replanned' if self.nav.path else ''))
        if not self.nav_active():
            self._recalculate()
            return result(True, self.notice)
        return self._ok('Map edited (unsaved).')

    def cmd_save_map(self, args):
        with self._lock:
            try:
                path = save_edited(self.opts['env'], self.grid, self.edit_dir, self._scale())
            except (OSError, ValueError) as exc:
                return self._fail('Save failed: %s' % exc, 'error')
            self.map_source, self.map_unsaved = 'saved', False
            self._bump('map')
            return self._ok('Map saved to %s. It loads automatically next time; data/%s_occupancy.npy is unchanged.'
                            % (path, self.opts['env']), 'success')

    def cmd_revert_map(self, args):
        with self._lock:
            env = self.opts['env']
            has_saved = edited_path(env, self.edit_dir).exists()
            if not has_saved and not self.map_unsaved:
                return self._ok('%s already uses the original map.' % env)
            if not arg_bool(args, 'confirm'):
                return result(False, 'Use the original %s map again? Your saved edits are kept as a backup file in '
                              'data/edited/.' % env, 'warn', need_confirm='revert_map')
            backup = revert_to_original(env, self.edit_dir)
            self.map_unsaved = False
            self._load_env()
            return self._ok('Original %s map restored.%s' % (env, ' Previous edits backed up to %s.' % backup
                                                              if backup else ''), 'success')

    def cmd_lidar_layer(self, args):
        """Obstacles the LiDAR added during a run: 'keep' = make them part of the map, 'clear' = forget them."""
        action = arg_choice(args, 'action', ('keep', 'clear'))
        with self._lock:
            nav = self.nav
            if not nav:
                return self._fail('Connect first.')
            if nav.state == RUNNING:
                return self._fail('Stop the robot first.')
            count = int(nav.lidar_mask.sum())
            if not count:
                return self._ok('No LiDAR-detected obstacles on the map.')
            if action == 'clear':
                nav.clear_lidar_obstacles()
                self._bump('map', 'plan')
                return self._ok('%d LiDAR-detected cells removed.' % count)
            b = self.grid
            self.grid = Grid((b.cells | nav.lidar_mask).astype(np.uint8), b.resolution, b.origin)
            self.edited = self.map_unsaved = True
            self._push_map_to_nav(keep_lidar=False)
            self._bump('map', 'plan')
            return self._ok('%d LiDAR-detected cells added to the map (unsaved: press Save map to keep them).' % count)

    # ------------------------------------------------------------------ planner mode (offline)
    def _recalculate(self, use_current=True):
        """Plan with A* on the current map (planner mode) and evaluate against GT, like the original GUI."""
        running = self.sim.state == 'running'
        self.sim.pause()
        self._bump('plan', 'offline')
        try:
            config = self.planner_config(record_search=bool(self.opts['show_search']))
            if use_current and self.sim.position is not None:
                self.start_point = self.sim.position
            if self.goal_point is None:
                self.result = self.report = None
                self.dispatch_allowed, self.dispatch_message = False, 'Set a goal first: Goal tool, then click the map.'
                self.sim.set_route([])
                self._gt_guard = None
                self._say('No goal yet: choose the Goal tool and click the map.', log=False)
                return
            began = time.perf_counter()
            res = self.planner.replan(self.grid, self.start_point, self.goal_point, config)
            raw_length, raw_points, smoothed = res.distance_m, len(remove_collinear(res.waypoints)), False
            if res.found and self.opts['smooth_path']:
                points, info = smooth_route(remove_collinear(res.waypoints), self.grid, config, self.planner.cache)
                if info['smoothed']:
                    smoothed = True
                    res.waypoints = [tuple(p) for p in points]
                    res.distance_m = path_length(points)
                    res.cost_cells = res.distance_m / self.grid.resolution
            res.computation_ms = (time.perf_counter() - began) * 1000
            self.active_config, self.result = config, res
            report = evaluate(res, self.gt, config.footprint)
            self.dispatch_allowed, self.dispatch_message = simulation_preflight(res, self.grid, report,
                                                                               config.footprint)
            report.update(dispatch_allowed=self.dispatch_allowed, dispatch_message=self.dispatch_message,
                          simulation_only_edits=self.edited, smoothed=smoothed,
                          raw_path_length_m=raw_length if res.found else None, raw_waypoints=raw_points)
            self.report = report
            self.sim.set_route(res.waypoints)
            g = self.grid
            self._gt_guard = (self.planner.cache.checker(self.gt), self.planner.cache.checker(g),
                              config.footprint.effective_radius)
            self._say(res.message if self.dispatch_allowed or not res.found else self.dispatch_message, log=False)
            if running and self.dispatch_allowed:
                self.sim.start()
        except (ValueError, TypeError) as exc:
            self.dispatch_allowed = False
            self._say('Invalid settings: %s. Playback paused; last result kept.' % exc, 'warn', log=False)

    def cmd_sim_control(self, args):
        action = arg_choice(args, 'action', ('start', 'pause', 'reset'))
        with self._lock:
            if self.nav_active():
                return self._fail('Playback is for Planner mode.')
            if action == 'start':
                if not self.dispatch_allowed:
                    return self._fail(self.dispatch_message or 'Apply valid settings before starting.')
                if self.sim.state in ('arrived', 'collision'):
                    self.sim.reset()
                self.sim.start()
            elif action == 'pause':
                self.sim.pause()
            else:
                self.sim.reset()
            self._bump('offline')
            return result(True)

    def _offline_step(self, dt):
        sim = self.sim
        if sim.state != 'running' or self.nav_active() or self._gt_guard is None:
            return
        with self._lock:
            sim.speed = float(self.opts['sim_speed'])
            gt_checker, occ_checker, radius = self._gt_guard
            sim.step(dt, lambda a, b: gt_checker.is_clear(a, b, radius) and occ_checker.is_clear(a, b, radius))
            if sim.state == 'collision':
                self._say('Occupancy/GT monitor: motion stopped before a collision/clearance violation. '
                          'The route was not repaired using GT.', 'warn')
            if sim.state != 'running':
                self._bump('offline')

    def cmd_export_run(self, args):
        """PNG / CSV / JSON of the current planner-mode run (same files as the original GUI)."""
        from ugv_planner.exporting import export_run, export_trajectory
        with self._lock:
            if self.nav_active() or self.result is None:
                return self._fail('Export is for Planner mode (after a plan).')
            folder = self.export_dir / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            report = dict(self.report, simulation_state=self.sim.state, simulation_final_position_m=self.sim.position)
            env, grid, gt, res, cfg, history = (self.opts['env'], self.grid, self.gt, self.result, self.active_config,
                                                list(self.sim.history))
        export_run(folder, env, grid, gt, res, report, cfg)            # heavy (matplotlib): outside the lock
        export_trajectory(folder, env, history)
        return self._ok('Exported to %s' % folder, 'success', folder=str(folder))

    def cmd_batch_run(self, args):
        """Run and export the three example environments (original maps), like the original GUI button."""
        from ugv_planner.batch import run_batch
        with self._lock:
            self.sim.pause()
            config = self.planner_config()
        folder = self.export_dir / datetime.now().strftime('comparison_%Y%m%d_%H%M%S_%f')
        reports = run_batch(folder, config)
        keys = ['path_found', 'path_length_m', 'planning_ms', 'expanded_nodes', 'turns', 'detour_ratio',
                'minimum_gt_clearance_cm', 'gt_center_collision', 'gt_footprint_collision', 'simulation_state']
        table = dict(metrics=keys, environments=list(reports),
                     rows=[[reports[name].get(k) for name in reports] for k in keys],
                     validation={name: reports[name]['validation'] for name in reports})
        return self._ok('Comparison exported to %s' % folder, 'success', folder=str(folder), table=self.clean(table))

    # ------------------------------------------------------------------ connection
    def cmd_connect(self, args):
        with self._lock:
            if not self.nav_active():
                return self._fail('Choose Simulator or Robot mode first.')
            self._disconnect()
            o = self.opts
            try:
                base = self.edge_cfg.navigator if self.edge_cfg else NavigatorConfig()
                room_mode = o['lidar_sees'] == 'room'
                # own copy of the localization settings: the live switch must never change the loaded config
                localization = dataclasses.replace(base.localization, ignore_drift=o['ignore_drift'])
                nav_cfg = dataclasses.replace(base, safety=self._safety_config(), localization=localization,
                                              auto_continue_after_replan=o['auto_continue'],
                                              allow_tight_radius=o['allow_tight'], smooth_path=o['smooth_path'],
                                              lidar_correction=o['lidar_correction'], auto_find=o['auto_find'],
                                              lidar_reference='room' if room_mode else 'map')
                planner_cfg = self.planner_config()
                if o['mode'] == SIM:
                    start = self.pending_pose or self.start_point
                    half_l, half_w = nav_cfg.safety.half_length_m, nav_cfg.safety.half_width_m
                    sim = dict(odom_scale_error=0.02)
                    if room_mode:                  # like the MentorPi: the LiDAR looks over the arena into the room
                        sim.update(lidar_world=simulated_room(self.gt), scan_noise_m=0.008)
                    self.session = SimulatedUGVSession(self.gt, (start[0], start[1], math.radians(o['heading'])),
                                                       half_l, half_w, sim_kwargs=sim)
                    self.client = self.session.client
                else:
                    if self.edge_cfg is None:
                        raise ValueError('config/navigation.toml could not be loaded: ' + self.config_error)
                    if not o['host']:
                        raise ValueError('Enter the MentorPi IP address.')
                    if not Path(self.edge_cfg.key_path).is_file():
                        raise ValueError('the shared key file %s is missing. Copy config/mentorpi.key from your old '
                                         'project folder (or run: python nav_main.py import-old <old folder>).'
                                         % self.edge_cfg.key_path)
                    key = load_key(self.edge_cfg.key_path)
                    self.client = EdgeClient(o['host'], int(o['port']), key, self.edge_cfg.network.client_id,
                                             link_timeout=self.edge_cfg.network.link_timeout_s).start()
                self.nav = Navigator(self.client, self.grid, planner_cfg, nav_cfg)
                self.nav.planner = self.planner                       # one planner cache for the whole console
                self._nav_log_seen = 0
                self.mission = SearchMission(self.nav, self._mission_config(),
                                             camera_active=lambda: self.vision is not None)
                self.nav.mission = self.mission
                self.nav.set_scale(validate_scale(o['scale_x'], 'X scale'), validate_scale(o['scale_y'], 'Y scale'))
                self._apply_speed()
                room_note = ''
                if room_mode:
                    if self.session is not None:  # the simulated LiDAR sits exactly in the centre: nothing to learn
                        self.nav.set_lidar_mount((0.0, 0.0, 0.0), True)
                    else:
                        self.nav.set_lidar_mount(self.lidar_mount, self.lidar_mount_ok)
                    room_note = self._load_room()
                self._room_saved, self._found_seen = None, 0
                self.nav.run_in_background()
                if o['camera_on']:
                    self._camera_timer = threading.Timer(0.8, self._camera_autostart)
                    self._camera_timer.daemon = True
                    self._camera_timer.start()
                self._settings_dirty = True
                self._bump('map', 'plan', 'settings')
                return self._ok('Connecting... The UGV is DISARMED. %s'
                                % (room_note or 'Place the robot (Corner start, or Start + heading) and press '
                                   'Confirm pose.'))
            except Exception as exc:
                self._disconnect()
                return self._fail('Connection failed: %s' % exc, 'error')

    def cmd_disconnect(self, args):
        with self._lock:
            was = self.nav is not None
            self._disconnect()
            self._bump('map', 'plan')
            return self._ok('Disconnected.' if was else 'Not connected.')

    def _disconnect(self):
        self._record_stop('the robot was disconnected', wait=True)      # save the video first: the camera goes away next
        if self._camera_timer is not None:
            self._camera_timer.cancel()
            self._camera_timer = None
        self._stop_camera(keep_setting=True)
        if self.mission is not None and self.mission.active:
            self.mission.stop()                   # survivor list stays visible until the next Connect
        if self.nav:
            self._collect_nav_log()
            self._room_housekeeping(force=True)
            self.nav.emergency_stop('Disconnecting')
            self.nav.shutdown()
        if self.session:
            self.session.close()
        elif self.client:
            self.client.close()
        self.nav = self.session = self.client = None
        self.cal = None
        self._auth = None
        self.sim_blocks = None
        self.field_samples, self.field_measured = [], None

    def close(self):
        """Program exit: stop everything, keep unsaved map edits as a backup file, save settings."""
        self._closing.set()
        with self._lock:
            self._uav_close()                      # a drone in the air is told to land
            self._disconnect()
            if self.map_unsaved:
                try:
                    path = edited_path(self.opts['env'], self.edit_dir)
                    backup = path.with_name('%s.unsaved_%s.bak.npy' % (path.stem, time.strftime('%Y%m%d_%H%M%S')))
                    _atomic_save_npy(backup, np.flipud(self.grid.cells).astype(np.uint8))
                    self.map_unsaved = False
                    print('Unsaved map edits kept as %s' % backup)
                except OSError:
                    pass
            self._save_settings()

    # ------------------------------------------------------------------ operator actions
    def cmd_stop(self, args):
        """STOP: never waits for a lock, a plan or a slow operation. A drone in the air is told to land."""
        reason = str(args.get('reason') or 'Operator STOP')[:80]
        landing = self._uav_stop(reason)
        nav, client = self.nav, self.client
        if nav is not None:
            nav.emergency_stop(reason)
        elif client is not None:
            client.stop()
        self.sim.pause()
        self._auth = None
        if landing:
            self._say('STOP pressed: the drone is landing.', 'warn')
        elif self.opts['section'] == 'uav':
            self._say('STOP pressed. The drone is not in the air.', 'warn', log=False)
        else:
            self._say('STOP pressed: robot stopped and disarmed. Re-plan / authorize to continue.', 'warn')
        self._bump('offline')
        return result(True, self.notice, 'warn')

    def is_moving(self):
        nav, mission = self.nav, self.mission
        return bool((nav is not None and nav.state == RUNNING)
                    or (mission is not None and mission.active and not mission.paused))

    def cmd_set_start(self, args):
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        heading = arg_number(args, 'heading', -360, 360, required=False)
        with self._lock:
            if not self.grid.contains(self.grid.world_to_cell((x, y))):
                return self._fail('That point is outside the map.')
            if heading is not None:
                self.opts['heading'] = round(heading, 1)
                self._bump('settings')
                self._settings_dirty = True
            self.dock = None
            self._bump('plan')
            if not self.nav_active():
                self.start_point = self.grid.cell_to_world(self.grid.world_to_cell((x, y)))
                self._recalculate(use_current=False)
                return result(True, self.notice)
            self.pending_pose = self.start_point = (round(x, 3), round(y, 3))    # exact: a measured robot position
            return self._ok('Start (%.3f, %.3f) m, heading %.0f deg selected: press Confirm pose.'
                            % (x, y, self.opts['heading']))

    def cmd_set_goal(self, args):
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        with self._lock:
            cell = self.grid.world_to_cell((x, y))
            if not self.grid.contains(cell):
                return self._fail('That point is outside the map.')
            self.goal_point = self.grid.cell_to_world(cell)          # A* goals are cell centres
            self._bump('plan')
            if not self.nav_active():
                self._recalculate()
                return result(True, self.notice)
            return self._plan_to_goal()

    def _plan_to_goal(self):
        if self.goal_point is None:
            return self._fail('Set a goal first: Goal tool, then click the map.')
        if not self.nav:
            return self._fail('Connect first (the goal is remembered).')
        self.nav.set_planner_config(self.planner_config())
        self.nav.set_goal(self.goal_point)
        self._bump('plan')
        found = self.nav.state == PLANNED
        self._say(self.nav.message, 'info' if found else 'warn', log=False)
        return result(found, self.nav.message, 'info' if found else 'warn')

    def cmd_apply_coordinates(self, args):
        """Typed Start/Goal coordinates. Robot modes: Start = pose to confirm, Goal = plan."""
        sx, sy = arg_number(args, 'sx', -1, 100), arg_number(args, 'sy', -1, 100)
        has_goal = args.get('gx') is not None and args.get('gy') is not None     # blank = no goal chosen yet
        gx, gy = (arg_number(args, 'gx', -1, 100), arg_number(args, 'gy', -1, 100)) if has_goal else (None, None)
        with self._lock:
            g = self.grid
            if not g.contains(g.world_to_cell((sx, sy))) or (has_goal and not g.contains(g.world_to_cell((gx, gy)))):
                return self._fail('Start and goal must be inside the map.')
            self._bump('plan')
            if has_goal:
                self.goal_point = g.cell_to_world(g.world_to_cell((gx, gy)))
            if not self.nav_active():
                # Explicit coordinate input changes both endpoints (cell centres); no GT repair.
                self.start_point = g.cell_to_world(g.world_to_cell((sx, sy)))
                self._recalculate(use_current=False)
                return result(True, self.notice)
            current = self.pending_pose or self.start_point
            if current is None or math.dist((sx, sy), current) > 1e-6:
                self.dock = None
                self.pending_pose = self.start_point = (sx, sy)
                if self.nav:
                    return self._ok('Start set to (%.3f, %.3f). Press Confirm pose, then Plan again to plan to the '
                                    'goal.' % (sx, sy))
            return self._plan_to_goal()

    def cmd_confirm_pose(self, args):
        with self._lock:
            if not self.nav:
                return self._fail('Connect first.')
            try:
                xy = self.pending_pose or self.start_point
                pose = (xy[0], xy[1], math.radians(float(self.opts['heading'])))
                docked = bool(self.dock and math.dist(self.dock['pose'][:2], xy) < 1e-6
                              and abs(self.dock['pose'][2] - float(self.opts['heading'])) < 1e-6)
                if self.session:                  # simulator: physically place the simulated robot there
                    if self.nav.state == RUNNING:
                        return self._fail('Stop the robot first.')
                    self.session.backend.place(pose)
                    time.sleep(0.15)
                self.nav.set_planner_config(self.planner_config())
                self.nav.set_initial_pose(pose, source='dock' if docked else 'manual')
                self._bump('plan')
                return self._ok(self.nav.message, 'success')
            except ValueError as exc:
                return self._fail('Robot pose rejected: %s' % exc)

    # ------------------------------------------------------------------ where is the robot? (no pin by eye)
    def _corner_start(self, corner, turn=True):
        half_l, half_w = self.robot_half_size()
        heading = float(self.opts['heading'])
        if turn and self.dock and self.dock['corner'] == corner:
            heading += 90.0                        # same corner again: the robot faces the next way
        gap = float(self.opts['corner_gap_cm']) / 100.0
        x, y, heading = corner_pose(self.grid, corner, heading, half_l, half_w, gap)
        heading = heading if heading <= 180 else heading - 360
        if not self.grid.contains(self.grid.world_to_cell((x, y))):
            raise ValueError('That corner is outside the map.')
        self.pending_pose = self.start_point = (round(x, 4), round(y, 4))
        self.opts['heading'] = heading
        self.dock = dict(corner=corner, pose=(self.pending_pose[0], self.pending_pose[1], heading))
        self._bump('plan', 'settings')
        self._settings_dirty = True
        facing = {0: 'right (+x)', 90: 'up (+y)', 180: 'left (-x)', -90: 'down (-y)'}[int(heading)]
        try:
            ok, _ = validate_path([self.pending_pose], self.grid, self.planner_config().planning_radius)
        except ValueError:
            ok = True
        text = ('Corner start: stand the robot square in the %s corner, %.0f cm from both walls%s, facing %s. Then '
                'press Confirm pose. Click the same corner again to turn it.'
                % (CORNER_NAMES[corner], gap * 100, ' (one block length)' if abs(gap - 0.10) < 1e-6 else '', facing))
        if not ok:
            text += (' NOTE: on the map this spot is too close to a wall or obstacle for A* to start from: use a '
                     'larger distance or another corner.')
        self._say(text, 'info' if ok else 'warn')
        return ok, text

    def cmd_corner_start(self, args):
        """Start pose from a corner of the arena: a physical reference instead of a pin placed by eye."""
        corner = arg_choice(args, 'corner', CORNERS)
        with self._lock:
            if not self.nav_active():
                return self._fail('Corner start is for Simulator and Robot mode.')
            ok, text = self._corner_start(corner)
            return result(True, text, 'info' if ok else 'warn')

    def cmd_find_robot(self, args):
        """Ask the LiDAR where the robot is (no pin). ``choose``: pick one of the places offered before."""
        nav = self.nav
        if not nav:
            return self._fail('Connect first.')
        choose = args.get('choose')
        if choose is not None:
            choose = int(arg_number(args, 'choose', 0, 8))
        deadline = time.monotonic() + 2.5
        while nav.state != RUNNING and not nav.robot_still() and time.monotonic() < deadline:
            time.sleep(0.1)
        ok, message = nav.find_robot(choose)
        with self._lock:
            if self.nav is not nav:
                return self._fail('Not connected any more.')
            self._bump('plan')
            if ok:
                self._follow_found()
                return self._ok(message, 'success')
            found = nav.found
            if found:
                return result(False, message, 'warn', candidates=[
                    dict(x=round(c['pose'][0], 3), y=round(c['pose'][1], 3),
                         heading=round(math.degrees(c['pose'][2]), 1), fit=round(c['matched'], 2))
                    for c in found['candidates']])
            return self._fail(message)

    def _follow_found(self):
        """The navigator placed the robot by LiDAR: show the Start marker there too."""
        pose = self.nav.estimator.pose() if self.nav else None
        if pose is None:
            return
        self.dock = None
        self.pending_pose = self.start_point = (round(pose[0], 3), round(pose[1], 3))
        self.opts['heading'] = round(math.degrees(pose[2]), 1)
        self._bump('plan', 'settings')

    def cmd_nudge_pose(self, args):
        """Move the confirmed position a little by hand: dx / dy in cm (map axes), turn in degrees."""
        dx, dy = arg_number(args, 'dx', -50, 50, 0.0), arg_number(args, 'dy', -50, 50, 0.0)
        turn = arg_number(args, 'turn', -180, 180, 0.0)
        with self._lock:
            nav = self.nav
            if not nav:
                return self._fail('Connect first.')
            pose = nav.estimator.pose()
            if pose is None:
                return self._fail('Set the robot pose first (Corner start or Start, then Confirm pose).')
            if nav.state == RUNNING:
                return self._fail('Stop the robot first.')
            new = (pose[0] + dx / 100.0, pose[1] + dy / 100.0, pose[2] + math.radians(turn))
            try:
                nav.adjust_pose(new)
            except ValueError as exc:
                return self._fail('Not moved: %s' % exc)
            self._found_seen = nav.found_seq
            self._follow_found()
            return self._ok(nav.message)

    def cmd_forget_room(self, args):
        """Throw away what the LiDAR learnt about the room (and, on request, about its own mounting)."""
        with self._lock:
            path = self._room_path()
            had = path.exists()
            if had:
                path.unlink()
            if arg_bool(args, 'mount') and self.session is None:
                self.lidar_mount, self.lidar_mount_ok = (0.0, 0.0, 0.0), False
                self._settings_dirty = True
            nav = self.nav
            if nav is not None and nav.cfg.lidar_reference == 'room':
                if nav.state == RUNNING:
                    return self._fail('Stop the robot first.')
                if self.session is None:
                    nav.set_lidar_mount(self.lidar_mount, self.lidar_mount_ok)
                else:
                    nav.set_lidar_mount((0.0, 0.0, 0.0), True)
                pose = nav.estimator.pose()
                if pose is not None:
                    nav._room().restart(pose, trusted=False)
            self._room_saved = None
            self._bump('settings')
            return self._ok('Room outline forgotten%s. It is drawn again from the next Confirm pose.'
                            % (' (saved file removed)' if had else ''))

    def _room_path(self):
        return self.room_dir / ('%s_room%s.npz' % (self.opts['env'], '_sim' if self.session is not None else ''))

    def _load_room(self):
        path = self._room_path()
        if not path.is_file():
            return ''
        try:
            text = self.nav.load_room(path.read_bytes())
        except (OSError, ValueError) as exc:
            self._say('The saved room outline is not used: %s.' % exc, 'warn')
            return ''
        return ('Known %s: the robot is found by itself in a moment (or press Find robot).' % text
                if self.opts['auto_find'] else 'Known %s: press Find robot.' % text)

    def _room_housekeeping(self, force=False):
        """Remember what the robot taught us: the LiDAR mounting (real robot) and the room outline."""
        nav = self.nav
        if nav is None or nav.cfg.lidar_reference != 'room' or nav.room is None:
            return
        room = nav.room
        if self.session is None:
            mount = tuple(round(float(v), 5) for v in nav.safety.lidar_mount)
            moved = mount != tuple(round(float(v), 5) for v in self.lidar_mount)
            if moved or bool(room.calibrated) != self.lidar_mount_ok:
                self.lidar_mount, self.lidar_mount_ok = mount, bool(room.calibrated)
                self._settings_dirty = True
                self._bump('settings')
                if moved and self._room_path().exists():           # drawn with the old mounting: useless now
                    self._room_path().unlink()
        key = (id(room.map), room.map.rev)
        now = time.monotonic()
        if key == self._room_saved or (not force and now - self._room_check < 10.0):
            return
        self._room_check = now
        data = nav.room_file()
        if data is None:
            return
        try:
            self.room_dir.mkdir(parents=True, exist_ok=True)
            path = self._room_path()
            tmp = path.with_suffix('.tmp')
            tmp.write_bytes(data)
            tmp.replace(path)
            self._room_saved = key
        except OSError as exc:
            self._say('Could not save the room outline: %s' % exc, 'warn')

    # ------------------------------------------------------------------ real size of the field
    def _map_inner(self, env=None):
        """Inner size (x, y) of the field as the DATA FILE draws it, in metres at 3 cm per cell."""
        base, _ = load_env(env or self.opts['env'])
        x0, x1, y0, y1 = inner_box(base)
        return x1 - x0, y1 - y0

    def _set_field_scale(self, scale, why):
        env = self.opts['env']
        if self.map_unsaved:
            raise ValueError('Save or discard the unsaved map edits first.')
        if self.is_moving():
            raise ValueError('Stop the robot first.')
        scale = validate_field_scale(scale)
        if all(abs(v - 1.0) < 1e-9 for v in scale):
            self.field_scale.pop(env, None)
        else:
            self.field_scale[env] = scale
        self._settings_dirty = True
        note = ''
        if self.session is not None:               # the simulated world has the old size: build it again
            self._disconnect()
            note = ' Simulator stopped: press Start simulator again.'
        self._load_env()
        if self.nav:
            self.nav.forget_pose('The field size changed')
            self._room_saved = None
            note = ' Place the robot and confirm its pose again.'
        w, h = self._map_inner()
        self._bump('map', 'plan', 'settings')
        return self._ok('%s: the field of %s is now %.2f x %.2f m inside the walls (the data file alone says '
                        '%.2f x %.2f m).%s' % (why, env, w * scale[0], h * scale[1], w, h, note), 'success')

    def cmd_field_apply(self, args):
        """Real inner size of the field (wall to wall, metres): the map is stretched to it."""
        x, y = arg_number(args, 'x', 0.5, 10.0), arg_number(args, 'y', 0.5, 10.0)
        with self._lock:
            w, h = self._map_inner()
            return self._set_field_scale((x / w, y / h), 'Field size applied')

    def cmd_field_reset(self, args):
        with self._lock:
            return self._set_field_scale((1.0, 1.0), 'Field size reset')

    def cmd_field_sample(self, args):
        """Measure the field with the robot: stand it in a corner exactly like for a corner start (the same
        distance from both walls) and call this with that corner. Two corners give the real distance between
        the walls, taken from the LiDAR's view of the room (not from the wheels)."""
        corner = arg_choice(args, 'corner', CORNERS)
        with self._lock:
            nav = self.nav
            if not nav or nav.cfg.lidar_reference != 'room' or nav.room is None:
                return self._fail('Measuring with the robot needs live LiDAR tracking of the room (Robot tab). '
                                  'Without it, measure the inner width and depth with a tape and type them in.')
            if nav.state == RUNNING or not nav.robot_still():
                return self._fail('The robot must stand still.')
            room = nav.room
            pose = nav.estimator.pose()
            if pose is None or room.state != 'locked' or not room.calibrated:
                return self._fail('The LiDAR is not sure where the robot is right now: %s. Wait a moment after '
                                  'putting it down. Measuring works once the robot has driven through the arena '
                                  '(then the LiDAR has drawn the whole room).' %
                                  ('it is still learning how the LiDAR is mounted' if not room.calibrated
                                   else room.reason or room.state))
            half_l, half_w = self.robot_half_size()
            heading = (int(round(math.degrees(pose[2]) / 90.0)) % 4) * 90
            if abs(((math.degrees(pose[2]) - heading + 180) % 360) - 180) > 12:
                return self._fail('Stand the robot square to the walls (it is turned %.0f deg).'
                                  % (((math.degrees(pose[2]) - heading + 180) % 360) - 180))
            gap = float(self.opts['corner_gap_cm']) / 100.0
            ex, ey = (half_l, half_w) if heading in (0, 180) else (half_w, half_l)
            ex, ey = ex + gap, ey + gap             # robot centre to the wall = half the body + the spacer
            expect = corner_pose(self.grid, corner, heading, half_l, half_w, gap)
            if math.dist(expect[:2], pose[:2]) > 0.45:
                return self._fail('The robot does not seem to be in the %s corner (the LiDAR puts it %.0f cm away). '
                                  'Stand it there like for a corner start, wait until the console shows it there, '
                                  'then try again.' % (CORNER_NAMES[corner], 100 * math.dist(expect[:2], pose[:2])))
            self.field_samples = [s_ for s_ in self.field_samples if s_['corner'] != corner]
            self.field_samples.append(dict(corner=corner, x=pose[0], y=pose[1], ex=ex, ey=ey))
            xs = {s_['corner'][1]: s_ for s_ in self.field_samples}
            ys = {s_['corner'][0]: s_ for s_ in self.field_samples}
            width = (xs['r']['x'] + xs['r']['ex']) - (xs['l']['x'] - xs['l']['ex']) if len(xs) == 2 else None
            depth = (ys['t']['y'] + ys['t']['ey']) - (ys['b']['y'] - ys['b']['ey']) if len(ys) == 2 else None
            self.field_measured = (width, depth)
            self._bump('settings')
            if width is None and depth is None:
                return self._ok('%s corner measured. Now carry the robot to another corner, stand it there the '
                                'same way (%.0f cm from both walls), wait until the console shows it there, and '
                                'press that corner.' % (CORNER_NAMES[corner].capitalize(), gap * 100))
            parts = [('width (X) %.2f m' % width) if width is not None else '',
                     ('depth (Y) %.2f m' % depth) if depth is not None else '']
            return self._ok('Measured with the robot: %s. Press Apply to use it for the map.'
                            % ' and '.join(p for p in parts if p), 'success')

    def cmd_field_clear(self, args):
        with self._lock:
            self.field_samples, self.field_measured = [], None
            self._bump('settings')
            return result(True)

    def cmd_sync_pose_sim(self, args):
        """Simulator only: re-localize to the simulated robot's true pose (like a perfect re-measure)."""
        with self._lock:
            if not self.session or not self.nav:
                return self._fail('Only in the simulator. On the real robot: set Start at its real position, set '
                                  'the heading and press Confirm pose (or use Re-localize with LiDAR).')
            try:
                self.nav.set_initial_pose(self.session.backend.truth_pose())
            except ValueError as exc:
                return self._fail(str(exc))
            self._bump('plan')
            return self._ok(self.nav.message, 'success')

    def cmd_relocalize(self, args):
        """Correct the pose by fitting the LiDAR scan to the map (operator request, robot standing still)."""
        nav = self.nav
        if not nav:
            return self._fail('Connect first.')
        deadline = time.monotonic() + 2.5           # pressed right after a stop: let the robot finish braking
        while nav.state != RUNNING and not nav.robot_still() and time.monotonic() < deadline:
            time.sleep(0.1)
        with self._lock:
            if self.nav is not nav:
                return self._fail('Not connected any more.')
            ok, message = nav.relocalize_with_lidar()
            self._bump('plan')
            return self._ok(message, 'success') if ok else self._fail(message)

    # ------------------------------------------------------------------ authorization (two steps)
    def _search_ready(self):
        if not self.nav or self.mission is None:
            return 'Connect first.'
        if self.vision is None:
            return ('Turn on Camera + YOLO (Camera tab) before starting a search: the robot finds survivors with '
                    'the camera.')
        return None

    def cmd_authorize_prepare(self, args):
        action = arg_choice(args, 'action', ('drive', 'search_here', 'resume'), 'drive')
        with self._lock:
            nav, mission = self.nav, self.mission
            if not nav:
                return self._fail('Connect first.')
            if self.uav_airborne():
                return self._fail('The drone is in the air: land it before the robot drives.')
            snap = nav.snapshot()
            search = False
            safety = 'Confirm the area is physically clear and you can reach STOP.'
            if action == 'resume':
                if not (mission.active and mission.paused):
                    return self._fail('No paused search mission.')
                if snap['localization'][0] != 'OK':
                    return self._fail('Localization is %s (%s). Confirm the robot pose or re-localize first.'
                                      % snap['localization'])
                title, label = 'Resume search mission', 'Resume mission'
                text = 'Resume the survivor search?\n\nPaused because: %s' % mission.pause_reason
            elif action == 'search_here':
                problem = self._search_ready()
                if problem:
                    return self._fail(problem)
                if mission.active and not mission.paused:
                    return self._fail('The search mission is already running.')
                if snap['state'] == RUNNING:
                    return self._fail('Wait until the robot has stopped.')
                if snap['pose'] is None or snap['localization'][0] != 'OK':
                    return self._fail('Confirm the robot pose first.')
                title, label = 'Start search mission', 'Start search'
                text = ('Start the survivor search HERE?\n\nThe robot will turn, explore and approach green objects '
                        'on its own. It goes to the first green object it sees, stands in front of it and marks it secured.')
            else:
                if mission.active and mission.paused:
                    return self._fail('The search mission is paused: use Resume mission.')
                if snap['state'] != PLANNED or not snap['path']:
                    return self._fail('Plan a route first: confirm the robot pose, then set a goal.')
                search = bool(self.opts['search_at_goal'])
                if search:
                    problem = self._search_ready()
                    if problem:
                        return self._fail(problem)
                length = path_length(snap['path'])
                title, label = 'Authorize navigation', 'Authorize'
                text = ('Start autonomous motion along the planned %.2f m route?' % length if not search else
                        'Drive the planned %.2f m route, then SEARCH FOR SURVIVORS on its own (turning, exploring '
                        'and approaching the first green object it sees) until the survivor is secured?' % length)
            nonce = secrets.token_hex(8)
            self._auth = dict(nonce=nonce, action=action, expires=time.monotonic() + 60.0, path=snap['path'],
                              search=search)
            return result(True, '', nonce=nonce, title=title, text=text + '\n\n' + safety, confirm_label=label)

    def cmd_authorize_confirm(self, args):
        nonce = args.get('nonce')
        with self._lock:
            auth, self._auth = self._auth, None
            if (not auth or not isinstance(nonce, str) or not hmac.compare_digest(auth['nonce'], nonce)
                    or time.monotonic() > auth['expires']):
                return self._fail('Authorization expired: press the button again.')
            nav, mission = self.nav, self.mission
            if not nav:
                return self._fail('Not connected.')
            if self.uav_airborne():
                return self._fail('The drone is in the air: land it before the robot drives.')
            if auth['action'] == 'resume':
                ok, message = mission.resume()
                return self._ok(message, 'success') if ok else self._fail(message)
            if auth['action'] == 'search_here':
                if nav.state == RUNNING or self._search_ready():
                    return self._fail('The situation changed: try again.')
                mission.cfg = self._mission_config()
                mission.start(transit=False)
                self._record_start()
                return self._ok('Search mission started from the current position.', 'success')
            if nav.snapshot()['path'] != auth['path'] or nav.state != PLANNED:
                return self._fail('The route changed since you were asked: check it and authorize again.')
            ok, message = nav.authorize()
            if ok and auth['search']:
                mission.cfg = self._mission_config()
                mission.start(transit=True)
                self._record_start()
                message = 'Mission started: driving to the goal, then searching for survivors.'
            return self._ok(message, 'success') if ok else self._fail('Not authorized: %s' % message)

    # ------------------------------------------------------------------ survivor search mission
    def cmd_stop_mission(self, args):
        with self._lock:
            mission = self.mission
            if mission is None or not mission.active:
                return self._ok('No search mission is running.')
            mission.stop()
            return self._ok('Search mission stopped. %d survivor(s) on the list.' % len(mission.survivors))

    # ------------------------------------------------------------------ mission video
    def _recording_folder(self):
        custom = str(self.opts['record_dir']).strip()
        if not custom:
            return default_folder()
        folder = Path(custom).expanduser()
        return folder if folder.is_absolute() else ROOT / folder

    def _record_frame(self):
        vision = self.vision
        return vision.wait_frame(None, 0) if vision is not None else None      # newest picture, never waits

    def _record_info(self):
        mission = self.mission
        if mission is None:
            return {}
        snap = mission.snapshot(blocking=False)
        rows, events = snap['survivors'], snap['events']
        return dict(phase=snap['phase'], elapsed=snap['elapsed'], found=len(rows),
                    secured=sum(1 for s in rows if s['status'] == 'secured'), event=events[-1] if events else None)

    def _record_start(self):
        """A search mission has just started: film it, unless the operator switched that off."""
        if not self.opts['record_mission']:
            return
        self._record_stop('a new mission started', wait=True)
        try:
            recorder = MissionRecorder(self._recording_folder(), self._record_frame, self._record_info,
                                       label=self.opts['env'], overlay=bool(self.opts['record_overlay'])).start()
        except RecorderError as exc:
            self.recording_note = 'The mission is NOT being filmed: %s.' % exc
            self._say(self.recording_note, 'warn')
            self._bump('recording')
            return
        self.recorder, self.recording_note = recorder, ''
        self._say('Filming the mission. The video is saved in %s when the mission ends.' % recorder.folder)
        self._bump('recording')

    def _record_stop(self, reason, tail=0.0, wait=False):
        recorder = self.recorder
        if recorder is None:
            return
        recorder.stop(reason, tail_s=tail, wait=wait)
        if wait:
            self._record_collect()

    def _record_collect(self):
        """Take over a finished video: tell the operator where it is (or why there is none)."""
        recorder = self.recorder
        if recorder is None or not recorder.done:
            return
        self.recorder = None
        result, fault = recorder.take_result()
        if result is not None:
            self.last_recording = result.as_dict()
            self.recordings_made += 1
            seconds = int(round(result.seconds))
            self._say('Mission video saved: %s (%d:%02d, %.1f MB).' % (result.path, seconds // 60, seconds % 60,
                                                                       result.size_bytes / 1024 ** 2), 'success')
        elif not fault:
            fault = 'the camera delivered no pictures, so nothing was saved'
        self.recording_note = ('Mission video: %s.' % fault) if fault else ''
        if fault:
            self._say(self.recording_note, 'warn')
        self._bump('recording')

    def _record_housekeeping(self):
        """Called by the ticker: save the video when the mission has ended (with 2 s of tail), or when the recorder
        ended by itself (disk almost full, maximum length, error)."""
        recorder = self.recorder
        if recorder is None:
            return
        if recorder.done:
            return self._record_collect()
        mission = self.mission
        if not recorder.stopping and (mission is None or not mission.active):
            recorder.stop('mission complete' if mission is not None and mission.complete else 'mission stopped',
                          tail_s=2.0)
            self._bump('recording')

    def _open_folder(self, folder):
        if os.name == 'nt':
            os.startfile(str(folder))                                             # noqa: the Windows file explorer
        elif sys.platform == 'darwin':
            subprocess.Popen(['open', str(folder)])
        else:
            subprocess.Popen(['xdg-open', str(folder)])

    def cmd_open_recordings(self, args):
        try:
            folder = check_folder(self._recording_folder())
            self._open_folder(folder)
        except RecorderError as exc:
            return self._fail('Cannot open the video folder: %s.' % exc)
        except OSError as exc:
            return self._fail('Could not open the folder: %s' % exc)
        return self._ok('Opened %s.' % folder)

    def cmd_clear_survivors(self, args):
        with self._lock:
            mission = self.mission
            if mission is None or not mission.survivors:
                return result(True)
            if mission.active:
                return self._fail('Stop the mission before clearing the survivor list.')
            if not arg_bool(args, 'confirm'):
                return result(False, 'Remove all %d survivors from the list?' % len(mission.survivors), 'warn',
                              need_confirm='clear_survivors')
            mission.clear_survivors()
            return self._ok('Survivor list cleared.')

    def cmd_export_survivors(self, args):
        with self._lock:
            rows = self.mission.snapshot()['survivors'] if self.mission is not None else []
            env = self.opts['env']
        if not rows:
            return self._fail('No survivors to export yet.')
        self.export_dir.mkdir(parents=True, exist_ok=True)
        path = self.export_dir / ('survivors_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S'))
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['survivor', 'x_m', 'y_m', 'found_after_s', 'found_at', 'status', 'environment'])
            for r in rows:
                writer.writerow(['S%d' % r['id'], '%.3f' % r['x'], '%.3f' % r['y'], r['found_s'], r['found_clock'],
                                 r['status'], env])
        return self._ok('Survivor list exported to %s' % path, 'success', path=str(path))

    # ------------------------------------------------------------------ scale calibration
    def cmd_apply_scale(self, args):
        sx = validate_scale(arg_number(args, 'x', 0.1, 10), 'X scale')
        sy = validate_scale(arg_number(args, 'y', 0.1, 10), 'Y scale')
        with self._lock:
            return self._set_scale(sx, sy)

    def cmd_reset_scale(self, args):
        with self._lock:
            return self._set_scale(1.0, 1.0)

    def _set_scale(self, sx, sy):
        if self.nav:
            self.nav.set_scale(sx, sy)            # ValueError while driving
        self.opts['scale_x'], self.opts['scale_y'] = round(sx, 3), round(sy, 3)
        self._bump('settings')
        self._settings_dirty = True
        return self._ok('Scale calibration X %.3f, Y %.3f %s and saved.'
                        % (sx, sy, 'applied' if self.nav else 'stored (applies on Connect)'), 'success')

    def cmd_cal_start(self, args):
        axis = arg_choice(args, 'axis', ('x', 'y'))
        with self._lock:
            if not self.nav:
                return self._fail('Connect first.')
            snap = self.nav.snapshot()
            if snap['state'] == RUNNING:
                return self._fail('Stop the robot first.')
            raw, anchor = snap['raw_pose'], snap['anchor']
            if raw is None or anchor is None or snap['localization'][0] != 'OK':
                return self._fail('Place the robot, set its start position and heading, and press Confirm pose '
                                  'first.')
            if math.dist(raw[:2], anchor) > 0.03:
                return self._fail('The robot has moved since the pose was confirmed. Confirm pose at its real '
                                  'position again, then start the run.')
            self.cal = dict(axis=axis, raw_start=tuple(raw[:2]), true_start=tuple(anchor))
            return self._ok('%s calibration run started at (%.3f, %.3f). Drive the robot along the %s axis (set a '
                            'goal at the far wall + AUTHORIZE). When it has stopped, enter its REAL centre position '
                            '(tape measure, or pick it on the map) and press Finish run.'
                            % (axis.upper(), anchor[0], anchor[1], axis.upper()))

    def cmd_cal_cancel(self, args):
        with self._lock:
            self.cal = None
            return self._ok('Calibration run cancelled; scale unchanged.')

    def cmd_cal_finish(self, args):
        true_end = (arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100))
        with self._lock:
            if not self.cal or not self.nav:
                return self._fail('Start an X or Y run first.')
            snap = self.nav.snapshot()
            if snap['state'] == RUNNING:
                return self._fail('Wait until the robot has stopped.')
            axis = self.cal['axis']
            scale, info = compute_axis_scale(axis, self.cal['raw_start'], snap['raw_pose'][:2],
                                             self.cal['true_start'], true_end)        # ValueError if unusable
            old = self.opts['scale_x' if axis == 'x' else 'scale_y']
            sx, sy = (scale, self.opts['scale_y']) if axis == 'x' else (self.opts['scale_x'], scale)
            self._set_scale(validate_scale(sx, 'X scale'), validate_scale(sy, 'Y scale'))
            message = ('%s run: robot measured %.3f m, real distance %.3f m -> %s scale %.3f (was %.3f). Sideways '
                       'error %.1f cm.' % (axis.upper(), info['robot_measured_m'], info['marked_m'], axis.upper(),
                                           scale, old, info['sideways_error_m'] * 100))
            self.cal = None
            try:                                   # continue from the measured real position
                self.nav.set_initial_pose((true_end[0], true_end[1], snap['pose'][2]))
                self.pending_pose = self.start_point = true_end
            except ValueError:
                message += ' Before navigating, confirm the pose again (robot is close to a wall).'
            self._bump('plan')
            return self._ok(message, 'success')

    # ------------------------------------------------------------------ camera + YOLO
    def _camera_model(self):
        from ugv_nav.vision import CameraModel
        o = self.opts
        cam = CameraModel(height_m=o['camera_height_cm'] / 100.0, pitch_rad=math.radians(o['camera_pitch_deg']),
                          x_offset_m=o['camera_forward_cm'] / 100.0, yaw_offset_rad=math.radians(o['camera_yaw_deg']))
        info = self.client.camera_info if self.client is not None else None
        return CameraModel.from_camera_info(info, cam) if info else cam

    def _camera_autostart(self):
        with self._lock:
            if self.nav and self.opts['camera_on'] and self.vision is None:
                try:
                    self._say(self._start_camera())
                except ValueError as exc:
                    self._say(str(exc), 'warn')
                self._bump('settings')

    def _start_camera(self):
        if self.vision is not None:
            return 'Camera + YOLO is already running.'
        if not self.nav or not self.client:
            return 'Camera + YOLO starts when you connect (setting remembered).'
        import importlib.util
        if importlib.util.find_spec('cv2') is None:
            self.opts['camera_on'] = False
            raise ValueError('Camera + YOLO needs the extra packages: python -m pip install -r requirements-yolo.txt')
        from ugv_nav.vision import CameraModel, ObstacleTracker, VisionWorker, load_detector
        cam = self._camera_model()
        try:
            if self.session is not None:          # simulator: virtual camera + exact detector (1 px jitter)
                from ugv_nav.vision_sim import SimBlocks, SimCamera, SimDetector
                true_cam = CameraModel(**SIM_TRUE_CAMERA)
                self.sim_blocks = self.sim_blocks or SimBlocks()
                backend = self.session.backend
                backend.blocks = self.sim_blocks
                backend.camera_source = SimCamera(backend, self.sim_blocks, true_cam)
                detector = SimDetector(backend, self.sim_blocks, true_cam, noise_px=0.7)
            else:
                detector = load_detector()
        except ImportError as exc:
            self.opts['camera_on'] = False
            raise ValueError('YOLO is not available: %s' % exc)
        self.client.request_camera(8)
        from ugv_nav.vision import TrackerConfig
        tracker = ObstacleTracker(TrackerConfig(min_confidence=self.opts['yolo_min_conf'] / 100.0))
        self.vision = VisionWorker(self.client, lambda: self.nav, detector, cam, tracker,
                                   auto_tilt=self.opts['auto_tilt'], shape_filter=self.opts['shape_filter'],
                                   colour_check=self.opts['colour_check'],
                                   on_camera_change=self._on_camera_change).start()
        return 'Camera + YOLO started. Blocks seen by the camera appear as pins and are avoided by A*.'

    def _stop_camera(self, keep_setting=False):
        if self.vision is not None:
            self.vision.stop()
            self.vision = None
        if self.client is not None:
            try:
                self.client.request_camera(0)
            except Exception:
                pass
        if not keep_setting:
            self.opts['camera_on'] = False
        return 'Camera + YOLO stopped.'

    def _on_camera_change(self, cam, estimate):
        """Vision thread: the tilt was learned from block sightings. No locks here (see vision.py)."""
        self.opts['camera_pitch_deg'] = round(math.degrees(cam.pitch_rad), 1)
        self._settings_dirty = True
        self._bump('settings')
        self._say('Camera tilt set to %.1f deg automatically (from %d block sightings).'
                  % (estimate['deg'], estimate['n']), 'success')

    def cmd_use_tilt_estimate(self, args):
        """Apply the tilt estimated from block sightings once (when auto tilt is off)."""
        with self._lock:
            estimate = self.vision.snapshot()['tilt'] if self.vision else None
            if not estimate or not estimate['confident']:
                return self._fail('No reliable tilt estimate yet: show the camera a whole block at 0.3-1.5 m for a '
                                  'few seconds.')
            self.opts['camera_pitch_deg'] = round(estimate['deg'], 1)
            self.vision.set_camera(self._camera_model(), keep_tilt_samples=True)
            self._bump('settings')
            self._settings_dirty = True
            return self._ok('Camera tilt set to %.1f deg (from %d block sightings).' % (estimate['deg'], estimate['n']),
                            'success')

    def cmd_clear_camera_obstacles(self, args):
        with self._lock:
            if self.vision:
                self.vision.clear_obstacles()
            elif self.nav:
                self.nav.set_vision_obstacles([], reason='operator')
            return self._ok('Camera obstacles cleared.')

    def cmd_sim_block(self, args):
        """Simulator: place a virtual block at (x, y), or remove the one already there."""
        x, y = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        with self._lock:
            if self.session is None:
                return self._fail('Test blocks only exist in the simulator (start it first).')
            if self.sim_blocks is None:
                from ugv_nav.vision_sim import SimBlocks
                self.sim_blocks = SimBlocks()
                self.session.backend.blocks = self.sim_blocks
            for block_id, (bx, by, _cls) in self.sim_blocks.items():
                if math.hypot(bx - x, by - y) < 0.06:
                    self.sim_blocks.remove(block_id)
                    return self._ok('Simulated block removed at (%.2f, %.2f).' % (bx, by))
            colour = self.opts['sim_block_colour']
            self.sim_blocks.add(x, y, '%s_obstacle' % colour)
            return self._ok('Simulated %s block placed at (%.2f, %.2f) (click it again to remove).' % (colour, x, y))

    def cmd_camera_sample(self, args):
        """Calibration sample: a block whose real centre is at map (x, y) is in view right now."""
        tx, ty = arg_number(args, 'x', -1, 100), arg_number(args, 'y', -1, 100)
        with self._lock:
            if not self.vision or not self.nav:
                return self._fail('Turn on Camera + YOLO and confirm the robot pose first.')
            pose = self.nav.snapshot()['pose']
            if pose is None:
                return self._fail('Confirm the robot pose first.')
            snap = self.vision.snapshot()
            if not snap['boxes']:
                return self._fail('YOLO does not see a block right now.')
            dx, dy = tx - pose[0], ty - pose[1]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            target = (c * dx + s * dy, -s * dx + c * dy)
            cam = snap['camera']
            fw, fh = snap['frame_size']
            sx, sy = cam.width / float(fw), cam.height / float(fh)
            box = min(snap['boxes'], key=lambda b: math.dist(b['robot'], target) if b.get('robot') else 99.0)['box']
            self.cam_samples.append(((box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy), target))
            self._bump('vision')
            return self._ok('Sample %d added (block %.2f m ahead, %.2f m left). Use 2-4 blocks at different '
                            'distances, then Fit.' % (len(self.cam_samples), target[0], target[1]))

    def cmd_camera_samples_clear(self, args):
        with self._lock:
            self.cam_samples = []
            return self._ok('Calibration samples cleared.')

    def cmd_camera_fit(self, args):
        from ugv_nav.vision import calibrate
        with self._lock:
            fit, rms = calibrate(self.cam_samples, self._camera_model())       # ValueError with < 2 samples
            self.opts.update(camera_pitch_deg=round(math.degrees(fit.pitch_rad), 1),
                             camera_forward_cm=round(fit.x_offset_m * 100, 1),
                             camera_yaw_deg=round(math.degrees(fit.yaw_offset_rad), 1))
            if self.vision:
                self.vision.set_camera(fit)
            self._bump('settings')
            self._settings_dirty = True
            return self._ok('Camera calibrated from %d samples: tilt %.1f deg, %.1f cm ahead, yaw %.1f deg. Average '
                            'position error %.1f cm.' % (len(self.cam_samples), math.degrees(fit.pitch_rad),
                                                        fit.x_offset_m * 100, math.degrees(fit.yaw_offset_rad),
                                                        rms * 100), 'success')

    # ------------------------------------------------------------------ dispatch + housekeeping
    def command(self, name, args=None):
        """Run one validated command. Never raises: returns dict(ok, message, level, ...)."""
        args = {} if args is None else args
        handler = getattr(self, 'cmd_' + name, None) if isinstance(name, str) and name.isidentifier() else None
        if handler is None or not isinstance(args, dict):
            return result(False, 'Unknown command.')
        try:
            return handler(args) or result(True)
        except ValueError as exc:
            self._say(str(exc), 'warn', log=False)
            return result(False, str(exc), 'warn')
        except Exception as exc:                  # a bug must never take the console down
            traceback.print_exc()
            self._say('Internal error in "%s": %s' % (name, exc), 'error')
            return result(False, 'Internal error: %s' % exc)

    def _collect_nav_log(self):
        nav = self.nav
        if nav is None:
            return
        count = nav.log_count
        new = count - self._nav_log_seen
        if new <= 0:
            return
        self._nav_log_seen = count
        for _, text in list(nav.log)[-min(new, len(nav.log)):]:
            level = ('success' if 'SURVIVOR' in text or 'SECURED' in text or 'MISSION COMPLETE' in text or text.startswith('Goal reached')
                     else 'warn' if text.startswith('Stopped') or 'PAUSED' in text or 'STOP' in text else 'info')
            self._log_line(level, text)

    def tick_once(self, dt=0.04):
        self._offline_step(dt)
        self._collect_nav_log()
        if (self.uav is not None or self.uav_video is not None) and self._lock.acquire(blocking=False):
            try:
                self._uav_tick()
            finally:
                self._lock.release()
        if self.recorder is not None and self._lock.acquire(blocking=False):
            try:
                self._record_housekeeping()
            finally:
                self._lock.release()
        nav = self.nav
        if nav is not None and nav.cfg.lidar_reference == 'room':
            if self._lock.acquire(blocking=False):
                try:
                    self._room_housekeeping()
                    if nav.found_seq != self._found_seen:       # found by itself: move the Start marker along
                        self._found_seen = nav.found_seq
                        self._follow_found()
                finally:
                    self._lock.release()

    def _tick_loop(self):
        last = time.monotonic()
        save_at = last
        while not self._closing.wait(0.04):
            now = time.monotonic()
            try:
                self.tick_once(min(now - last, 0.1))
                if self._settings_dirty and now - save_at > 0.5:
                    save_at = now
                    self._save_settings()
            except Exception:
                traceback.print_exc()
            last = now
