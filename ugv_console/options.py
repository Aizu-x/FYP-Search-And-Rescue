"""Console settings: one table of every option with its type, range and default.

Everything that arrives from the web page (or from a saved settings file) goes through ``coerce``,
so the rest of the console only ever sees validated values.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ugv_planner.config import EXAMPLES

OFFLINE, SIM, ROBOT = 'offline', 'sim', 'robot'
MODES = (OFFLINE, SIM, ROBOT)
MODE_LABELS = {OFFLINE: 'Planner', SIM: 'Simulator', ROBOT: 'UGV'}
# names used by the previous (Tk) console's settings file
OLD_MODE_NAMES = {'Offline simulation (original)': OFFLINE, 'Closed-loop simulated UGV': SIM,
                  'Physical MentorPi': ROBOT}


@dataclass(frozen=True)
class Opt:
    kind: str                      # bool | int | float | optfloat | str | choice
    default: object
    lo: float = None
    hi: float = None
    choices: tuple = ()
    label: str = ''


SCHEMA = {
    'mode': Opt('choice', OFFLINE, choices=MODES, label='Mode'),
    'env': Opt('choice', 'ENV1', choices=tuple(EXAMPLES), label='Environment'),
    # planner / footprint
    'connectivity': Opt('choice', 8, choices=(4, 8), label='Movement'),
    'radius': Opt('optfloat', None, 0.0, 1.0, label='Radius (m)'),
    'margin': Opt('float', 0.0, 0.0, 0.5, label='Safety margin (m)'),
    'verified': Opt('bool', False, label='Measured radius verified'),
    'inflation': Opt('bool', True, label='Use inflation'),
    'clearance_weight': Opt('float', 4.0, 0.0, 100.0, label='Clearance weight'),
    'extra_clearance': Opt('float', 0.20, 0.005, 1.0, label='Preferred extra clearance (m)'),
    'smooth_path': Opt('bool', True, label='Smooth route'),
    # connection / pose
    'host': Opt('str', '', label='UGV IP'),
    'port': Opt('int', 8765, 1, 65535, label='UGV port'),
    'heading': Opt('float', 0.0, -360.0, 360.0, label='Heading (deg)'),
    'auto_continue': Opt('bool', False, label='Auto-continue after replan'),
    'allow_tight': Opt('bool', False, label='Allow radius below turning circle'),
    'ignore_drift': Opt('bool', False, label='Ignore drift alerts'),
    'lidar_correction': Opt('bool', True, label='Live LiDAR tracking'),
    'lidar_sees': Opt('choice', 'room', choices=('room', 'walls'), label='The LiDAR sees'),
    'auto_find': Opt('bool', True, label='Find the robot automatically'),
    'corner_gap_cm': Opt('float', 10.0, 0.0, 40.0, label='Corner start: distance from the walls (cm)'),
    # team simulator (team_console.py): 1 = the single-robot simulator, 2-4 = a team
    'sim_robots': Opt('int', 1, 1, 4, label='Robots in the simulator'),
    'team_scan': Opt('bool', True, label='Team: look around at the goal'),
    'team_approach': Opt('bool', True, label='Team: drive up to survivors found'),
    'brush': Opt('int', 1, 0, 10, label='Brush radius (cells)'),
    'scale_x': Opt('float', 1.0, 0.7, 1.3, label='X scale'),
    'scale_y': Opt('float', 1.0, 0.7, 1.3, label='Y scale'),
    'robot_speed': Opt('int', 60, 10, 100, label='Robot speed (%)'),
    'sim_speed': Opt('float', 0.25, 0.05, 1.0, label='Playback speed (m/s)'),
    # camera
    'camera_on': Opt('bool', False, label='Camera + YOLO'),
    'camera_height_cm': Opt('float', 14.5, 2.0, 60.0, label='Camera height (cm)'),
    'camera_pitch_deg': Opt('float', 10.0, -10.0, 60.0, label='Camera tilt (deg)'),
    'camera_forward_cm': Opt('float', 8.0, -20.0, 30.0, label='Camera ahead of centre (cm)'),
    'camera_yaw_deg': Opt('float', 0.0, -30.0, 30.0, label='Camera yaw (deg)'),
    'auto_tilt': Opt('bool', True, label='Auto tilt from block size'),
    'shape_filter': Opt('bool', True, label='Ignore boxes that cannot be a block'),
    'colour_check': Opt('bool', True, label='Box must contain its colour (green / red)'),
    'yolo_min_conf': Opt('float', 60.0, 30.0, 95.0, label='Minimum YOLO confidence (%)'),
    'sim_block_colour': Opt('choice', 'green', choices=('green', 'red'), label='Test block colour'),
    # survivor search
    'search_at_goal': Opt('bool', False, label='Search for survivors at the goal'),
    'record_mission': Opt('bool', True, label='Record every mission on video'),
    'record_overlay': Opt('bool', True, label='Draw detections and mission info on the video'),
    'record_dir': Opt('str', '', label='Video folder (empty = Videos\\UGV missions)'),
    'mission_gap_cm': Opt('float', 25.0, 5.0, 80.0, label='Stop in front of survivor (cm)'),
    'mission_hold_s': Opt('float', 3.0, 0.5, 30.0, label='Stand in front before "secured" (s)'),
    'mission_end': Opt('bool', True, label='End the mission when the survivor is secured'),
    'mission_same_cm': Opt('float', 30.0, 5.0, 60.0, label='Same survivor if closer than (cm)'),
    'mission_step_deg': Opt('float', 45.0, 20.0, 120.0, label='Look-around step (deg)'),
    'mission_dwell_s': Opt('float', 1.5, 0.3, 5.0, label='Hold still at each step (s)'),
    'mission_turn_rate': Opt('float', 0.30, 0.15, 0.6, label='Look-around turn speed (rad/s)'),
    'mission_range_m': Opt('float', 1.2, 0.4, 3.0, label='Camera search range (m)'),
    # UAV section (Tello drone)
    'section': Opt('choice', 'ugv', choices=('ugv', 'uav'), label='Section'),
    'uav_sim': Opt('bool', False, label='Simulated drone'),
    'uav_ip': Opt('str', '192.168.10.1', label='Drone IP'),
    'uav_speed': Opt('int', 35, 10, 100, label='Stick strength (%)'),
    'uav_mode': Opt('choice', 'manual', choices=('manual', 'plan'), label='Flight mode'),
    'uav_record': Opt('bool', True, label='Record every flight on video'),
    'uav_dir': Opt('str', '', label='Footage folder (empty = Videos\\UAV footage)'),
    # display
    'show_path': Opt('bool', True), 'show_gt': Opt('bool', False), 'show_search': Opt('bool', False),
    'show_seen': Opt('bool', True), 'show_inflation': Opt('bool', True), 'show_scan': Opt('bool', True),
    'show_truth': Opt('bool', True), 'show_fov': Opt('bool', True), 'show_zone': Opt('bool', True),
    'show_trail': Opt('bool', True), 'show_grid': Opt('bool', True), 'show_room': Opt('bool', True),
    'theme': Opt('choice', 'dark', choices=('dark', 'light'), label='Theme'),
}

PLANNER_KEYS = ('connectivity', 'radius', 'margin', 'verified', 'inflation', 'clearance_weight', 'extra_clearance',
                'smooth_path', 'show_search')
MISSION_KEYS = ('mission_gap_cm', 'mission_hold_s', 'mission_end', 'mission_same_cm', 'mission_step_deg', 'mission_dwell_s', 'mission_turn_rate',
                'mission_range_m')
CAMERA_MOUNT_KEYS = ('camera_height_cm', 'camera_pitch_deg', 'camera_forward_cm', 'camera_yaw_deg')
# old settings file key -> new key
OLD_KEYS = {'nav_mode': 'mode', 'speed': 'sim_speed'}


def defaults():
    return {key: opt.default for key, opt in SCHEMA.items()}


def coerce(key, value):
    """Validated value for option ``key`` or ValueError with a message the operator can read."""
    if key not in SCHEMA:
        raise ValueError('Unknown setting "%s".' % key)
    opt = SCHEMA[key]
    name = opt.label or key
    if opt.kind == 'bool':
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) and value in (0, 1):
            return bool(value)
        if isinstance(value, str) and value.lower() in ('true', 'false', '0', '1'):
            return value.lower() in ('true', '1')
        raise ValueError('%s must be on or off.' % name)
    if opt.kind == 'str':
        if not isinstance(value, str) or len(value) > 200 or any(ord(ch) < 32 for ch in value):
            raise ValueError('%s must be a short text.' % name)
        return value.strip()
    if opt.kind == 'choice':
        if key == 'mode' and value in OLD_MODE_NAMES:
            value = OLD_MODE_NAMES[value]
        for choice in opt.choices:
            if value == choice or (not isinstance(value, bool) and str(value) == str(choice)):
                return choice
        raise ValueError('%s must be one of: %s.' % (name, ', '.join(str(c) for c in opt.choices)))
    if opt.kind == 'optfloat' and (value is None or (isinstance(value, str) and not value.strip())):
        return None
    if isinstance(value, bool):
        raise ValueError('%s must be a number.' % name)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError('%s must be a number.' % name)
    if not math.isfinite(number):
        raise ValueError('%s must be a finite number.' % name)
    if opt.kind == 'int':
        if abs(number - round(number)) > 1e-9:
            raise ValueError('%s must be a whole number.' % name)
        number = int(round(number))
    if (opt.lo is not None and number < opt.lo) or (opt.hi is not None and number > opt.hi):
        raise ValueError('%s must be between %g and %g.' % (name, opt.lo, opt.hi))
    return number


def from_saved(saved):
    """Options from a settings file (new or old console format). Unknown / invalid entries are skipped."""
    out = {}
    for key, value in (saved or {}).items():
        key = OLD_KEYS.get(key, key)
        if key in SCHEMA:
            try:
                out[key] = coerce(key, value)
            except ValueError:
                pass
    return out


# ---------------------------------------------------------------------------- command arguments
def arg_number(args, key, lo=None, hi=None, default=None, required=True):
    value = args.get(key, default)
    if value is None:
        if required:
            raise ValueError('Missing value "%s".' % key)
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError('"%s" must be a number.' % key)
    try:
        number = float(value)
    except ValueError:
        raise ValueError('"%s" must be a number.' % key)
    if not math.isfinite(number) or (lo is not None and number < lo) or (hi is not None and number > hi):
        raise ValueError('"%s" is out of range.' % key)
    return number


def arg_bool(args, key, default=False):
    value = args.get(key, default)
    if not isinstance(value, bool):
        raise ValueError('"%s" must be true or false.' % key)
    return value


def arg_choice(args, key, choices, default=None):
    value = args.get(key, default)
    if value not in choices:
        raise ValueError('"%s" must be one of: %s.' % (key, ', '.join(str(c) for c in choices)))
    return value


def arg_points(args, key, limit=4000, bound=100.0):
    value = args.get(key)
    if not isinstance(value, list) or len(value) > limit:
        raise ValueError('"%s" must be a list of at most %d points.' % (key, limit))
    out = []
    for p in value:
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                       or abs(v) > bound for v in p)):
            raise ValueError('"%s" contains an invalid point.' % key)
        out.append((float(p[0]), float(p[1])))
    return out
