"""What the console shows: JSON-ready "views" of the state, built from the same objects the old GUI drew.

``fast()``  -> tiny sample for smooth motion (pose, state, command), sent ~20 times a second.
``view(name)`` -> one section (settings, map, plan, nav, scan, vision, mission, seen, trail, log,
offline, diag, static). The server sends a section only when its content changed.

Nothing here waits for the navigator while it is busy: displays use non-blocking snapshots.
"""
from __future__ import annotations

import base64
import math
import time

import numpy as np

from ugv_nav.localization import transform_point
from ugv_nav.navigator import PLANNED, RUNNING

from . import options as O
from .options import OFFLINE, SIM

SECTIONS = ('static', 'settings', 'map', 'plan', 'nav', 'scan', 'vision', 'mission', 'seen', 'trail', 'log',
            'offline', 'diag', 'room', 'recording', 'uav', 'uavplan')


def pack_mask(mask):
    """Boolean grid (row 0 = bottom, as in ugv_planner) -> base64 of bits, row-major, most significant bit first."""
    return base64.b64encode(np.packbits(np.asarray(mask, bool), axis=None).tobytes()).decode('ascii')


def _decimate(points, limit):
    if len(points) <= limit:
        return points
    step = int(math.ceil(len(points) / float(limit)))
    out = points[::step]
    if out[-1] is not points[-1]:
        out.append(points[-1])
    return out


class Views:
    # ------------------------------------------------------------------ JSON hygiene
    @staticmethod
    def clean(obj):
        """Plain JSON types only; NaN/inf become null; floats are rounded (smaller messages)."""
        if isinstance(obj, dict):
            return {str(k): Views.clean(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple, set)):
            return [Views.clean(v) for v in obj]
        if isinstance(obj, (bool, np.bool_)):
            return bool(obj)
        if isinstance(obj, (int, np.integer)):
            return int(obj)
        if isinstance(obj, (float, np.floating)):
            value = float(obj)
            return round(value, 5) if math.isfinite(value) else None
        if obj is None or isinstance(obj, str):
            return obj
        return str(obj)

    # ------------------------------------------------------------------ fast sample (~20 Hz)
    def fast(self):
        out = dict(t=round(time.monotonic(), 4), mode=self.opts['mode'])
        nav = self.nav
        if nav is not None:
            sample = nav.fast_state()
            if sample:
                out['state'] = sample['state']
                if sample['pose'] is not None:
                    out['pose'] = [round(v, 4) for v in sample['pose']]
                    out['pt'] = round(sample['t'], 4)
                    out['sigma'] = round(sample['sigma'], 4)
                if sample['cmd'] is not None:
                    out['cmd'] = [round(sample['cmd'][0], 3), round(sample['cmd'][1], 3)]
            session = self.session
            if session is not None:
                out['truth'] = [round(v, 4) for v in session.backend.truth_pose()]
        elif self.opts['mode'] == OFFLINE:
            sim = self.sim
            position = sim.position
            if position is not None:
                route, k = sim.route, min(sim.index, max(len(sim.route) - 2, 0))
                yaw = (math.atan2(route[k + 1][1] - route[k][1], route[k + 1][0] - route[k][0])
                       if len(route) > 1 else math.radians(self.opts['heading']))
                out['pose'] = [round(position[0], 4), round(position[1], 4), round(yaw, 4)]
                out['pt'] = out['t']
                out['state'] = sim.state
        return out

    # ------------------------------------------------------------------ sections
    def view(self, name):
        return getattr(self, '_view_' + name)()

    def _view_static(self):
        return dict(version=self.VERSION, envs=list(O.SCHEMA['env'].choices),
                    modes=[dict(id=m, label=O.MODE_LABELS[m]) for m in O.MODES],
                    schema={k: dict(kind=v.kind, lo=v.lo, hi=v.hi, choices=list(v.choices), label=v.label)
                            for k, v in O.SCHEMA.items()},
                    config_error=self.config_error)

    def _view_settings(self):
        o = self.opts
        pct = int(o['robot_speed'])
        full_v, full_w = self._full_speed()
        half_l, half_w = self.robot_half_size()
        safety = self._safety_config()
        try:
            radius = self.planner_config().planning_radius
            planner_error = ''
        except ValueError as exc:
            radius, planner_error = None, str(exc)
        turning = math.hypot(half_l, half_w) + safety.rotate_clearance_m
        cam = None
        if self.vision is not None:
            c = self.vision.snapshot()['camera']
            cam = dict(fov_deg=round(2 * math.degrees(math.atan(c.width / 2.0 / c.fx)), 1), x_offset=c.x_offset_m,
                       range=self.vision.tracker.cfg.max_range_m)
        return self.clean(dict(
            opts=dict(o), connected=self.nav is not None, simulator=self.session is not None,
            speed=dict(pct=pct, v=full_v * pct / 100.0, w=full_w * pct / 100.0, full_v=full_v, full_w=full_w),
            robot=dict(half_length=half_l, half_width=half_w, turning=turning),
            planning_radius=radius, planner_error=planner_error,
            tight=radius is not None and radius + 1e-9 < turning,
            calibrating=None if not self.cal else self.cal['axis'], camera=cam,
            map_status=dict(source=self.map_source, unsaved=self.map_unsaved, env=o['env']),
            field=self._field_view(), dock=None if not self.dock else self.dock['corner'],
            lidar=dict(mount_cm=[round(self.lidar_mount[0] * 100, 1), round(self.lidar_mount[1] * 100, 1)],
                       mount_deg=round(math.degrees(self.lidar_mount[2]), 1), mount_ok=self.lidar_mount_ok,
                       room_saved=self._room_path().is_file())))

    def _field_view(self):
        """Size of the field inside the walls: what the data file says, and what is used (real size)."""
        try:
            w, h = self._map_inner()
        except (ValueError, OSError):
            return None
        sx, sy = self._scale()
        measured = self.field_measured or (None, None)
        return dict(map_w=round(w, 3), map_h=round(h, 3), real_w=round(w * sx, 3), real_h=round(h * sy, 3),
                    scaled=(sx, sy) != (1.0, 1.0), corners=[s['corner'] for s in self.field_samples],
                    measured_w=measured[0], measured_h=measured[1])

    def _view_map(self):
        grid, nav = self.grid, self.nav
        plan_grid = nav.grid if nav is not None and nav.grid.shape == grid.shape else grid
        try:
            cfg = self.planner_config()
            radius = cfg.planning_radius
        except ValueError:
            cfg, radius = None, None
        lidar = nav.lidar_mask if nav is not None and nav.lidar_mask.shape == grid.shape else None
        key = (id(grid), id(plan_grid), radius, self.map_source, self.map_unsaved, id(self.gt),
               -1 if nav is None else nav.map_rev)
        cached = self._view_cache.get('map')
        if cached and cached[0] == key:
            return cached[1]
        cells = grid.cells.astype(bool)
        inflated = ''
        if cfg is not None:
            blocked = self.planner.cache.fields(plan_grid, cfg).blocked
            inflated = pack_mask(blocked & ~plan_grid.cells.astype(bool))
        h, w = grid.shape
        data = dict(env=self.opts['env'], w=w, h=h, res=grid.resolution, origin=list(grid.origin),
                    cells=pack_mask(cells), inflated=inflated, gt=pack_mask(self.gt.cells),
                    lidar=pack_mask(lidar & ~cells) if lidar is not None and lidar.any() else '',
                    lidar_count=int((lidar & ~cells).sum()) if lidar is not None else 0,
                    source=self.map_source, unsaved=self.map_unsaved,
                    occupied=int(cells.sum()))
        self._view_cache['map'] = (key, data)
        return data

    def _view_plan(self):
        o = self.opts
        start = self.pending_pose or self.start_point
        data = dict(start=[round(start[0], 4), round(start[1], 4), o['heading']], path=[], goal=None, unsafe=[],
                    info={}, segment=None, pending_goal=False)
        if self.nav_active():
            nav = self.nav
            if nav is not None:
                snap = nav.snapshot(blocking=False)
                data['path'] = [[round(p[0], 4), round(p[1], 4)] for p in snap['path']]
                goal = snap['goal']
                if goal is None:                   # nothing planned yet: show the remembered goal dimmed
                    goal, data['pending_goal'] = self.goal_point, True
                data['goal'] = [round(goal[0], 4), round(goal[1], 4)] if goal is not None else None
                data['info'] = snap['plan_info']
                data['segment'] = snap['segment'] if snap['state'] == RUNNING and snap['motion'] == 'path' else None
            elif self.goal_point is not None:
                data['goal'] = [round(self.goal_point[0], 4), round(self.goal_point[1], 4)]
                data['pending_goal'] = True
        else:
            res = self.result
            if self.goal_point is not None:
                data['goal'] = [round(self.goal_point[0], 4), round(self.goal_point[1], 4)]
            if res is not None and res.found:
                data['path'] = [[round(p[0], 4), round(p[1], 4)] for p in res.waypoints]
                data['unsafe'] = list(self.report.get('unsafe_segments') or []) if self.report else []
                data['info'] = dict(found=True, length_m=round(res.distance_m, 3), points=len(res.waypoints),
                                    astar_ms=round(res.computation_ms, 1), expanded=res.expanded_nodes,
                                    smoothed=bool(self.report and self.report.get('smoothed')), status=res.status)
            elif res is not None:
                data['info'] = dict(found=False, status=res.status, message=res.message)
        return self.clean(data)

    def _authorize_view(self, snap, mission):
        o = self.opts
        if mission and mission['active'] and mission['paused']:
            return dict(enabled=True, action='resume', label='RESUME MISSION',
                        hint='Mission paused: %s' % mission['pause_reason'])
        label = 'AUTHORIZE + SEARCH' if o['search_at_goal'] else 'AUTHORIZE'
        state = snap['state']
        if state == PLANNED and snap['path']:
            if snap['auto_pending']:
                return dict(enabled=False, action='drive', label=label, hint='Auto-continue is about to start.')
            return dict(enabled=True, action='drive', label=label, hint='Route ready. Check that the area is clear.')
        if snap['localization'][0] != 'OK':
            hint = 'Step 2: place the robot and press Confirm pose (%s).' % snap['localization'][1]
        elif state == RUNNING:
            hint = 'The robot is moving. STOP (Esc) stops it at any time.'
        elif snap['goal'] is None:
            hint = 'Step 3: choose the Goal tool and click the map.'
        elif state == 'ARRIVED':
            hint = 'Goal reached. Set a new goal to drive again.'
        else:
            hint = 'No route ready: press Plan (or set the goal again).'
        return dict(enabled=False, action='drive', label=label, hint=hint)

    def _view_nav(self):
        nav, client = self.nav, self.client
        if nav is None or client is None:
            return dict(connected=False, link=dict(state='DISCONNECTED', ok=False),
                        authorize=dict(enabled=False, action='drive', label='AUTHORIZE',
                                       hint='Planner mode: no robot.' if not self.nav_active() else
                                       'Step 1: press %s.' % ('Start simulator' if self.opts['mode'] == SIM else 'Connect')),
                        notice=self.notice)
        snap = nav.snapshot(blocking=False)
        link = client.snapshot()
        tel = link['telemetry'] or {}
        mission = self.mission.snapshot(blocking=False) if self.mission is not None else None
        cmd = snap['command']
        return self.clean(dict(
            connected=True, state=snap['state'], message=snap['message'], notice=self.notice,
            loc=list(snap['localization']), sigma=snap['sigma'], max_sigma=snap['max_sigma'],
            ignore_drift=bool(nav.estimator.cfg.ignore_drift),
            estop=snap['estop'], auto=dict(on=nav.cfg.auto_continue_after_replan, used=snap['auto_replans'],
                                           max=nav.cfg.max_auto_replans, pending=snap['auto_pending']),
            motion=snap['motion'], stop_kind=snap['stop_kind'], reasons=snap['reasons'],
            cmd=None if cmd is None or snap['state'] != RUNNING else [cmd.vx, cmd.wz],
            zone=snap['stop_zone_m'], scale=list(snap['scale']), anchor=snap['anchor'],
            goal=snap['goal'], has_path=bool(snap['path']),
            room=snap.get('room'), searching=bool(snap.get('searching')),
            found=None if not snap.get('found') else [
                dict(x=c['pose'][0], y=c['pose'][1], heading=math.degrees(c['pose'][2]), fit=c['matched'])
                for c in snap['found']['candidates']],
            link=dict(state=link['state'], ok=link['link_ok'], rtt_ms=None if link['rtt'] is None else link['rtt'] * 1000,
                      armed=bool(tel.get('armed')), safety_stop=tel.get('safety_stop') or '',
                      error=(link['error'] or '')[:120], age=link['telemetry_age']),
            authorize=self._authorize_view(snap, mission)))

    def _view_scan(self):
        nav, client = self.nav, self.client
        if nav is None or client is None or not self.opts['show_scan']:
            return dict(pts=[], unknown='')
        sample = nav.fast_state() or {}
        pose = sample.get('pose')
        telemetry = client.snapshot()['telemetry'] or {}
        if pose is None or not telemetry.get('scan'):
            return dict(pts=[], unknown='')
        points = nav.safety.lidar_points(telemetry)          # robot frame, LiDAR mounting applied
        unknown = set(nav.safety.unknown_points(points, pose, nav.grid, sample.get('sigma', 0.0)))
        return dict(pts=[[round(v, 3) for v in transform_point(pose, p)] for p in points],
                    unknown=''.join('1' if p in unknown else '0' for p in points))

    def _view_vision(self):
        v, nav = self.vision, self.nav
        data = dict(on=v is not None, samples=len(self.cam_samples), pins=[], blocks=[], status='off', fps=0.0,
                    frame_id=None, tilt=None, events=[], boxes=0, ignored=0)
        if v is not None:
            snap = v.snapshot()
            data.update(status=snap['status'], fps=round(snap['fps'], 1), frame_id=snap['frame_id'], tilt=snap['tilt'],
                        events=[text for _, text in snap['events'][-4:]], boxes=len(snap['boxes']),
                        ignored=snap['ignored_shape'], camera_deg=round(math.degrees(snap['camera'].pitch_rad), 1))
        if nav is not None:
            data['pins'] = [dict(id=p.get('id'), cls=p.get('cls', ''), x=p['x'], y=p['y'], size=p.get('size', 0.05),
                                 conf=p.get('conf')) for p in list(nav.vision_obstacles)]
        if self.session is not None and self.sim_blocks is not None:
            data['blocks'] = [dict(id=i, x=x, y=y, cls=cls) for i, (x, y, cls) in self.sim_blocks.items()]
        return self.clean(data)

    def _view_mission(self):
        mission = self.mission
        if mission is None:
            return dict(active=False, paused=False, complete=False, phase='OFF', survivors=[], elapsed=0, coverage=0, passes=1,
                        pause_reason='')
        snap = dict(mission.snapshot(blocking=False))
        snap.pop('seen', None)
        snap.pop('events', None)
        return self.clean(snap)

    def _view_seen(self):
        mission = self.mission
        if mission is None or mission.seen is None or not self.opts['show_seen'] or mission.seen.shape != self.grid.shape:
            return dict(mask='', count=0)
        seen = mission.seen & ~self.grid.cells.astype(bool)
        count = int(seen.sum())
        cached = self._view_cache.get('seen')
        if cached and cached[0] == count:
            return cached[1]
        data = dict(mask=pack_mask(seen) if count else '', count=count)
        self._view_cache['seen'] = (count, data)
        return data

    def _view_trail(self):
        nav = self.nav
        data = dict(est=[], truth=[])
        if nav is not None:
            data['est'] = [[round(p[0], 3), round(p[1], 3)] for p in _decimate(list(nav.trajectory), 700)]
            if self.session is not None:
                history = list(self.session.backend.truth_history[-6000:])
                data['truth'] = [[round(p[0], 3), round(p[1], 3)] for p in _decimate(history, 500)]
        elif self.opts['mode'] == OFFLINE:
            data['est'] = [[round(p[0], 3), round(p[1], 3)] for p in _decimate(list(self.sim.history), 700)]
        return data

    def _view_log(self):
        return dict(lines=self.log_lines(150))

    def _view_offline(self):
        if self.nav_active() or self.result is None:
            return dict(active=not self.nav_active(), report=None, search='', sim_state=self.sim.state)
        res = self.result
        search = ''
        if self.opts['show_search'] and res.search_cells:
            mask = np.zeros(self.grid.shape, bool)
            cells = np.asarray(res.search_cells)
            mask[cells[:, 0], cells[:, 1]] = True
            search = pack_mask(mask)
        cfg = self.active_config
        return self.clean(dict(
            active=True, report=self.report, search=search, sim_state=self.sim.state,
            dispatch_allowed=self.dispatch_allowed, dispatch_message=self.dispatch_message,
            weight=cfg.clearance_weight, extra_cm=cfg.preferred_clearance_m * 100, message=res.message,
            edited=self.edited))

    def _view_room(self):
        """What the LiDAR has drawn of the room around the arena (wall cells, map coordinates)."""
        nav = self.nav
        if nav is None or nav.room is None or nav.cfg.lidar_reference != 'room' or not self.opts['show_room']:
            return dict(pts=[])
        room = nav.room.map
        key = (id(room), room.rev)
        cached = self._view_cache.get('room')
        if cached and cached[0] == key:
            return cached[1]
        data = dict(pts=[] if room.empty else room.wall_points(), res=room.res)
        self._view_cache['room'] = (key, data)
        return data

    def _view_recording(self):
        """Mission video: is one being recorded, where videos go, and the newest saved one."""
        recorder = self.recorder
        active = recorder is not None and not recorder.done
        status = recorder.status() if recorder is not None else {}
        return self.clean(dict(
            on=bool(self.opts['record_mission']), overlay=bool(self.opts['record_overlay']),
            folder=str(self._recording_folder()), active=active, stopping=bool(status.get('stopping')) and active,
            seconds=int(status.get('seconds', 0.0)), mb=status.get('mb', 0.0),
            waiting=bool(status.get('waiting')) and active, last=self.last_recording, note=self.recording_note,
            made=self.recordings_made))

    def _view_diag(self):
        nav, v = self.nav, self.vision
        data = dict(plan_ms=round(self.planner.last_ms, 1), cache_hits=self.planner.cache.hits)
        if nav is not None:
            data['loop'] = nav.loop_stats()
            data['match'] = nav.map_match()
            data['lidar_fix'] = nav.lidar_fix
            data['plan'] = nav.plan_info
        if v is not None:
            data['yolo_fps'] = round(v.stats.fps, 1)
        return self.clean(data)
