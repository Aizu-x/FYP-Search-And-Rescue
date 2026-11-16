"""The console backend (no browser): the operator workflow through validated commands, simulated UGV."""
import importlib.util
import json
import math
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from ugv_planner import Footprint, PlannerConfig
from ugv_planner.grid import dataset_manifest
from ugv_planner.interfaces import NavigationService
from ugv_console.app import ConsoleApp
from ugv_console.views import SECTIONS
from ugv_nav.path_processing import remove_collinear

from nav_helpers import wait_for

HAVE_CV2 = importlib.util.find_spec('cv2') is not None
ROBOT = dict(radius=0.143, margin=0.02, verified=True)


class ConsoleCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv console test ')
        self.dir = Path(self.tmp.name)
        self.apps = []
        self.app = self.make()

    def make(self, settings=False):
        app = ConsoleApp(settings_path=self.dir / 'console_settings.json' if settings else None,
                         edit_dir=self.dir / 'edited', export_dir=self.dir / 'exports')
        self.apps.append(app)
        return app

    def tearDown(self):
        for app in self.apps:
            app.close()
        self.tmp.cleanup()

    def cmd(self, name, app=None, **args):
        return (app or self.app).command(name, args)

    def ok(self, name, app=None, **args):
        r = self.cmd(name, app, **args)
        self.assertTrue(r['ok'], '%s -> %s' % (name, r['message']))
        return r

    def options(self, app=None, **values):
        return self.ok('set_options', app, values=values)

    def start_simulator(self, start=(1.215, 0.405), heading=0.0, **options):
        self.ok('set_mode', mode='sim')
        self.options(**dict(ROBOT, **options))
        self.ok('set_start', x=start[0], y=start[1], heading=heading)
        self.ok('connect')
        self.assertTrue(wait_for(lambda: self.app.client.link_ok(), timeout=6))
        time.sleep(0.3)
        self.ok('confirm_pose')

    def authorize(self, action='drive'):
        r = self.ok('authorize_prepare', action=action)
        return self.cmd('authorize_confirm', nonce=r['nonce'])


class PlannerModeTests(ConsoleCase):
    def test_planner_mode_gives_the_original_route_when_smoothing_is_off(self):
        app = self.app
        self.assertEqual(app.opts['mode'], 'offline')
        self.assertIsNone(app.goal_point)                                       # no goal until the operator sets one
        self.assertIsNone(app.result)
        self.assertIsNone(app.view('plan')['goal'])
        self.assertEqual(app.view('plan')['path'], [])
        self.assertFalse(self.cmd('sim_control', action='start')['ok'])
        self.ok('apply_coordinates', sx=0.345, sy=0.315, gx=None, gy=None)      # typed start only: still no goal
        self.assertIsNone(app.goal_point)
        self.options(smooth_path=False)
        self.assertIsNone(app.result)
        self.ok('set_goal', x=2.655, y=2.595)
        self.assertTrue(app.result.found)
        reference = NavigationService(app.grid, app.start_point, app.goal_point, app.planner_config()).replan()
        self.assertEqual(app.result.waypoints, reference.waypoints)            # exactly the ugv_planner route
        self.assertAlmostEqual(app.report['path_length_m'], reference.distance_m)
        self.ok('set_goal', x=1.395, y=1.335)
        self.assertEqual(app.goal_point, app.grid.cell_to_world(app.grid.world_to_cell((1.395, 1.335))))
        raw = len(remove_collinear(app.result.waypoints))
        self.options(smooth_path=True)
        self.assertTrue(app.report['smoothed'])
        self.assertLessEqual(len(app.result.waypoints), raw)
        self.assertLessEqual(app.result.distance_m, app.report['raw_path_length_m'] + 1e-9)
        self.assertIsNone(app.nav)

    def test_playback_follows_the_route_and_export_writes_the_original_files(self):
        app = self.app
        self.ok('set_goal', x=1.395, y=1.335)
        self.options(sim_speed=1.0)
        self.ok('sim_control', action='start')
        self.assertTrue(wait_for(lambda: app.sim.state == 'arrived', timeout=15))
        self.assertLess(math.dist(app.sim.position, app.goal_point), 1e-6)
        self.assertEqual(app.fast()['state'], 'arrived')
        folder = Path(self.ok('export_run')['folder'])
        for name in ('ENV1.json', 'ENV1.csv', 'ENV1_waypoints.csv', 'ENV1.png', 'ENV1_executed_positions.csv'):
            self.assertTrue((folder / name).is_file(), name)
        self.assertTrue(json.loads((folder / 'ENV1.json').read_text())['metrics']['path_found'])

    def test_invalid_settings_are_refused_and_nothing_changes(self):
        before = dict(self.app.opts)
        for values in (dict(extra_clearance=0), dict(radius=-1), dict(connectivity=6), dict(robot_speed=500),
                       dict(nonsense=1), dict(radius='abc'), dict(verified=True, radius=None), dict(mode='sim')):
            r = self.cmd('set_options', values=values)
            self.assertFalse(r['ok'], values)
            self.assertEqual(self.app.opts, before, values)
        self.assertFalse(self.cmd('set_goal', x=float('nan'), y=1.0)['ok'])
        self.assertFalse(self.cmd('set_goal', x=9.0, y=1.0)['ok'])              # outside the map
        self.assertFalse(self.cmd('no_such_command')['ok'])
        self.assertFalse(self.cmd('_load_env')['ok'])                           # private methods are not commands
        self.assertFalse(self.app.command('set_goal', 'not a dict')['ok'])

    def test_every_view_is_plain_json(self):
        self.ok('set_goal', x=1.395, y=1.335)
        self.options(show_search=True)
        for name in SECTIONS:
            json.dumps(self.app.view(name), allow_nan=False)
        self.assertGreater(len(self.app.view('offline')['search']), 100)        # expanded cells overlay
        self.assertEqual(self.app.view('map')['w'], 100)
        json.dumps(self.app.fast(), allow_nan=False)


class MapAndSettingsTests(ConsoleCase):
    def stroke(self, points, value, app=None, brush=1):
        return self.ok('paint', app, points=[list(p) for p in points], brush=brush, value=value, end=True)

    def test_paint_save_reload_and_revert_map(self):
        manifest = dataset_manifest()
        app = self.app
        original = app.grid.cells.copy()
        self.stroke([(1.5, 0.2 + 0.03 * i) for i in range(10)], 1)
        self.assertTrue(app.map_unsaved)
        self.assertGreater(int((app.grid.cells & ~original).sum()), 10)
        self.assertTrue(app.view('map')['unsaved'])
        self.stroke([(1.5, 0.2)], 0)                                           # erase part of it again
        self.assertFalse(app.grid.cells[app.grid.world_to_cell((1.5, 0.2))])
        edited = app.grid.cells.copy()
        need = self.cmd('set_env', env='ENV2')                                  # unsaved edits: asks first
        self.assertEqual(need.get('need_confirm'), 'discard_edits')
        self.assertEqual(app.opts['env'], 'ENV1')
        self.ok('save_map')
        self.assertFalse(app.map_unsaved)
        saved = self.dir / 'edited' / 'ENV1_occupancy_edited.npy'
        self.assertTrue(saved.is_file())
        self.assertGreater(json.loads(saved.with_suffix('.json').read_text())['added_obstacle_cells'], 0)
        self.assertEqual(dataset_manifest(), manifest)                          # original data files untouched
        app2 = self.make()                                                      # a new session loads the saved map
        self.assertEqual(app2.map_source, 'saved')
        self.assertTrue((app2.grid.cells == edited).all())
        self.assertEqual(self.cmd('revert_map', app2).get('need_confirm'), 'revert_map')
        self.ok('revert_map', app2, confirm=True)
        self.assertEqual(app2.map_source, 'original')
        self.assertTrue((app2.grid.cells == original).all())
        self.assertFalse(saved.exists())
        self.assertEqual(len(list((self.dir / 'edited').glob('*.bak.npy'))), 1)  # previous edits kept as backup

    def test_unsaved_edits_survive_program_exit_as_a_backup(self):
        self.stroke([(1.5, 0.3)], 1)
        self.app.close()
        self.assertEqual(len(list((self.dir / 'edited').glob('*.unsaved_*.bak.npy'))), 1)
        self.assertEqual(dataset_manifest(), dataset_manifest())

    def test_settings_remembered_between_sessions(self):
        app1 = self.make(settings=True)
        self.assertEqual(app1.opts['radius'], 0.143)                            # first run: robot values from navigation.toml
        self.options(app1, radius=0.15, margin=0.03, extra_clearance=0.25, host='192.168.149.1', auto_continue=True,
                     robot_speed=30, smooth_path=False, mission_gap_cm=30, theme='light')
        self.ok('set_env', app1, env='ENV2')
        self.ok('apply_scale', app1, x=1.04, y=0.985)
        self.assertFalse(self.cmd('apply_scale', app1, x=5, y=1)['ok'])         # absurd value refused
        self.assertEqual(self.cmd('set_options', app1, values=dict(allow_tight=True)).get('need_confirm'), 'allow_tight')
        self.assertFalse(app1.opts['allow_tight'])
        self.ok('set_options', app1, values=dict(allow_tight=True), confirm=True)
        self.assertIn('0.045 m/s', app1._apply_speed())                         # 30 % of 0.15 m/s
        app1.close()
        data = json.loads((self.dir / 'console_settings.json').read_text())
        self.assertEqual((data['radius'], data['env'], data['host']), (0.15, 'ENV2', '192.168.149.1'))
        app2 = self.make(settings=True)
        o = app2.opts
        self.assertEqual((o['radius'], o['margin'], o['extra_clearance'], o['env'], o['host']),
                         (0.15, 0.03, 0.25, 'ENV2', '192.168.149.1'))
        self.assertTrue(o['auto_continue'] and o['allow_tight'])
        self.assertEqual((o['scale_x'], o['scale_y'], o['robot_speed'], o['smooth_path'], o['theme']),
                         (1.04, 0.985, 30, False, 'light'))
        self.assertIsNone(app2.goal_point)                                      # the goal is never remembered / preset
        self.assertEqual(app2.planner_config().footprint.radius_m, 0.15)        # applied to planning

    def test_settings_of_the_previous_console_are_imported_once(self):
        old = dict(env='ENV2', connectivity='8', radius='0.15', margin='0.02', verified=True, inflation=True,
                   clearance_weight='4.0', extra_clearance='0.20', nav_mode='Physical MentorPi', host='192.168.149.1',
                   port='8765', heading='90', scale_x='1.040', scale_y='0.985', robot_speed=30, speed=0.5,
                   camera_pitch_deg='12.5', view_rotation=90, mission_gap_cm='25')
        (self.dir / 'gui_settings.json').write_text(json.dumps(old))
        app = self.make(settings=True)
        o = app.opts
        self.assertEqual((o['env'], o['radius'], o['mode'], o['host'], o['port'], o['heading']),
                         ('ENV2', 0.15, 'robot', '192.168.149.1', 8765, 90.0))
        self.assertEqual((o['scale_x'], o['robot_speed'], o['sim_speed'], o['camera_pitch_deg']), (1.04, 30, 0.5, 12.5))
        self.assertIsNone(app.nav)                                              # never connects by itself


class SimulatorWorkflowTests(ConsoleCase):
    def test_workflow_authorize_two_steps_and_stop(self):
        app = self.app
        self.start_simulator(auto_continue=False)
        self.assertEqual(app.nav.estimator.status(time.monotonic())[0], 'OK')
        self.assertFalse(app.view('nav')['authorize']['enabled'])               # nothing planned yet
        self.ok('set_goal', x=2.205, y=0.405)
        self.assertEqual(app.nav.state, 'PLANNED')
        plan = app.view('plan')
        self.assertEqual(plan['start'][:2], [1.215, 0.405])
        self.assertEqual(plan['goal'], [2.205, 0.405])
        self.ok('apply_coordinates', sx=1.215, sy=0.405, gx=1.2, gy=0.9)        # typed goal
        self.assertEqual(app.nav.goal, app.grid.cell_to_world(app.grid.world_to_cell((1.2, 0.9))))
        self.ok('set_goal', x=2.205, y=0.405)
        time.sleep(0.5)
        self.assertFalse(app.session.server.armed)                              # planning never moves the robot
        self.assertTrue(app.view('nav')['authorize']['enabled'])
        # a confirmation without / with a wrong / with a used code is refused
        self.assertFalse(self.cmd('authorize_confirm', nonce='')['ok'])
        prepared = self.ok('authorize_prepare', action='drive')
        self.assertIn('0.99 m', prepared['text'])
        self.assertFalse(self.cmd('authorize_confirm', nonce='0' * 16)['ok'])
        self.assertFalse(self.cmd('authorize_confirm', nonce=prepared['nonce'])['ok'])   # consumed by the wrong try
        self.assertEqual(app.nav.state, 'PLANNED')
        self.assertFalse(app.session.server.armed)
        # the route changes between question and answer: refused
        prepared = self.ok('authorize_prepare', action='drive')
        self.ok('set_goal', x=2.0, y=0.405)
        self.assertFalse(self.cmd('authorize_confirm', nonce=prepared['nonce'])['ok'])
        self.assertEqual(app.nav.state, 'PLANNED')
        # the real thing
        r = self.authorize()
        self.assertTrue(r['ok'], r['message'])
        self.assertEqual(app.nav.state, 'RUNNING')
        self.assertTrue(app.is_moving())
        self.assertTrue(wait_for(lambda: math.dist(app.session.backend.truth_pose()[:2], (1.215, 0.405)) > 0.05, timeout=5))
        fast = app.fast()
        self.assertEqual(fast['state'], 'RUNNING')
        self.assertEqual(len(fast['pose']), 3); self.assertEqual(len(fast['truth']), 3)
        self.ok('stop')
        self.assertEqual(app.nav.state, 'STOPPED')
        self.assertTrue(wait_for(lambda: app.session.server.output == (0.0, 0.0, 0.0), timeout=1))
        self.assertFalse(app.is_moving())
        app.tick_once()
        self.assertTrue(any('STOP' in line['text'] for line in app.view('log')['lines']))
        for name in SECTIONS:
            json.dumps(app.view(name), allow_nan=False)

    def test_paint_while_connected_replans_and_env_change_resets(self):
        app = self.app
        self.start_simulator()
        self.ok('set_goal', x=2.205, y=0.405)
        straight = list(app.nav.path)
        original = app.grid.cells.copy()
        wall = [[1.7, 0.15 + 0.04 * i] for i in range(12)]
        for k in range(0, 12, 4):                                               # a stroke arrives in batches
            self.ok('paint', points=wall[k:k + 4], brush=1, value=1, end=k == 8)
        painted = int((app.grid.cells & ~original).sum())
        self.assertGreater(painted, 20)
        self.assertTrue((app.nav.base_grid.cells == app.grid.cells).all())      # navigator has the edited map
        self.assertEqual(app.nav.state, 'PLANNED')                              # route kept: replanned around the wall
        self.assertNotEqual(app.nav.path, straight)
        self.ok('paint', points=wall, brush=1, value=0, end=True)
        self.assertTrue((app.grid.cells == original).all())
        self.assertEqual(app.nav.path, straight)
        self.options(margin=0.05)                                               # footprint change reaches the navigator
        self.assertAlmostEqual(app.nav.planner_config.footprint.margin_m, 0.05)
        self.assertEqual(app.nav.state, 'PLANNED')
        self.authorize()
        self.assertFalse(self.cmd('paint', points=wall[:2], brush=1, value=1, end=True)['ok'])   # not while driving
        self.ok('stop')
        self.ok('set_env', env='ENV2', confirm=True)                            # other world: simulator must restart
        self.assertIsNone(app.nav)
        self.assertEqual(app.opts['env'], 'ENV2')

    def test_lidar_found_obstacles_are_a_separate_layer(self):
        app = self.app
        self.start_simulator()
        nav = app.nav
        with nav._lock:                                                         # as if the LiDAR had mapped a box
            r, c = app.grid.world_to_cell((1.8, 0.405))
            nav.lidar_mask[r - 1:r + 2, c - 1:c + 2] = True
            nav._compose()
        view = app.view('map')
        self.assertEqual(view['lidar_count'], 9)
        self.assertFalse(app.grid.cells[r, c])                                  # not part of the operator map
        self.assertFalse(app.map_unsaved)
        self.ok('lidar_layer', action='clear')
        self.assertEqual(app.view('map')['lidar_count'], 0)
        with nav._lock:
            nav.lidar_mask[r - 1:r + 2, c - 1:c + 2] = True
            nav._compose()
        self.ok('lidar_layer', action='keep')
        self.assertTrue(app.grid.cells[r, c])                                   # now the operator's (unsaved) map
        self.assertTrue(app.map_unsaved)
        self.assertEqual(app.view('map')['lidar_count'], 0)

    def test_relocalize_and_scale_commands(self):
        app = self.app
        self.start_simulator()
        self.assertFalse(self.cmd('cal_finish', x=2.0, y=0.4)['ok'])            # no run started
        self.ok('cal_start', axis='x')
        self.assertEqual(app.view('settings')['calibrating'], 'x')
        self.ok('cal_cancel')
        self.ok('apply_scale', x=1.02, y=0.98)
        self.assertEqual(app.nav.estimator.scale_x, 1.02)
        self.ok('reset_scale')
        self.assertEqual((app.opts['scale_x'], app.nav.estimator.scale_y), (1.0, 1.0))
        # operator placed the robot 6 cm / 5 deg wrong: LiDAR re-localization fixes it
        app.nav.set_initial_pose((1.275, 0.375, math.radians(5)))
        self.assertLess(app.nav.map_match()['matched'], 0.9)
        r = self.ok('relocalize')
        self.assertIn('LiDAR', r['message'])
        pose = app.nav.snapshot()['pose']
        self.assertLess(math.dist(pose[:2], (1.215, 0.405)), 0.03)
        self.ok('sync_pose_sim')
        self.assertLess(math.dist(app.nav.snapshot()['pose'][:2], app.session.backend.truth_pose()[:2]), 1e-6)

    def test_relocalize_pressed_right_after_a_stop_waits_for_the_robot(self):
        app = self.app
        self.start_simulator(robot_speed=100)
        self.ok('set_goal', x=2.205, y=0.405)
        self.assertTrue(self.authorize()['ok'])
        self.assertTrue(wait_for(lambda: math.dist(app.session.backend.truth_pose()[:2], (1.215, 0.405)) > 0.15, timeout=8))
        self.ok('stop')                                                         # still rolling out ...
        r = self.cmd('relocalize')                                              # ... and the button is pressed at once
        self.assertTrue(r['ok'], r['message'])
        self.assertLess(math.dist(app.nav.snapshot()['pose'][:2], app.session.backend.truth_pose()[:2]), 0.03)
        self.assertEqual(app.nav.state, 'STOPPED')                              # STOP stays latched: no new route by itself
        self.assertFalse(app.session.server.armed)

    def test_robot_mode_needs_host_and_key(self):
        app = self.app
        self.ok('set_mode', mode='robot')
        self.options(host='')
        r = self.cmd('connect')
        self.assertFalse(r['ok']); self.assertIn('IP', r['message'])
        self.options(host='192.168.149.1')
        if not app.edge_cfg.key_path.is_file():
            r = self.cmd('connect')
            self.assertFalse(r['ok']); self.assertIn('key', r['message'])
        self.assertIsNone(app.nav)
        self.assertFalse(self.cmd('confirm_pose')['ok'])
        self.assertFalse(self.cmd('authorize_prepare', action='drive')['ok'])
        self.ok('stop')                                                         # STOP always answers


@unittest.skipUnless(HAVE_CV2, 'OpenCV not installed (pip install -r requirements-yolo.txt)')
class CameraAndMissionTests(ConsoleCase):
    def test_camera_pins_auto_tilt_and_manual_calibration(self):
        app = self.app
        # deliberately wrong mount (the simulated camera is 15 deg / 8 cm); auto tilt off first
        self.start_simulator(camera_pitch_deg=5, camera_forward_cm=2, auto_tilt=False)
        self.options(camera_on=True)
        self.assertIsNotNone(app.vision)
        self.options(sim_block_colour='red')
        self.ok('sim_block', x=1.60, y=0.47)
        self.assertTrue(wait_for(lambda: len(app.nav.snapshot()['vision_obstacles']) == 1, timeout=12))
        before = app.nav.snapshot()['vision_obstacles'][0]
        self.assertGreater(math.dist((before['x'], before['y']), (1.60, 0.47)), 0.05)      # wrong mount = wrong spot
        frame = app.vision.wait_frame(timeout=2)
        self.assertEqual(frame[1][:2], b'\xff\xd8')                             # JPEG for the live view
        self.assertTrue(wait_for(lambda: (app.view('vision')['tilt'] or {}).get('confident'), timeout=10))
        self.assertAlmostEqual(app.view('vision')['tilt']['deg'], 15.0, delta=1.0)
        self.ok('use_tilt_estimate')                                            # one click: tilt from block size
        self.assertAlmostEqual(app.opts['camera_pitch_deg'], 15.0, delta=1.0)
        # manual calibration also fixes the forward offset: three blocks at measured spots
        for bx, by in [(1.60, 0.405), (1.85, 0.30), (2.05, 0.52)]:
            for block_id, _ in app.sim_blocks.items():
                app.sim_blocks.remove(block_id)
            app.sim_blocks.add(bx, by, 'red_obstacle')
            time.sleep(0.3)
            self.assertTrue(wait_for(lambda: any(b.get('robot') for b in app.vision.snapshot()['boxes']), timeout=8))
            time.sleep(0.5)
            self.ok('camera_sample', x=bx, y=by)
        self.assertEqual(app.view('vision')['samples'], 3)
        self.ok('camera_fit')
        self.assertAlmostEqual(app.opts['camera_pitch_deg'], 15.0, delta=1.5)
        self.assertAlmostEqual(app.opts['camera_forward_cm'], 8.0, delta=2.5)
        self.ok('clear_camera_obstacles')
        self.assertTrue(wait_for(lambda: len(app.nav.snapshot()['vision_obstacles']) == 1, timeout=12))
        time.sleep(1.0)
        after = app.nav.snapshot()['vision_obstacles'][0]
        self.assertLess(math.dist((after['x'], after['y']), (2.05, 0.52)), 0.03)           # now within 3 cm
        self.assertEqual(len(app.view('vision')['pins']), 1)
        self.options(camera_on=False)
        self.assertIsNone(app.vision)

    def test_auto_tilt_is_saved_to_the_settings(self):
        app = self.app
        self.start_simulator(camera_pitch_deg=5, auto_tilt=True)
        self.options(camera_on=True, sim_block_colour='green')
        self.ok('sim_block', x=1.60, y=0.47)
        self.assertTrue(wait_for(lambda: abs(app.opts['camera_pitch_deg'] - 15.0) < 1.0, timeout=12))
        self.assertTrue(wait_for(lambda: len(app.nav.snapshot()['vision_obstacles']) == 1, timeout=8))
        pin = app.nav.snapshot()['vision_obstacles'][0]
        self.assertLess(math.dist((pin['x'], pin['y']), (1.60, 0.47)), 0.04)
        self.assertTrue(any('Camera tilt set' in line['text'] for line in app.view('log')['lines']))

    def test_search_mission_from_console(self):
        app = self.app
        self.start_simulator(camera_pitch_deg=15, camera_forward_cm=8, lidar_correction=True,
                             mission_step_deg=90, mission_dwell_s=0.8, mission_turn_rate=0.5, robot_speed=100)
        self.assertFalse(self.cmd('set_options', values=dict(mission_gap_cm=500))['ok'])   # absurd value refused
        self.ok('set_goal', x=1.815, y=0.405)
        self.options(search_at_goal=True)
        self.assertEqual(app.view('nav')['authorize']['label'], 'AUTHORIZE + SEARCH')
        r = self.cmd('authorize_prepare', action='drive')                       # camera is off: refused
        self.assertFalse(r['ok']); self.assertIn('Camera', r['message'])
        self.assertFalse(self.cmd('authorize_prepare', action='search_here')['ok'])
        self.assertFalse(app.mission.active)
        self.assertEqual(app.nav.state, 'PLANNED')
        self.options(camera_on=True, sim_block_colour='green')
        self.ok('sim_block', x=1.5, y=1.0)
        self.assertIn('SEARCH FOR SURVIVORS', self.ok('authorize_prepare', action='drive')['text'])
        r = self.authorize()
        self.assertTrue(r['ok'], r['message'])
        self.assertTrue(app.mission.active)
        self.assertTrue(wait_for(lambda: len(app.view('mission')['survivors']) == 1, timeout=60), app.nav.message)
        survivor = app.view('mission')['survivors'][0]
        self.assertEqual(survivor['id'], 1)
        self.assertLess(math.dist((survivor['x'], survivor['y']), (1.5, 1.0)), 0.05)
        time.sleep(0.3)
        self.assertTrue(any('SURVIVOR S1' in line['text'] and line['level'] == 'success'
                            for line in app.view('log')['lines']))
        self.assertGreater(len(app.view('seen')['mask']), 100)                  # searched-floor overlay
        self.ok('stop')                                                         # STOP pauses the mission
        self.assertTrue(app.mission.paused)
        nav = app.view('nav')
        self.assertEqual((nav['authorize']['label'], nav['authorize']['action']), ('RESUME MISSION', 'resume'))
        r = self.authorize('resume')
        self.assertTrue(r['ok'], r['message'])
        self.assertTrue(wait_for(lambda: app.nav.state == 'RUNNING', timeout=10), app.nav.message)
        self.ok('stop_mission')
        self.assertFalse(app.mission.active)
        self.assertEqual(self.cmd('clear_survivors').get('need_confirm'), 'clear_survivors')
        path = Path(self.ok('export_survivors')['path'])
        self.assertIn('S1', path.read_text())
        self.ok('clear_survivors', confirm=True)
        self.assertEqual(app.view('mission')['survivors'], [])


class RobotModeTests(unittest.TestCase):
    """Robot mode against a stand-in bridge on this computer (the same bridge program the robot runs,
    with its hardware-free backend). Proves the connection path; it is NOT a test on the real robot."""

    def setUp(self):
        import shutil
        from ugv_bridge.backends.dryrun_backend import DryRunBackend
        from ugv_bridge.protocol import load_key, write_key_file
        from ugv_bridge.server import BridgeServer
        from ugv_nav.config import DEFAULT_CONFIG
        from nav_helpers import bridge_config
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv robot mode ')
        self.dir = Path(self.tmp.name)
        shutil.copy(DEFAULT_CONFIG, self.dir / 'navigation.toml')
        write_key_file(self.dir / 'mentorpi.key')
        self.backend = DryRunBackend()
        self.bridge = BridgeServer(bridge_config(), self.backend, load_key(self.dir / 'mentorpi.key')).start()
        self.app = ConsoleApp(config_path=self.dir / 'navigation.toml', settings_path=None,
                              edit_dir=self.dir / 'edited', export_dir=self.dir / 'exports')

    def tearDown(self):
        self.app.close()
        self.bridge.shutdown()
        self.tmp.cleanup()

    def cmd(self, name, **args):
        return self.app.command(name, args)

    def test_connect_show_link_and_never_arm_by_itself(self):
        app = self.app
        self.assertTrue(self.cmd('set_mode', mode='robot')['ok'])
        self.assertTrue(self.cmd('set_options', values=dict(host='127.0.0.1', port=self.bridge.port, radius=0.143,
                                                            margin=0.02, verified=True))['ok'])
        self.assertTrue(self.cmd('connect')['ok'])
        self.assertTrue(wait_for(lambda: app.client.link_ok(), timeout=6))
        nav = app.view('nav')
        self.assertTrue(nav['connected'] and nav['link']['ok'])
        self.assertFalse(nav['link']['armed'])
        self.assertIsNone(app.session)                                          # a real link, not the simulator
        self.assertTrue(self.cmd('set_start', x=1.215, y=0.405, heading=0)['ok'])
        self.assertTrue(self.cmd('confirm_pose')['ok'])
        self.assertEqual(app.view('nav')['loc'][0], 'OK')
        self.assertTrue(self.cmd('set_goal', x=2.205, y=0.405)['ok'])
        time.sleep(1.0)
        self.assertFalse(self.bridge.armed)                                     # planning never arms or moves
        self.assertTrue(all(c[1:] == (0.0, 0.0, 0.0) for c in self.backend.commands))   # only zero-velocity frames
        self.assertFalse(self.cmd('sync_pose_sim')['ok'])                       # simulator-only helper refused
        self.assertFalse(self.cmd('sim_block', x=1.5, y=1.0)['ok'])
        self.assertTrue(self.cmd('disconnect')['ok'])
        self.assertIsNone(app.nav)

    def test_wrong_key_gives_no_link(self):
        from ugv_bridge.protocol import generate_key_hex
        (self.dir / 'mentorpi.key').write_text(generate_key_hex() + '\n')        # not the bridge's key
        app = self.app
        self.cmd('set_mode', mode='robot')
        self.cmd('set_options', values=dict(host='127.0.0.1', port=self.bridge.port))
        self.assertTrue(self.cmd('connect')['ok'])                              # the attempt starts ...
        time.sleep(1.5)
        self.assertFalse(app.client.link_ok())                                  # ... but never authenticates
        self.assertFalse(app.view('nav')['link']['ok'])
        self.assertFalse(self.cmd('confirm_pose')['ok'])


class ConfigTests(unittest.TestCase):
    def test_planner_config_matches_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = ConsoleApp(settings_path=None, edit_dir=Path(tmp) / 'e', export_dir=Path(tmp) / 'x', start_ticker=False)
            try:
                app.command('set_options', dict(values=dict(radius=0.143, margin=0.02, verified=True, connectivity=4,
                                                            clearance_weight=2.5, extra_clearance=0.15)))
                self.assertEqual(app.planner_config(),
                                 PlannerConfig(4, Footprint(0.143, 0.02, True), True, False, 2.5, 0.15))
                self.assertTrue(np.isfinite(app.view('settings')['planning_radius']))
            finally:
                app.close()


if __name__ == '__main__':
    unittest.main()


class OrientationTests(ConsoleCase):
    def test_env2_and_env3_are_shown_in_their_real_orientation_and_files_stay_as_they_are(self):
        import numpy as np
        from ugv_planner import load_environment
        from ugv_nav.map_store import load_edited, load_env
        app = self.app
        for env, turns in (('ENV1', 0), ('ENV2', 1), ('ENV3', -1)):
            self.ok('set_env', env=env)
            raw, raw_gt = load_environment(env)
            self.assertTrue(np.array_equal(app.grid.cells, np.rot90(raw.cells, turns)), env)
            self.assertTrue(np.array_equal(app.gt.cells, np.rot90(raw_gt.cells, turns)), env)
            self.assertIsNone(app.goal_point)                                   # a new environment has no goal either
            self.assertFalse(app.grid.cells[app.grid.world_to_cell(app.start_point)])
        # ENV2: picture turned 90 deg clockwise -> what was at the left edge is now at the top
        raw, _ = load_environment('ENV2')
        turned, _ = load_env('ENV2')
        self.assertTrue(np.array_equal(turned.cells[-1, :], raw.cells[:, 0]))
        # an edited map is stored in the orientation of the data files and comes back turned
        self.ok('set_env', env='ENV3')
        self.ok('paint', points=[[1.5, 1.5], [1.6, 1.5]], value=1, brush=2, end=True)
        self.ok('save_map')
        stored = np.load(app.edit_dir / 'ENV3_occupancy_edited.npy')
        self.assertTrue(np.array_equal(np.rot90(np.flipud(stored), -1), app.grid.cells))
        self.assertTrue(np.array_equal(load_edited('ENV3', app.edit_dir).cells, app.grid.cells))
