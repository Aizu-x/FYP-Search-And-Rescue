"""Edge navigation entry point (version 2: web console). The original ``main.py`` is unchanged.

    python nav_main.py gui                      # operator console in its own window (planner / simulator / robot)
    python nav_main.py import-old <folder>      # copy key, robot settings, edited maps from the previous project
    python nav_main.py sim-demo --env ENV1      # headless closed-loop simulation, writes a report
    python nav_main.py check-connection --host <PI_IP>   # Test B: link + telemetry only, NO motion
    python nav_main.py motor-test --host <PI_IP>         # Test C: wheels-off-ground, operator-gated
    python nav_main.py genkey --out config/mentorpi.key  # shared key (copy to the Pi too)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def _client_from_args(args):
    from ugv_bridge.protocol import load_key
    from ugv_nav.config import load_config
    from ugv_nav.edge_client import EdgeClient
    cfg = load_config(args.config)
    host = args.host or cfg.network.ugv_host
    if not host:
        sys.exit('No UGV host: pass --host or set network.ugv_host in config/navigation.toml')
    client = EdgeClient(host, args.port or cfg.network.ugv_port, load_key(cfg.key_path), cfg.network.client_id,
                        link_timeout=cfg.network.link_timeout_s).start()
    return client, cfg


def _wait_link(client, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if client.link_ok():
            return True
        time.sleep(0.1)
    return False


def console(args):
    """The operator console: a local web page served from this computer only (127.0.0.1)."""
    from ugv_console.server import run
    return run(config_path=args.config, port=args.port, stay=args.stay, verbose=args.verbose,
               browser='none' if args.no_browser else ('tab' if args.tab else 'window'))


def _toml_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, str):
        return json.dumps(value)
    return repr(value)


def import_old(args):
    """Copy YOUR values from the previous project folder into this one. The old folder is only read."""
    import shutil
    import tomllib
    src = Path(args.folder).expanduser().resolve()
    if not (src / 'ugv_planner').is_dir() or not (src / 'config').is_dir():
        sys.exit('"%s" does not look like the UGV project folder (no ugv_planner/ and config/ inside).' % src)
    if src == ROOT:
        sys.exit('That is this folder. Give the path of the OLD project folder.')
    done = []

    def copy(source, target, what):
        if not source.is_file():
            return
        if target.exists() and not args.force:
            done.append('kept existing %s (use --force to replace)' % target.relative_to(ROOT))
            return
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        done.append('copied %s' % what)

    copy(src / 'config' / 'mentorpi.key', ROOT / 'config' / 'mentorpi.key', 'the shared key (config/mentorpi.key)')
    copy(src / 'config' / 'gui_settings.json', ROOT / 'config' / 'gui_settings.json',
         'the old console settings (used on the first start of the new console)')
    for item in sorted((src / 'data' / 'edited').glob('*')) if (src / 'data' / 'edited').is_dir() else []:
        copy(item, ROOT / 'data' / 'edited' / item.name, 'edited map file data/edited/%s' % item.name)
    old_toml = src / 'config' / 'navigation.toml'
    if old_toml.is_file():
        old = tomllib.loads(old_toml.read_text(encoding='utf-8'))
        from ugv_nav.config import local_config_path
        target = ROOT / 'config' / 'navigation.toml'
        shipped = tomllib.loads(target.read_text(encoding='utf-8'))
        local_file = local_config_path(target)
        local = tomllib.loads(local_file.read_text(encoding='utf-8')) if local_file.is_file() else {}
        changed = 0
        for section, values in shipped.items():
            if not isinstance(values, dict) or not isinstance(old.get(section), dict):
                continue
            for key, value in values.items():
                if key in old[section] and old[section][key] != value and local.get(section, {}).get(key) != old[section][key]:
                    local.setdefault(section, {})[key] = old[section][key]
                    changed += 1
        if changed:
            text = ['# YOUR values. They win over config/navigation.toml and are never touched by updates (git pull).']
            for section, values in local.items():
                text.append('\n[%s]' % section)
                text.extend('%s = %s' % (key, _toml_value(value)) for key, value in values.items())
            local_file.write_text('\n'.join(text) + '\n', encoding='utf-8')
        done.append('took %d value(s) from the old config/navigation.toml (robot size, IP, limits ...) into '
                    'config/navigation.local.toml' % changed)
    print('\n'.join('  - ' + line for line in done) if done else 'Nothing to import was found.')
    print('The old folder was not changed. Start the console with:  python nav_main.py gui')
    return 0


def check_connection(args):
    """Test B on the real robot: authenticate, read telemetry, measure RTT. Sends no motion."""
    client, _ = _client_from_args(args)
    try:
        if not _wait_link(client):
            print('FAILED: no authenticated link.', client.snapshot()['error']); return 1
        samples = []
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            s = client.snapshot(); samples.append((s['rtt'], s['telemetry_age'])); time.sleep(0.1)
        s = client.snapshot(); tel = s['telemetry']
        rtts = [r for r, _ in samples if r is not None]
        print('Link OK. RTT min/max: %.1f / %.1f ms' % (min(rtts) * 1e3, max(rtts) * 1e3))
        print('Max telemetry age: %.0f ms' % (max(a for _, a in samples if a is not None) * 1e3))
        print('UGV armed:', tel['armed'], '(must be False)')
        for k in ('odom', 'imu'):
            print('%-5s' % k, json.dumps(tel.get(k)))
        scan = tel.get('scan')
        print('scan ', 'missing' if scan is None else '%d points, age %.0f ms' % (len(scan['points']), scan['age'] * 1e3))
        print('UGV counters:', tel.get('counters'), '\nRecent UGV events:', tel.get('events'))
        return 0
    finally:
        client.close()


def motor_test(args):
    """Test C. Wheels must be OFF THE GROUND. Every phase needs operator confirmation."""
    print(__doc__.split('\n')[0])
    print('\nTEST C: WHEELS-OFF-GROUND MOTOR TEST\n'
          'Put the MentorPi on a stand so NO wheel touches anything. Keep a hand on the power switch.\n'
          'Commands: rotate wz=+0.2 rad/s for 1 s, forward vx=+0.05 m/s for 1 s, then a watchdog test.')
    if input('Type exactly  WHEELS OFF GROUND  to continue: ').strip() != 'WHEELS OFF GROUND':
        print('Not confirmed. Nothing sent.'); return 1
    client, _ = _client_from_args(args)
    try:
        if not _wait_link(client):
            print('No link.'); return 1

        def odom():
            t = client.snapshot()['telemetry']; return None if not t or not t.get('odom') else t['odom']

        def phase(label, vx, wz, seconds=1.0):
            input('\n[Enter] to run: %s (Ctrl+C aborts)' % label)
            ok, why = client.arm()
            if not ok:
                print('UGV refused to arm:', why); return False
            before = odom(); t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                client.send_velocity(vx, 0.0, wz, ttl=0.3); time.sleep(0.1)
            client.stop(); time.sleep(0.6)
            after = odom()
            print('  odom before:', before, '\n  odom after: ', after)
            print('  UGV output now:', client.snapshot()['telemetry']['output'], '(must be zeros)')
            return True

        if not phase('rotate in place wz=+0.2 rad/s for 1 s', 0.0, 0.2):
            return 1
        if not phase('forward vx=+0.05 m/s for 1 s', 0.05, 0.0):
            return 1
        input('\n[Enter] to run WATCHDOG test: one vx=0.05 command, then silence (no STOP sent)')
        ok, why = client.arm()
        if not ok:
            print('UGV refused to arm:', why); return 1
        client.send_velocity(0.05, 0.0, 0.0, ttl=0.3)
        t0 = time.monotonic(); seen_motion = False; stopped_at = None
        while time.monotonic() - t0 < 2.0:
            out = client.snapshot()['telemetry']['output']
            seen_motion |= any(out)
            if seen_motion and not any(out) and stopped_at is None:
                stopped_at = time.monotonic() - t0
            time.sleep(0.02)
        tel = client.snapshot()['telemetry']
        print('  motion seen: %s, output zero after %s s, armed now: %s'
              % (seen_motion, 'n/a' if stopped_at is None else '%.2f' % stopped_at, tel['armed']))
        print('  PASS criterion: output zero within ~0.35 s and armed False within ~1.1 s. Verify the WHEELS stopped.')
        return 0
    except KeyboardInterrupt:
        print('\nAborted: STOP sent.')
        return 1
    finally:
        client.stop(); time.sleep(0.2); client.close()


def sim_demo(args):
    """Closed-loop simulation of the full pipeline on a real environment map."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    from ugv_planner import Footprint, PlannerConfig
    from ugv_planner.exporting import draw_map
    from ugv_planner.evaluation import evaluate
    from ugv_nav.controller import ControllerConfig
    from ugv_nav.localization import LocalizationConfig
    from ugv_nav.navigator import RUNNING, Navigator, NavigatorConfig
    from ugv_nav.safety_monitor import SafetyConfig
    from ugv_nav.sim_session import SimulatedUGVSession

    from ugv_nav.map_store import example_points, load_env
    occ, gt = load_env(args.env)                    # real-world orientation, like the console
    start_xy, goal = example_points(args.env)
    start = (*occ.cell_to_world(occ.world_to_cell(start_xy)), 0.0)
    goal = occ.cell_to_world(occ.world_to_cell(goal))
    session = SimulatedUGVSession(gt, start, 0.08, 0.07, sim_kwargs=dict(odom_scale_error=args.odom_error))
    planner = PlannerConfig(8, Footprint(math.hypot(0.08, 0.07), 0.02, True), True, clearance_weight=4.0)
    nav = Navigator(session.client, occ, planner, NavigatorConfig(
        controller=ControllerConfig(max_v=args.speed),
        localization=LocalizationConfig(drift_per_m=args.drift_per_m, max_sigma_m=args.max_sigma),
        safety=SafetyConfig(0.08, 0.07)))
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    try:
        if not _wait_link(session.client):
            print('loopback link failed'); return 1
        time.sleep(0.2)
        nav.set_initial_pose(start)
        recalibrations = 0
        nav.set_goal(goal)
        print(nav.message)
        t0 = time.monotonic()
        while True:
            ok, msg = nav.authorize()
            if not ok:
                print('authorize:', msg); break
            nav.run_in_background()
            while nav.state == RUNNING and time.monotonic() - t0 < args.timeout:
                time.sleep(0.2)
            nav.shutdown()
            snap = nav.snapshot(); print(snap['state'], '-', snap['message'])
            if snap['state'] == 'STOPPED' and 'LOW_CONFIDENCE' in snap['message'] and args.recalibrate and recalibrations < 5:
                # Emulates the operator re-measuring the robot's true position (e.g. against floor marks).
                truth = session.backend.truth_pose()
                nav.set_initial_pose(truth); recalibrations += 1
                print('  recalibrated at true pose (%.3f, %.3f) -- operator procedure in the field' % truth[:2])
                nav.set_goal(goal)
                continue
            break
        elapsed = time.monotonic() - t0
        snap = nav.snapshot()
        truth = session.backend.truth_pose()
        summary = dict(environment=args.env, final_state=snap['state'], message=snap['message'],
                       goal=goal, estimated_final=snap['pose'], true_final=truth,
                       true_error_to_goal_m=math.dist(truth[:2], goal), elapsed_s=elapsed,
                       collided_with_gt=session.backend.collided, recalibrations=recalibrations,
                       odom_scale_error=args.odom_error, note='Simulation only; not a physical result.')
        (out / ('%s_closed_loop.json' % args.env)).write_text(json.dumps(summary, indent=2))
        fig = Figure(figsize=(7, 7)); ax = fig.add_subplot(111)
        from ugv_planner.astar import plan
        result = plan(occ, occ.world_to_cell(start[:2]), occ.world_to_cell(goal), planner)
        draw_map(ax, occ, gt, result, evaluate(result, gt, planner.footprint), planner, show_gt=True)
        tr = snap['trajectory']; th = session.backend.truth_history
        ax.plot([p[0] for p in tr], [p[1] for p in tr], color='#1565c0', lw=2, label='estimated')
        ax.plot([p[0] for p in th], [p[1] for p in th], color='black', lw=1, ls=':', label='true')
        ax.set_title('%s closed-loop sim: %s (%.1f s)' % (args.env, snap['state'], elapsed))
        fig.savefig(out / ('%s_closed_loop.png' % args.env), dpi=140)
        print(json.dumps(summary, indent=2))
        return 0 if snap['state'] == 'ARRIVED' and not session.backend.collided else 2
    finally:
        nav.shutdown(); session.close()


def main(argv=None):
    from ugv_nav.config import DEFAULT_CONFIG
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('gui', help='operator console (opens in its own window)')
    g.add_argument('--config', default=DEFAULT_CONFIG)
    g.add_argument('--port', type=int, help='local port (default: first free one from 8780)')
    g.add_argument('--tab', action='store_true', help='open in a normal browser tab instead of an app window')
    g.add_argument('--no-browser', action='store_true', help='only print the link')
    g.add_argument('--stay', action='store_true', help='keep running after the console page is closed')
    g.add_argument('--verbose', action='store_true')
    i = sub.add_parser('import-old', help='copy key, robot settings and edited maps from the previous project folder')
    i.add_argument('folder'); i.add_argument('--force', action='store_true', help='replace files that already exist here')
    k = sub.add_parser('genkey'); k.add_argument('--out', help='write the key to this file (recommended on Windows)')
    for name in ('check-connection', 'motor-test'):
        s = sub.add_parser(name)
        s.add_argument('--host'); s.add_argument('--port', type=int); s.add_argument('--config', default=DEFAULT_CONFIG)
        s.add_argument('--seconds', type=float, default=5.0)
    d = sub.add_parser('sim-demo')
    d.add_argument('--env', default='ENV1', choices=['ENV1', 'ENV2', 'ENV3'])
    d.add_argument('--speed', type=float, default=0.12)
    d.add_argument('--odom-error', type=float, default=0.02)
    d.add_argument('--drift-per-m', type=float, default=0.05)
    d.add_argument('--max-sigma', type=float, default=0.15)
    d.add_argument('--recalibrate', action='store_true', help='emulate operator re-localization when drift limit is hit')
    d.add_argument('--timeout', type=float, default=300)
    d.add_argument('--output', default=str(ROOT / 'nav_outputs'))
    args = p.parse_args(argv)
    if args.cmd == 'gui':
        return console(args)
    if args.cmd == 'import-old':
        return import_old(args)
    if args.cmd == 'genkey':
        from ugv_bridge.protocol import generate_key_hex, write_key_file
        if args.out:
            write_key_file(args.out)  # refuses to overwrite an existing key
            print('Key written to %s. Copy this SAME file to the Pi.' % args.out)
        else:
            print(generate_key_hex())
        return 0
    return {'check-connection': check_connection, 'motor-test': motor_test, 'sim-demo': sim_demo}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
