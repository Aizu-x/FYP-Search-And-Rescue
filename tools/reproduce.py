"""Regenerate measured report artifacts. Run from any working directory."""
from pathlib import Path
import sys
import json
import platform
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ugv_planner.batch import run_batch
from ugv_planner.config import PlannerConfig, Footprint, EXAMPLES
from ugv_planner.grid import load_environment, dataset_manifest, DATA_DIR
from ugv_planner.astar import plan
from ugv_planner.evaluation import evaluate
from ugv_planner.exporting import draw_map


def main():
    baseline = run_batch(ROOT/'example_outputs')
    inflated = run_batch(ROOT/'example_outputs'/'inflation_experiment', PlannerConfig(8, Footprint(.10, .02)))
    run_batch(ROOT/'example_outputs'/'clearance_preferred', PlannerConfig(8, Footprint(0, .10), clearance_weight=4))
    manifest = {}
    for name, digest in dataset_manifest().items():
        a = np.load(DATA_DIR/name, allow_pickle=False)
        manifest[name] = dict(sha256=digest, shape=list(a.shape), dtype=str(a.dtype),
                              free_cells=int((a == 0).sum()), occupied_cells=int((a == 1).sum()))
    (ROOT/'data_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    fig = Figure(figsize=(15, 6.4)); FigureCanvasAgg(fig)
    fig.subplots_adjust(left=.05, right=.99, top=.87, bottom=.20, wspace=.22)
    for index, (name, (start, goal)) in enumerate(EXAMPLES.items(), 1):
        grid, gt = load_environment(name)
        result = plan(grid, grid.world_to_cell(start), grid.world_to_cell(goal))
        report = evaluate(result, gt, Footprint())
        ax = fig.add_subplot(1, 3, index)
        draw_map(ax, grid, gt, result, report, PlannerConfig(), show_gt=True)
        ax.set_title(f'{name} | {result.distance_m:.3f} m\n'+('GT centerline collision' if report['gt_center_collision'] else 'Centerline clear; footprint unknown'), fontsize=10)
    fig.savefig(ROOT/'example_outputs'/'comparison.png', dpi=150)
    lines = ['# Executed results', '',
             'Generated from the six supplied arrays by `python tools/reproduce.py`. Timings are single-run wall-clock measurements, not real-time guarantees.', '',
             '## Baseline: 8-way, unknown footprint, zero margin', '',
             '| Metric | ENV1 | ENV2 | ENV3 |', '|---|---:|---:|---:|']
    for label, key in [('Path found', 'path_found'), ('Path length (m)', 'path_length_m'),
                       ('Planning time (ms)', 'planning_ms'), ('Expanded nodes', 'expanded_nodes'),
                       ('Turns', 'turns'), ('Detour ratio', 'detour_ratio'),
                       ('Minimum GT center clearance (cm)', 'minimum_gt_clearance_cm'),
                       ('GT centerline collision', 'gt_center_collision'), ('Simulation outcome', 'simulation_state')]:
        values = [f'{r[key]:.3f}' if type(r[key]) is float else str(r[key]) for r in baseline.values()]
        lines.append('| '+label+' | '+' | '.join(values)+' |')
    lines += ['', '| GT validation | ENV1 | ENV2 | ENV3 |', '|---|---|---|---|',
              '| Status | '+' | '.join(r['validation'] for r in baseline.values())+' |', '',
              '![Actual three-environment routes](example_outputs/comparison.png)', '',
              'The shortest-distance ENV3 route is valid against occupancy but intersects GT. The new preflight gate blocks dispatch. ENV1/ENV2 have only 1.5 cm centerline clearance; this is not a physical robot safety claim.', '',
              '## Sensitivity experiment: arbitrary radius 0.10 m, margin 0.02 m', '',
              'These are test parameters, **not MentorPi measurements**, and remain unverified.', '',
              '| Metric | ENV1 | ENV2 | ENV3 |', '|---|---:|---:|---:|']
    for label, key in [('Path found', 'path_found'), ('Path length (m)', 'path_length_m'),
                       ('Planning time (ms)', 'planning_ms'), ('Expanded nodes', 'expanded_nodes'),
                       ('Turns', 'turns'), ('Detour ratio', 'detour_ratio'),
                       ('Minimum GT center clearance (cm)', 'minimum_gt_clearance_cm'),
                       ('Minimum footprint clearance (cm)', 'minimum_footprint_clearance_cm'),
                       ('GT footprint collision', 'gt_footprint_collision'), ('Simulation outcome', 'simulation_state')]:
        values = [f'{r[key]:.3f}' if type(r[key]) is float else str(r[key]) for r in inflated.values()]
        lines.append('| '+label+' | '+' | '.join(values)+' |')
    lines += ['', 'ENV3 clears the configured physical disk but fails its additional 2 cm margin. That is correctly reported as a clearance failure; it is not a centerline collision in this experiment.', '',
              '## Verification scope', '',
              'See `verification/tests.txt` for the executed suite and `verification/runtime.json` for the runtime. The GUI callback test creates actual Tk widgets, switches all environments, changes endpoints/footprints, replans during motion, tests edits and exports, and captures callback exceptions. It skips explicitly if a display is unavailable.', '',
              'The Windows interface was also visually inspected and its Start/Pause controls exercised through desktop UI input. Ubuntu and physical MentorPi/ROS2 execution were not tested. Array shape/coordinate conventions were checked; physical registration cannot be established from `.npy` alone.', '',
              'The analytical no-route cases and inflation-induced passage closure are in `tests/test_system.py`. Source files are preserved and identified in `data_manifest.json`.']
    (ROOT/'RESULTS.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    verification = ROOT/'verification'; verification.mkdir(exist_ok=True)
    runtime = dict(python=sys.version, platform=platform.platform(), numpy=np.__version__, matplotlib=matplotlib.__version__)
    (verification/'runtime.json').write_text(json.dumps(runtime, indent=2), encoding='utf-8')
    completed = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v'],
                               cwd=ROOT, capture_output=True, text=True)
    (verification/'tests.txt').write_text(completed.stdout+completed.stderr, encoding='utf-8')
    print(completed.stdout+completed.stderr)
    if completed.returncode:
        raise SystemExit(completed.returncode)
    print(f'Artifacts regenerated in {ROOT}')


if __name__ == '__main__':
    main()
