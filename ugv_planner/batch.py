"""Batch runner for the three example environments (used by `main.py --batch` and the GUI button).

REPLACEMENT FILE: the original batch.py could not be extracted from the uploaded RAR archive.
This version provides the same run_batch(folder, config) -> {env: report} interface. If you still
have the original batch.py, copy it over this file.
"""
from pathlib import Path
from .grid import load_environment
from .config import EXAMPLES
from .astar import plan
from .evaluation import evaluate, simulation_preflight
from .simulation import SimulatedUGV
from .collision import CollisionChecker
from .exporting import export_run, export_summary, export_trajectory


def run_batch(folder, config):
    reports = {}
    for name, (start, goal) in EXAMPLES.items():
        grid, gt = load_environment(name)
        result = plan(grid, grid.world_to_cell(start), grid.world_to_cell(goal), config)
        report = evaluate(result, gt, config.footprint)
        ok, _ = simulation_preflight(result, grid, report, config.footprint)
        sim = SimulatedUGV(); sim.set_route(result.waypoints)
        if ok:
            sim.start(); g = CollisionChecker(gt); r = config.footprint.effective_radius
            while sim.state == 'running':
                sim.step(0.1, lambda a, b: g.is_clear(a, b, r))
        report['simulation_state'] = sim.state
        export_run(folder, name, grid, gt, result, report, config)
        export_trajectory(folder, name, sim.history)
        reports[name] = report
    export_summary(folder, reports)
    return reports
