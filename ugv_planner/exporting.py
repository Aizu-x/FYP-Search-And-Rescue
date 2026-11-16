from pathlib import Path
import csv
import json
from dataclasses import asdict
import numpy as np
from .inflation import inflated_mask
from .grid import dataset_manifest


def draw_map(ax, grid, gt, result, report, config, show_gt=False, show_search=False, show_path=True):
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    ax.clear()
    ax.imshow(grid.cells, origin='lower', extent=grid.extent, interpolation='nearest',
              cmap=ListedColormap(['#f4f7fb', '#263942']), vmin=0, vmax=1)
    mask = inflated_mask(grid, config.planning_radius) & ~grid.cells
    ax.imshow(np.ma.masked_where(~mask, mask), origin='lower', extent=grid.extent,
              interpolation='nearest', cmap=ListedColormap(['#f4bf67']), alpha=.55, vmin=0, vmax=1)
    if show_gt:
        ax.imshow(np.ma.masked_where(~gt.cells, gt.cells), origin='lower', extent=gt.extent,
                  interpolation='nearest', cmap=ListedColormap(['#d65166']), alpha=.4, vmin=0, vmax=1)
    if show_search and result.search_cells:
        xy = np.asarray([grid.cell_to_world(c) for c in result.search_cells])
        ax.scatter(xy[:, 0], xy[:, 1], s=3, c='#6a9ee5', alpha=.25)
    if result.found:
        xy = np.asarray(result.waypoints)
        if show_path:
            ax.plot(xy[:, 0], xy[:, 1], color='#168a74', lw=2)
            for i in report['unsafe_segments']:
                segment = xy[i:i+2]
                ax.plot(segment[:, 0], segment[:, 1], color='#dc334d', lw=3)
        ax.scatter(*xy[0], c='#087d67', marker='o', s=65, edgecolors='white', zorder=5)
        ax.scatter(*xy[-1], c='#972f67', marker='*', s=140, edgecolors='white', zorder=5)
    ax.set(xlabel='x (m)', ylabel='y (m)', xlim=grid.extent[:2], ylim=grid.extent[2:])
    ax.set_aspect('equal'); ax.grid(alpha=.12)
    handles = [Line2D([], [], color='#263942', lw=6, label='Occupancy'),
                       Line2D([], [], color='#f4bf67', lw=6, label='Inflation'),
                       Line2D([], [], color='#168a74', label='A* route'),
                       Line2D([], [], color='#dc334d', label='GT violation'),
                       Line2D([], [], marker='o', color='#087d67', lw=0, label='Start'),
                       Line2D([], [], marker='*', color='#972f67', lw=0, label='Goal')]
    if show_gt:
        handles.append(Line2D([], [], color='#d65166', alpha=.4, lw=6, label='GT overlay'))
    ax.legend(handles=handles,
              loc='upper center', bbox_to_anchor=(.5, -.12), ncol=3, fontsize=8)


def export_run(folder, name, grid, gt, result, report, config):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    payload = dict(environment=name, config=asdict(config), metrics=report,
                   waypoints_m=result.waypoints, route_cells=result.cells,
                   resolution_m=grid.resolution, origin_m=grid.origin,
                   orientation='Internal bottom-first; source files top-first',
                   simulation_only_edits=bool(report.get('simulation_only_edits', False)),
                   source_sha256=dataset_manifest())
    (folder/f'{name}.json').write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
    with (folder/f'{name}.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(report)); writer.writeheader(); writer.writerow(report)
    with (folder/f'{name}_waypoints.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f); writer.writerow(['x_m', 'y_m']); writer.writerows(result.waypoints)
    # A snapshot of the exact planning grid makes simulation-only edits reproducible.
    np.save(folder/f'{name}_planning_grid_bottom_first.npy', grid.cells.astype(np.uint8))
    fig = Figure(figsize=(7, 8), constrained_layout=True); FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    draw_map(ax, grid, gt, result, report, config, show_gt=True)
    ax.set_title(f'{name} | {report["validation"]}\n{result.distance_m:.3f} m | {result.computation_ms:.1f} ms', fontsize=10)
    fig.savefig(folder/f'{name}.png', dpi=160)
    return folder


def export_summary(folder, reports):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    with (folder/'summary.csv').open('w', newline='', encoding='utf-8') as f:
        fields = ['environment']+list(next(iter(reports.values())))
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for name, report in reports.items():
            writer.writerow(dict(environment=name, **report))
    (folder/'summary.json').write_text(json.dumps(reports, indent=2, allow_nan=False), encoding='utf-8')


def export_trajectory(folder, name, history):
    """Ordered executed positions; no timestamps implied by this discrete history."""
    with (Path(folder)/f'{name}_executed_positions.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f); writer.writerow(['sample_index', 'x_m', 'y_m'])
        writer.writerows((i, *point) for i, point in enumerate(history))
