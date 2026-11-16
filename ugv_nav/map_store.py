"""Persistent, hand-edited occupancy maps -- stored NEXT TO the originals, never over them.

    data/ENV1_occupancy.npy                     original ERFNet map (read-only, never written)
    data/edited/ENV1_occupancy_edited.npy       your edited map (same format: 100x100 uint8 0/1, row 0 = top)
    data/edited/ENV1_occupancy_edited.json      when/how it was edited
    data/edited/ENV1_occupancy_edited.<time>.bak.npy   previous edits kept when you revert or overwrite

The GT files are never edited: they stay an independent evaluation reference.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path

import numpy as np

from ugv_planner.grid import DATA_DIR, Grid, load_environment

EDIT_DIR = DATA_DIR / 'edited'

# Real-world orientation of each environment relative to its data file, as quarter turns for np.rot90 on the
# in-memory (row 0 = bottom) cells. The data files and the edited-map files are never rotated: only the
# in-memory map is, so files saved before this table existed still load correctly.
#   ENV2: picture turned 90 deg clockwise, ENV3: 90 deg counter-clockwise (operator's photos of the arenas).
ENV_QUARTER_TURNS = {'ENV1': 0, 'ENV2': 1, 'ENV3': -1}


def _turn(grid: Grid, k):
    if k % 4 == 0:
        return grid
    return Grid(np.rot90(grid.cells, k).astype(np.uint8), grid.resolution, grid.origin)


def load_env(env):
    """(occupancy Grid, ground-truth Grid) for ``env`` in its real-world orientation."""
    k = ENV_QUARTER_TURNS.get(env, 0)
    occ, gt = load_environment(env)
    return _turn(occ, k), _turn(gt, k)


def example_points(env):
    """The planner's example (start, goal) for ``env``, turned with the map."""
    from ugv_planner.config import EXAMPLES
    grid, _ = load_environment(env)
    k = ENV_QUARTER_TURNS.get(env, 0)
    h, w = grid.shape
    out = []
    for point in EXAMPLES[env]:
        mark = np.zeros((h, w), dtype=np.uint8)
        mark[grid.world_to_cell(point)] = 1
        r, c = np.argwhere(np.rot90(mark, k))[0]
        out.append(grid.cell_to_world((int(r), int(c))))
    return tuple(out)


def edited_path(env, edit_dir=EDIT_DIR):
    return Path(edit_dir) / ('%s_occupancy_edited.npy' % env)


def _atomic_save_npy(path, array):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, suffix='.tmp.npy')
    os.close(fd)
    try:
        np.save(tmp, array)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


def _backup(path):
    if path.exists():
        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup = path.with_name('%s.%s.bak.npy' % (path.stem, stamp))
        os.replace(path, backup)
        meta = path.with_suffix('.json')
        if meta.exists():
            os.replace(meta, backup.with_suffix('.json'))
        return backup
    return None


def save_edited(env, grid: Grid, edit_dir=EDIT_DIR):
    """Save ``grid`` (internal bottom-first) as the edited map for ``env``. Returns the file path."""
    original, _ = load_env(env)
    if grid.shape != original.shape:
        raise ValueError('Edited map must have the original shape %s.' % (original.shape,))
    path = edited_path(env, edit_dir)
    _backup(path)  # keep the previous edited version instead of overwriting it
    k = ENV_QUARTER_TURNS.get(env, 0)
    _atomic_save_npy(path, np.flipud(np.rot90(grid.cells, -k)).astype(np.uint8))  # same orientation as the source files
    changed = int(np.count_nonzero(grid.cells != original.cells))
    meta = dict(environment=env, saved_at=time.strftime('%Y-%m-%d %H:%M:%S'), cells_changed_vs_original=changed,
                added_obstacle_cells=int(np.count_nonzero(grid.cells & ~original.cells)),
                removed_obstacle_cells=int(np.count_nonzero(~grid.cells & original.cells)),
                resolution_m=grid.resolution, orientation='row 0 = top (same as data/*.npy)')
    path.with_suffix('.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    return path


def load_edited(env, edit_dir=EDIT_DIR):
    """Return the saved edited Grid for ``env`` or None if there is none."""
    path = edited_path(env, edit_dir)
    if not path.exists():
        return None
    a = np.load(path, allow_pickle=False)
    original, _ = load_environment(env)
    if a.shape != original.shape:
        raise ValueError('%s has shape %s, expected %s.' % (path, a.shape, original.shape))
    return _turn(Grid(np.flipud(a), original.resolution, original.origin), ENV_QUARTER_TURNS.get(env, 0))


def revert_to_original(env, edit_dir=EDIT_DIR):
    """Move the saved edits aside (kept as a .bak file) so the original map is used again."""
    return _backup(edited_path(env, edit_dir))


def paint(grid: Grid, center_xy, brush_cells, occupied):
    """Return a new Grid with a disk of radius ``brush_cells`` cells set to occupied/free."""
    r0, c0 = grid.world_to_cell(center_xy)
    cells = grid.cells.copy()
    n = max(0, int(brush_cells))
    h, w = cells.shape
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            r, c = r0 + dr, c0 + dc
            if 0 <= r < h and 0 <= c < w and math.hypot(dr, dc) <= n + 0.5:
                cells[r, c] = occupied
    if np.array_equal(cells, grid.cells):
        return grid
    return Grid(cells.astype(np.uint8), grid.resolution, grid.origin)
