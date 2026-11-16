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


# Real size of the field. The data files are 100 x 100 cells and were assumed to be 3 cm per cell. If the real
# arena is larger or smaller than the map says, the map is stretched by (scale_x, scale_y) and cut into 3 cm
# cells again, so that every coordinate the console shows is a real distance in metres.
FIELD_SCALE_MIN, FIELD_SCALE_MAX = 0.6, 1.6


def validate_field_scale(scale):
    sx, sy = (float(v) for v in scale)
    for name, v in (('X', sx), ('Y', sy)):
        if not math.isfinite(v) or not FIELD_SCALE_MIN <= v <= FIELD_SCALE_MAX:
            raise ValueError('Field size %s: the real size would be %.0f %% of the map; allowed is %.0f-%.0f %%. '
                             'Check the measurement.' % (name, 100 * v, 100 * FIELD_SCALE_MIN, 100 * FIELD_SCALE_MAX))
    return sx, sy


def scaled_shape(shape, scale):
    """Cells (rows, columns) of a map of ``shape`` after stretching by scale = (x, y)."""
    sx, sy = scale
    return max(8, int(round(shape[0] * sy))), max(8, int(round(shape[1] * sx)))


def resample_to(grid: Grid, shape):
    """The same map stretched to ``shape`` cells of unchanged size. A new cell is wall when at least 40 % of
    it was wall, so a wall one cell thick never disappears and never doubles for stretches of 0.8-1.6."""
    h, w = grid.shape
    rows, cols = int(shape[0]), int(shape[1])
    if (rows, cols) == (h, w):
        return grid
    integral = np.zeros((h + 1, w + 1))
    integral[1:, 1:] = grid.cells.astype(float).cumsum(0).cumsum(1)
    ys = np.arange(rows + 1) * (h / rows)
    xs = np.arange(cols + 1) * (w / cols)
    y0 = np.minimum(ys.astype(int), h - 1); x0 = np.minimum(xs.astype(int), w - 1)
    fy = (ys - y0)[:, None]; fx = (xs - x0)[None, :]
    total = (integral[np.ix_(y0, x0)] * (1 - fy) * (1 - fx) + integral[np.ix_(y0, x0 + 1)] * (1 - fy) * fx
             + integral[np.ix_(y0 + 1, x0)] * fy * (1 - fx) + integral[np.ix_(y0 + 1, x0 + 1)] * fy * fx)
    cover = (total[1:, 1:] - total[:-1, 1:] - total[1:, :-1] + total[:-1, :-1]) / ((h / rows) * (w / cols))
    return Grid((cover >= 0.4).astype(np.uint8), grid.resolution, grid.origin)


def load_env(env, scale=(1.0, 1.0)):
    """(occupancy Grid, ground-truth Grid) for ``env`` in its real-world orientation and real size."""
    k = ENV_QUARTER_TURNS.get(env, 0)
    occ, gt = load_environment(env)
    occ, gt = _turn(occ, k), _turn(gt, k)
    shape = scaled_shape(occ.shape, scale)
    return resample_to(occ, shape), resample_to(gt, shape)


def example_points(env, scale=(1.0, 1.0)):
    """The planner's example (start, goal) for ``env``, turned and stretched with the map."""
    from ugv_planner.config import EXAMPLES
    grid, _ = load_environment(env)
    k = ENV_QUARTER_TURNS.get(env, 0)
    h, w = grid.shape
    out = []
    for point in EXAMPLES[env]:
        mark = np.zeros((h, w), dtype=np.uint8)
        mark[grid.world_to_cell(point)] = 1
        r, c = np.argwhere(np.rot90(mark, k))[0]
        x, y = grid.cell_to_world((int(r), int(c)))
        out.append((x * scale[0], y * scale[1]))
    return tuple(out)


def edited_path(env, edit_dir=EDIT_DIR):
    return Path(edit_dir) / ('%s_occupancy_edited.npy' % env)


# ---------------------------------------------------------------------------- arena geometry from the map
CORNERS = ('bl', 'br', 'tl', 'tr')
CORNER_NAMES = dict(bl='bottom-left', br='bottom-right', tl='top-left', tr='top-right')


def _faces(free, res, low_side):
    """Inner face of an outer wall for every line of ``free`` (lines along axis 1), NaN where a line has no floor.
    ``low_side``: the wall at index 0 (left / bottom) or the one at the far end (right / top)."""
    any_free = free.any(axis=1)
    if low_side:
        first = free.argmax(axis=1).astype(float)
    else:
        first = (free.shape[1] - free[:, ::-1].argmax(axis=1)).astype(float)
    first[~any_free] = np.nan
    return first * res


def inner_box(grid: Grid):
    """(x0, x1, y0, y1): where the floor begins inside the four outer walls, in metres.
    Median over all rows / columns, so wall roughness and obstacles standing against a wall do not matter."""
    free = ~grid.cells.astype(bool)
    if not free.any():
        raise ValueError('The map has no free floor.')
    res, (ox, oy) = grid.resolution, grid.origin
    x0 = float(np.nanmedian(_faces(free, res, True))); x1 = float(np.nanmedian(_faces(free, res, False)))
    y0 = float(np.nanmedian(_faces(free.T, res, True))); y1 = float(np.nanmedian(_faces(free.T, res, False)))
    return ox + x0, ox + x1, oy + y0, oy + y1


def corner_faces(grid: Grid, corner, span_m=0.6):
    """(x_face, y_face) of the two outer walls that meet in ``corner``, measured only near that corner."""
    if corner not in CORNERS:
        raise ValueError('Unknown corner "%s".' % corner)
    free = ~grid.cells.astype(bool)
    res, (ox, oy) = grid.resolution, grid.origin
    h, w = free.shape
    bx0, bx1, by0, by1 = inner_box(grid)
    left, bottom = corner[1] == 'l', corner[0] == 'b'
    n = max(3, int(round(span_m / res)))

    def band(face, low, size):
        k = int(round(face / res))
        return slice(max(0, k), min(size, k + n)) if low else slice(max(0, k - n), min(size, k))
    x_face = (bx0 if left else bx1) - ox
    y_face = (by0 if bottom else by1) - oy
    for _ in range(2):                              # the two faces define each other's band: settle in two rounds
        rows = band(y_face, bottom, h)
        xs = _faces(free[rows, :], res, left)
        if np.isfinite(xs).any():
            x_face = float(np.nanmedian(xs))
        cols = band(x_face, left, w)
        ys = _faces(free[:, cols].T, res, bottom)
        if np.isfinite(ys).any():
            y_face = float(np.nanmedian(ys))
    return ox + x_face, oy + y_face


def corner_pose(grid: Grid, corner, heading_deg, half_length, half_width, gap_m=0.0):
    """Pose (x, y, heading in degrees) of a robot standing square in ``corner`` with ``gap_m`` of free floor
    between its body and each of the two walls (0 = touching both). The heading is snapped to 0/90/180/270."""
    heading = (int(round(float(heading_deg) / 90.0)) % 4) * 90
    along_x = heading in (0, 180)
    ex, ey = (half_length, half_width) if along_x else (half_width, half_length)
    x_face, y_face = corner_faces(grid, corner)
    x = x_face + (gap_m + ex) * (1 if corner[1] == 'l' else -1)
    y = y_face + (gap_m + ey) * (1 if corner[0] == 'b' else -1)
    return float(x), float(y), float(heading)


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


def save_edited(env, grid: Grid, edit_dir=EDIT_DIR, scale=(1.0, 1.0)):
    """Save ``grid`` (internal bottom-first) as the edited map for ``env``. Returns the file path."""
    original, _ = load_env(env, scale)
    if grid.shape != original.shape:
        raise ValueError('Edited map must have the shape of the current map %s.' % (original.shape,))
    path = edited_path(env, edit_dir)
    _backup(path)  # keep the previous edited version instead of overwriting it
    k = ENV_QUARTER_TURNS.get(env, 0)
    _atomic_save_npy(path, np.flipud(np.rot90(grid.cells, -k)).astype(np.uint8))  # same orientation as the source files
    changed = int(np.count_nonzero(grid.cells != original.cells))
    meta = dict(environment=env, saved_at=time.strftime('%Y-%m-%d %H:%M:%S'), cells_changed_vs_original=changed,
                added_obstacle_cells=int(np.count_nonzero(grid.cells & ~original.cells)),
                removed_obstacle_cells=int(np.count_nonzero(~grid.cells & original.cells)),
                resolution_m=grid.resolution, orientation='row 0 = top (same as data/*.npy)',
                field_scale=[float(scale[0]), float(scale[1])])
    path.with_suffix('.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    return path


def load_edited(env, edit_dir=EDIT_DIR, scale=(1.0, 1.0)):
    """Return the saved edited Grid for ``env`` (stretched to the current field size) or None if there is none."""
    path = edited_path(env, edit_dir)
    if not path.exists():
        return None
    a = np.load(path, allow_pickle=False)
    original, _ = load_environment(env)
    if a.ndim != 2 or min(a.shape) < 8 or max(a.shape) > 400:
        raise ValueError('%s has shape %s, expected about %s.' % (path, a.shape, original.shape))
    grid = _turn(Grid(np.flipud(a), original.resolution, original.origin), ENV_QUARTER_TURNS.get(env, 0))
    return resample_to(grid, scaled_shape(original.shape, scale))   # saved with another field size: stretch it


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
