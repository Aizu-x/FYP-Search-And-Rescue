from dataclasses import dataclass
from pathlib import Path
import hashlib
import math
import numpy as np

DATA_DIR = Path(__file__).resolve().parents[1] / 'data'
FILES = {f'ENV{i}': (f'ENV{i}_occupancy.npy', 'ENV1_GT (1).npy' if i == 1 else f'ENV{i}_GT.npy') for i in range(1, 4)}


@dataclass(frozen=True)
class Grid:
    """Internal row 0 is bottom (world +y). Cells are closed occupied squares.

    origin is the lower-left map corner in meters, not a cell center.
    """
    cells: np.ndarray
    resolution: float = 0.03
    origin: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        a = np.asarray(self.cells)
        if a.ndim != 2 or min(a.shape) == 0 or not np.isin(a, [0, 1]).all():
            raise ValueError('Expected a nonempty 2-D binary occupancy array.')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Resolution must be finite and positive.')
        if len(self.origin) != 2 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError('Origin must contain two finite coordinates.')
        a = np.array(a, dtype=bool, copy=True); a.flags.writeable = False
        object.__setattr__(self, 'cells', a)

    @property
    def shape(self):
        return self.cells.shape

    @property
    def extent(self):
        x, y = self.origin; h, w = self.shape; s = self.resolution
        return (x, x + w*s, y, y + h*s)

    def contains(self, cell):
        r, c = cell
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1]

    def cell_to_world(self, cell):
        r, c = cell; x, y = self.origin; s = self.resolution
        return (x+(c+0.5)*s, y+(r+0.5)*s)

    def world_to_cell(self, point):
        x, y = point
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Coordinates must be finite.')
        return (math.floor((y-self.origin[1])/self.resolution), math.floor((x-self.origin[0])/self.resolution))

    def edited(self, cell, occupied):
        if not self.contains(cell):
            raise ValueError('Edit outside map.')
        a = self.cells.copy(); a[cell] = occupied
        return Grid(a, self.resolution, self.origin)


def load_environment(name, data_dir=DATA_DIR):
    if name not in FILES:
        raise ValueError(f'Unknown environment: {name}')
    arrays = [np.load(Path(data_dir)/p, allow_pickle=False) for p in FILES[name]]
    if any(a.shape != (100, 100) for a in arrays):
        raise ValueError('Dataset requires aligned 100 x 100 arrays.')
    # Image row order is top to bottom; flip in memory, never rewrite source data.
    return tuple(Grid(np.flipud(a)) for a in arrays)


def dataset_manifest(data_dir=DATA_DIR):
    return {p: hashlib.sha256((Path(data_dir)/p).read_bytes()).hexdigest()
            for pair in FILES.values() for p in pair}
