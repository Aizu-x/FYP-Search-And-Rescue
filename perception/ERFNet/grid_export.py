"""Functions extracted unchanged from the existing ERFNet map-export pipeline.
Only file/module placement changed for this three-module handoff."""
from __future__ import annotations
import numpy as np
SEMANTIC_IDS=(0,1,2,255)
OCCUPANCY_IDS=(0,1,255)


def validate_labels(array: np.ndarray, allowed=SEMANTIC_IDS) -> np.ndarray:
    a = np.asarray(array)
    if a.ndim != 2 or 0 in a.shape or a.dtype.kind not in "iu":
        raise ValueError("Expected a 2-D integer label mask, not a photo/overlay/RGB preview")
    invalid = np.setdiff1d(np.unique(a), allowed)
    if invalid.size:
        raise ValueError(f"Unexpected label IDs: {invalid.tolist()}; expected {list(allowed)}")
    return a.astype(np.uint8, copy=False)

def aggregate_semantics(mask: np.ndarray, shape=(60, 60)) -> tuple[np.ndarray, np.ndarray]:
    """Any-overlap occupied pooling; no interpolation or majority voting.

    A destination cell is occupied if ANY source pixel with positive-area
    overlap is blocked/obstacle. Otherwise ANY unknown makes it unknown; only
    an entirely known-free cell becomes free. Source pixels at fractional
    cell boundaries can conservatively occupy both neighboring cells.
    Coarse semantic colors are for display: obstacle takes priority over rail.
    """
    a = validate_labels(mask)
    rows, cols = shape
    if (not isinstance(rows, int) or not isinstance(cols, int) or
            rows <= 0 or cols <= 0 or rows > a.shape[0] or cols > a.shape[1]):
        raise ValueError("Target shape must be positive integers and no larger than the source")
    h, w = a.shape
    occ = np.empty((rows, cols), dtype=np.uint8)
    semantic = np.empty_like(occ)
    for r in range(rows):
        r0, r1 = r * h // rows, ((r + 1) * h + rows - 1) // rows
        for c in range(cols):
            c0, c1 = c * w // cols, ((c + 1) * w + cols - 1) // cols
            block = a[r0:r1, c0:c1]
            if np.any(block == 2):
                occ[r, c], semantic[r, c] = 1, 2
            elif np.any(block == 1):
                occ[r, c], semantic[r, c] = 1, 1
            elif np.any(block == 255):
                occ[r, c], semantic[r, c] = 255, 255
            else:
                occ[r, c], semantic[r, c] = 0, 0
    return occ, semantic
