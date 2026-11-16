"""Insert LiDAR-detected obstacles into the edge-side occupancy grid (never into source files)."""
from __future__ import annotations

import numpy as np

from ugv_planner.grid import Grid

from .localization import transform_point


def mark_scan_obstacles(grid: Grid, robot_pose, points, max_range_m=1.0, protect_radius_m=0.0):
    """Return (new_grid, newly_occupied_count).

    Points (robot frame) within ``max_range_m`` are transformed to the map and
    their cells marked occupied. Cells within ``protect_radius_m`` of the robot
    are never marked (avoids self-returns blocking the robot's own cell).
    """
    cells = grid.cells.copy()
    added = 0
    rx, ry = robot_pose[0], robot_pose[1]
    for p in points or []:
        if p[0] * p[0] + p[1] * p[1] > max_range_m * max_range_m:
            continue
        wx, wy = transform_point(robot_pose, p)
        if (wx - rx) ** 2 + (wy - ry) ** 2 <= protect_radius_m ** 2:
            continue
        cell = grid.world_to_cell((wx, wy))
        if grid.contains(cell) and not cells[cell]:
            cells[cell] = True
            added += 1
    if not added:
        return grid, 0
    return Grid(cells.astype(np.uint8), grid.resolution, grid.origin), added
