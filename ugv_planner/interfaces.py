"""Deployment boundary. ROS2 is deliberately not a laptop dependency."""
from dataclasses import dataclass
from typing import Protocol, Sequence
import numpy as np
from .grid import Grid
from .astar import plan
from .config import PlannerConfig
from .collision import CollisionChecker


class MotionController(Protocol):
    def follow_waypoints(self, waypoints_m: Sequence[tuple[float, float]]) -> None: ...
    def stop(self) -> None: ...


@dataclass
class NavigationService:
    """Edge-grid input, localization pose, goal, and replanning interface.

    World poses may be between centers. A checked connector from the exact
    current pose is prepended, with no teleportation. Goal is a disclosed
    cell-center target. Caller must stop controller before any update/replan.
    """
    grid: Grid | None = None
    pose: tuple | None = None
    goal: tuple | None = None
    config: PlannerConfig = PlannerConfig()

    def receive_grid(self, grid):
        self.grid = grid

    def update_pose(self, pose):
        self.pose = tuple(pose)

    def set_goal(self, goal):
        self.goal = tuple(goal)

    def replan(self):
        if self.grid is None or self.pose is None or self.goal is None:
            raise ValueError('Map, localization pose and goal are required.')
        result = plan(self.grid, self.grid.world_to_cell(self.pose), self.grid.world_to_cell(self.goal), self.config)
        if result.found and tuple(self.pose) != tuple(result.waypoints[0]):
            checker = CollisionChecker(self.grid)
            if not checker.is_clear(self.pose, result.waypoints[0], self.config.planning_radius):
                result.status = 'invalid'; result.message = 'Current pose cannot safely connect to its cell center.'
                result.cells = []; result.waypoints = []; result.distance_m = result.cost_cells = 0
            else:
                import math
                extra = math.dist(self.pose, result.waypoints[0])
                result.waypoints.insert(0, self.pose)
                result.distance_m += extra
                result.cost_cells += extra/self.grid.resolution
                # Fixed connector cost is independent of subsequent route choice.
                from .clearance import penalties
                penalty = penalties(self.grid, self.config)[self.grid.world_to_cell(self.pose)] if self.config.clearance_weight else 0
                result.objective_cost_cells += extra/self.grid.resolution*(1+self.config.clearance_weight*penalty)
        return result


def grid_from_ros_values(values, width, height, resolution, origin=(0., 0.), threshold=50):
    """Pure adapter for ROS OccupancyGrid data: row-major bottom-first;
    unknown (-1) is blocked. Rotated map origins must be transformed upstream.
    Actual ROS subscriptions/TF/controller transport remain untested integration work.
    """
    a = np.asarray(values)
    if a.size != width*height or not np.isin(a, np.arange(-1, 101)).all():
        raise ValueError('Invalid ROS occupancy payload.')
    if not 1 <= threshold <= 100:
        raise ValueError('Threshold must be between 1 and 100.')
    return Grid(((a < 0) | (a >= threshold)).reshape(height, width), resolution, origin)
