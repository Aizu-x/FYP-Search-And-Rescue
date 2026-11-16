"""Independent GT evaluation; never changes a plan or its endpoints."""
import math
import numpy as np
from .collision import CollisionChecker, EPS


def evaluate(result, gt, footprint):
    report = dict(path_found=result.found, planner_status=result.status,
                  path_length_m=result.distance_m if result.found else None,
                  planning_ms=result.computation_ms, expanded_nodes=result.expanded_nodes,
                  objective_cost_cells=result.objective_cost_cells if result.found else None,
                  turns=None, detour_ratio=None, minimum_gt_clearance_cm=None,
                  minimum_footprint_clearance_cm=None, gt_center_collision=None,
                  gt_footprint_collision=None, unsafe_segments=[],
                  validation='Invalid start or goal' if result.status == 'invalid' else 'No path available',
                  explanation=result.message)
    if not result.found:
        return report
    points = result.waypoints
    checker = CollisionChecker(gt)
    segments = list(zip(points, points[1:])) or [(points[0], points[0])]
    clearances = [checker.clearance(a, b) for a, b in segments]
    minimum = min(clearances)
    radius = footprint.radius_m
    required = footprint.effective_radius
    unsafe = [i for i, clearance in enumerate(clearances) if clearance <= required+EPS]
    center_hit = minimum <= EPS
    footprint_hit = None if radius is None else minimum <= radius+EPS
    directions = np.diff(np.asarray(points), axis=0)
    unit = directions / np.maximum(np.linalg.norm(directions, axis=1, keepdims=True), 1e-15)
    turns = sum(not np.allclose(a, b, atol=1e-9) for a, b in zip(unit, unit[1:]))
    straight = math.dist(points[0], points[-1])
    if unsafe:
        status = 'Path found but GT collision/clearance failed'
        explanation = 'Continuous centerline intersects GT or map boundary.' if center_hit else 'Configured footprint plus margin touches/intersects GT or map boundary.'
    elif radius is None or not footprint.verified:
        status = 'Safety undetermined: footprint not verified'
        explanation = 'No detected collision for configured geometry; measured bounding radius and assumptions must be verified.'
    else:
        status = 'Path found and GT-clearance passed (configured model)'
        explanation = 'Swept circular footprint plus margin clears GT; this does not establish real-world safety.'
    report.update(turns=int(turns), detour_ratio=result.distance_m/straight if straight else 1.0,
                  minimum_gt_clearance_cm=minimum*100,
                  minimum_footprint_clearance_cm=None if radius is None else (minimum-radius)*100,
                  gt_center_collision=bool(center_hit), gt_footprint_collision=None if footprint_hit is None else bool(footprint_hit),
                  unsafe_segments=unsafe, validation=status, explanation=explanation)
    return report


def simulation_preflight(result, occupancy, report, footprint):
    """Offline dispatch gate. GT may veto motion, never generate/repair routes."""
    if not result.found:
        return False, result.message
    if report['unsafe_segments']:
        return False, 'Simulation blocked: this route fails GT clearance. Improve the occupancy map or planning settings; GT will not repair it.'
    checker = CollisionChecker(occupancy)
    pairs = list(zip(result.waypoints, result.waypoints[1:])) or [(result.waypoints[0], result.waypoints[0])]
    if any(not checker.is_clear(a, b, footprint.effective_radius) for a, b in pairs):
        return False, 'Simulation blocked: configured footprint/margin intersects occupancy. Enable inflation or change the plan.'
    return True, 'Offline route checks passed for configured geometry; physical safety is not established.'
