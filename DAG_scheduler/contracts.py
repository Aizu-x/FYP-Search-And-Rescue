"""FYP handoff checks for three-class maps; not a geometric safety certificate.

Arena grids carry explicit dimensions, resolution, and origin.
Real mode expects simulated=False. Both modes require current run/map identities.
Update these proposed formats together with the team's adapters when needed.
"""

import math
import time
from dag_scheduler.engine import InvalidResult


CLASS_NAMES = ["free space", "obstacle", "blocked"]
SCHEMA_VERSION = 2  # Three-class encoding; do not silently accept old five-class data.
MAX_POSE_AGE_S = 5.0  # Demo setting; must be reviewed for a real robot.
POSITION_TOLERANCE_M = 0.05  # Demo setting, not a validated arrival tolerance.


class ContractError(InvalidResult):
    """A module returned data that cannot be handed to its successor."""


def require(condition, message):
    if not condition:
        raise ContractError(message)


def text(value, name):
    require(isinstance(value, str) and bool(value.strip()), f"{name} must be nonempty text")


def number(value, name):
    require(type(value) in (int, float) and math.isfinite(value),
            f"{name} must be a finite number")


def position(value, name):
    require(isinstance(value, list) and len(value) == 2, f"{name} must be [x, y] in metres")
    for coordinate in value:
        number(coordinate, name)


def field(data, name):
    require(name in data, f"Missing field: {name}")
    return data[name]


def same(actual, expected, name):
    require(actual == expected, f"{name} does not match the required input")


def map_identity(value, expected):
    version = field(value, "map_version")
    require(type(version) is int and version >= 1, "map_version must be a positive integer")
    for key in ("map_id", "map_version", "frame"):
        same(field(value, key), expected[key], key)


def inside(point, map_data, name):
    position(point, name)
    x, y = point
    ox, oy = map_data["origin_m"]
    resolution = map_data["resolution_m"]
    require(ox <= x < ox + map_data["width"] * resolution
            and oy <= y < oy + map_data["height"] * resolution,
            f"{name} lies outside the map")


def near(actual, expected, name, tolerance_m=POSITION_TOLERANCE_M):
    position(actual, name)
    number(tolerance_m, "tolerance_m")
    require(tolerance_m > 0, "tolerance_m must be positive")
    require(math.dist(actual, expected) <= tolerance_m + 1e-9,
            f"{name} is too far from the expected position")


def map_grid(value, grid_key, permitted):
    text(field(value, "map_id"), "map_id")
    version = field(value, "map_version")
    require(type(version) is int and version >= 1, "map_version must be a positive integer")
    same(field(value, "frame"), "arena", "frame")
    for dimension in ("width", "height"):
        require(type(field(value, dimension)) is int and 1 <= value[dimension] <= 10000,
                f"{dimension} must be a positive integer no larger than 10000")
    number(field(value, "resolution_m"), "resolution_m")
    require(value["resolution_m"] > 0, "resolution_m must be positive")
    position(field(value, "origin_m"), "origin_m")
    grid = field(value, grid_key)
    require(isinstance(grid, list) and len(grid) == value["height"],
            f"{grid_key} row count must match height")
    for row in grid:
        require(isinstance(row, list) and len(row) == value["width"],
                f"{grid_key} column count must match width")
        require(all(type(cell) is int and cell in permitted for cell in row),
                f"{grid_key} contains an unsupported cell value")


def validate_pose_age(pose, now, max_age_s=MAX_POSE_AGE_S):
    observed = field(pose, "observed_at_unix_s")
    number(observed, "observed_at_unix_s")
    age = now - observed
    number(max_age_s, "max_pose_age_s")
    require(max_age_s > 0, "max_pose_age_s must be positive")
    require(-1.0 <= age <= max_age_s,
            "UGV pose is stale or its timestamp is too far in the future")


def validate_task_output(task, value, results, attempt_id, now=None, simulated=True,
                         max_pose_age_s=MAX_POSE_AGE_S, tolerance_m=POSITION_TOLERANCE_M):
    """Raise before the scheduler saves a bad result or releases successors."""
    require(isinstance(value, dict), f"{task} must return a dictionary")
    same(field(value, "schema_version"), SCHEMA_VERSION, "schema_version")
    require(type(value["schema_version"]) is int, "schema_version must be an integer")
    require(field(value, "simulated") is simulated,
            "Result simulation flag does not match the selected execution mode")
    same(field(value, "attempt_id"), attempt_id, "attempt_id")
    now = time.time() if now is None else now

    if task == "plan_path":
        same(field(value, "pose_observed_at_unix_s"),
             results["get_ugv_pose"]["observed_at_unix_s"], "pose_observed_at_unix_s")

    if task == "prepare_recording":
        text(field(value, "recording_id"), "recording_id")
        text(field(value, "calibration_ref"), "calibration_ref")
        frames = field(value, "frames")
        require(isinstance(frames, list) and bool(frames), "frames must be a nonempty list")
        ids, previous_time = set(), -1.0
        for frame in frames:
            require(isinstance(frame, dict), "Each frame must be a dictionary")
            frame_id = field(frame, "frame_id")
            text(frame_id, "frame_id")
            require(frame_id not in ids, "Frame IDs must be unique")
            ids.add(frame_id)
            timestamp = field(frame, "timestamp_s")
            number(timestamp, "timestamp_s")
            require(timestamp >= 0 and timestamp > previous_time,
                    "Frame timestamps must be nonnegative and strictly increasing")
            previous_time = timestamp
            text(field(frame, "image_ref"), "image_ref")

    elif task == "run_slam":
        recording = results["prepare_recording"]
        same(field(value, "recording_id"), recording["recording_id"], "recording_id")
        frame_ids = [frame["frame_id"] for frame in recording["frames"]]
        keyframes = field(value, "keyframe_ids")
        require(isinstance(keyframes, list) and bool(keyframes), "SLAM must return keyframe IDs")
        require(all(isinstance(item, str) for item in keyframes), "Keyframe IDs must be strings")
        require(len(keyframes) == len(set(keyframes)), "Keyframe IDs must be unique")
        require(set(keyframes).issubset(frame_ids), "SLAM refers to an unknown recording frame")
        for key in ("poses_ref", "geometry_ref"):
            text(field(value, key), key)

    elif task == "run_erfnet":
        slam = results["run_slam"]
        same(field(value, "recording_id"), slam["recording_id"], "recording_id")
        same(field(value, "keyframe_ids"), slam["keyframe_ids"], "keyframe_ids")
        same(field(value, "class_names"), CLASS_NAMES, "class_names")
        text(field(value, "labels_ref"), "labels_ref")

    elif task == "fuse_map":
        map_grid(value, "semantic_grid", {-1, 0, 1, 2})
        same(field(value, "recording_id"), results["run_slam"]["recording_id"], "recording_id")
        same(value["recording_id"], results["run_erfnet"]["recording_id"], "recording_id")
        text(field(value, "alignment_ref"), "alignment_ref")

    elif task == "prepare_planning_map":
        map_grid(value, "blocked_grid", {0, 1})
        semantic_map = results["fuse_map"]
        map_identity(value, semantic_map)
        for key in ("width", "height", "resolution_m", "origin_m"):
            same(value[key], semantic_map[key], key)
        number(field(value, "inflation_radius_m"), "inflation_radius_m")
        require(value["inflation_radius_m"] >= 0, "inflation_radius_m cannot be negative")
        # Proposed conservative policy: only semantic class 0 is traversable.
        for labels, blocked in zip(semantic_map["semantic_grid"], value["blocked_grid"]):
            require(all(label == 0 or cell == 1 for label, cell in zip(labels, blocked)),
                    "Unknown or non-free semantic cells must be blocked")

    elif task == "select_target":
        semantic_map = results["fuse_map"]
        map_identity(value, semantic_map)
        text(field(value, "target_id"), "target_id")
        inside(field(value, "position_m"), semantic_map, "Target position")

    elif task == "get_ugv_pose":
        planning_map = results["prepare_planning_map"]
        map_identity(value, planning_map)
        inside(field(value, "position_m"), planning_map, "UGV position")
        number(field(value, "yaw_rad"), "yaw_rad")
        validate_pose_age(value, now, max_pose_age_s)

    elif task == "plan_path":
        planning_map = results["prepare_planning_map"]
        target, pose = results["select_target"], results["get_ugv_pose"]
        map_identity(value, planning_map)
        map_identity(target, planning_map)
        map_identity(pose, planning_map)
        validate_pose_age(pose, now, max_pose_age_s)
        same(field(value, "target_id"), target["target_id"], "target_id")
        text(field(value, "path_id"), "path_id")
        require(field(value, "status") == "PATH_FOUND", "Planner did not find a route")
        points = field(value, "waypoints_m")
        require(isinstance(points, list) and bool(points), "Route must contain at least one waypoint")
        for point in points:
            inside(point, planning_map, "Waypoint")
        near(points[0], pose["position_m"], "Route start", tolerance_m)
        near(points[-1], target["position_m"], "Route end", tolerance_m)

    elif task == "navigate":
        path = results["plan_path"]
        map_identity(value, path)
        for key in ("path_id", "target_id"):
            same(field(value, key), path[key], key)
        require(field(value, "arrived") is True, "Navigation has not confirmed arrival")
        near(field(value, "final_position_m"), path["waypoints_m"][-1], "Arrival position", tolerance_m)

    elif task == "inspect_target":
        navigation = results["navigate"]
        map_identity(value, navigation)
        for key in ("path_id", "target_id"):
            same(field(value, key), navigation[key], key)
        require(field(value, "inspection_complete") is True, "Inspection is not complete")
        near(field(value, "position_m"), navigation["final_position_m"], "Inspection position", tolerance_m)
        text(field(value, "finding"), "finding")

    elif task == "save_report":
        same(field(value, "mission_status"), "completed", "mission_status")
        same(field(value, "inspection"), results["inspect_target"], "inspection")
        text(field(value, "created_at_utc"), "created_at_utc")

    else:
        raise ContractError(f"No output contract is defined for {task}")


def validate(output, inputs, context):
    """Worker-side mission output validation, included in the task time limit."""
    validate_task_output(context.task_id, output, inputs, context.run_id,
                         simulated=(context.mode == "demo"),
                         max_pose_age_s=context.parameters.get("max_pose_age_s", MAX_POSE_AGE_S),
                         tolerance_m=context.parameters.get("arrival_tolerance_m", POSITION_TOLERANCE_M))


def before(inputs, context):
    """Reject stale planning information BEFORE calling the navigation adapter."""
    if context.task_id == "navigate":
        path = inputs["plan_path"]
        validate_pose_age({"observed_at_unix_s": field(path, "pose_observed_at_unix_s")}, time.time(),
                          context.parameters.get("max_pose_age_s", MAX_POSE_AGE_S))
