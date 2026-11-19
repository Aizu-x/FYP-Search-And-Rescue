"""Example results only. No perception, A*, YOLOv8 or robot control."""
import time
from contracts import CLASS_NAMES, SCHEMA_VERSION
from dag_scheduler import ReplanRequested

def prepare_recording(results):
    return {
        "recording_id": "demo_recording_001",
        "calibration_ref": "sim://camera-calibration",
        "frames": [
            {"frame_id": "frame_001", "timestamp_s": 0.0, "image_ref": "sim://frame-001"},
            {"frame_id": "frame_002", "timestamp_s": 0.1, "image_ref": "sim://frame-002"},
        ],
    }


def run_slam(results):
    recording = results["prepare_recording"]
    return {
        "recording_id": recording["recording_id"],
        "keyframe_ids": [frame["frame_id"] for frame in recording["frames"]],
        "poses_ref": "sim://camera-poses",
        "geometry_ref": "sim://sparse-geometry",
    }


def run_erfnet(results):
    slam = results["run_slam"]
    return {
        "recording_id": slam["recording_id"],
        "keyframe_ids": list(slam["keyframe_ids"]),
        "labels_ref": "sim://semantic-labels",
        "class_names": list(CLASS_NAMES),
    }


def fuse_map(results):
    # Synthetic test grid. This is not computed from camera data.
    grid = [[0 for column in range(60)] for row in range(60)]
    for row in range(20, 24):
        for column in range(20, 24):
            grid[row][column] = 1  # Obstacle in the new three-class encoding.
    return {
        "recording_id": results["run_slam"]["recording_id"],
        "map_id": "demo_map_001",
        "map_version": 1,
        "frame": "arena",
        "width": 60,
        "height": 60,
        "resolution_m": 0.05,
        "origin_m": [0.0, 0.0],
        "semantic_grid": grid,
        "alignment_ref": "sim://arena-alignment",
    }


def prepare_planning_map(results):
    semantic_map = results["fuse_map"]
    return {
        "simulated": True,
        "map_id": semantic_map["map_id"],
        "map_version": semantic_map["map_version"],
        "frame": semantic_map["frame"],
        "width": semantic_map["width"],
        "height": semantic_map["height"],
        "resolution_m": semantic_map["resolution_m"],
        "origin_m": list(semantic_map["origin_m"]),
        "blocked_grid": [[0 if cell == 0 else 1 for cell in row]
                         for row in semantic_map["semantic_grid"]],
        "inflation_radius_m": 0.0,  # No footprint inflation is performed in this demo.
    }


def select_target(results):
    semantic_map = results["fuse_map"]
    return {
        "simulated": True,
        "map_id": semantic_map["map_id"],
        "map_version": semantic_map["map_version"],
        "frame": semantic_map["frame"],
        "target_id": "inspection_location_001",
        "position_m": [1.00, 0.25],  # Predefined demo location, not detected.
    }


def get_ugv_pose(results):
    planning_map = results["prepare_planning_map"]
    return {
        "simulated": True,
        "map_id": planning_map["map_id"],
        "map_version": planning_map["map_version"],
        "frame": planning_map["frame"],
        "position_m": [0.25, 0.25],  # Predefined demo position, not a sensor.
        "yaw_rad": 0.0,
        "observed_at_unix_s": time.time(),
    }


def plan_path(results):
    planning_map = results["prepare_planning_map"]
    target = results["select_target"]
    pose = results["get_ugv_pose"]
    return {
        "simulated": True,
        "map_id": planning_map["map_id"],
        "map_version": planning_map["map_version"],
        "frame": planning_map["frame"],
        "target_id": target["target_id"],
        "pose_observed_at_unix_s": pose["observed_at_unix_s"],
        "path_id": f"demo_path_{planning_map['attempt_id']}",
        "status": "PATH_FOUND",
        # A preset two-point route: A* and collision checking have NOT run.
        "waypoints_m": [pose["position_m"], target["position_m"]],
        "description": "SIMULATED route, not validated for physical use",
    }


def navigate(results):
    path = results["plan_path"]
    if not path["waypoints_m"]:
        raise ValueError("Navigation requires a nonempty route")
    return {
        "simulated": True,
        "map_id": path["map_id"],
        "map_version": path["map_version"],
        "target_id": path["target_id"],
        "path_id": path["path_id"],
        "frame": path["frame"],
        "arrived": True,  # Simulated arrival acknowledgement, not movement.
        "final_position_m": path["waypoints_m"][-1],
    }


def inspect_target(results):
    navigation = results["navigate"]
    if not navigation["arrived"]:
        raise ValueError("Inspection requires arrival at the target")
    return {
        "simulated": True,
        "map_id": navigation["map_id"],
        "map_version": navigation["map_version"],
        "target_id": navigation["target_id"],
        "path_id": navigation["path_id"],
        "frame": navigation["frame"],
        "position_m": navigation["final_position_m"],
        "inspection_complete": True,
        "finding": "SIMULATED no victim detected; no camera or YOLO was used",
    }




def execute(inputs, context):
    scenario = context.parameters.get("scenario", "success")
    if context.task_id == "run_slam" and scenario == "fail_slam":
        raise RuntimeError("Demonstration: SLAM failed")
    if context.task_id == "navigate":
        if scenario == "timeout":
            time.sleep(60)
        if scenario == "replan" and context.mission_attempt == 1:
            raise ReplanRequested("Demonstration: a new obstacle requires a new map and route",
                                  {"frame": "arena", "blocked_cells": [[30, 30]],
                                   "observed_at_unix_s": time.time()})
    implementation = globals()[context.task_id]
    output = implementation(inputs)
    if context.task_id == "fuse_map":
        output["map_version"] = context.mission_attempt
        for row, column in context.parameters.get("replan_event", {}).get("blocked_cells", []):
            output["semantic_grid"][row][column] = 2
    if context.task_id == "get_ugv_pose" and context.mission_attempt > 1:
        output["position_m"] = [0.40, 0.25]
    if context.task_id == "plan_path" and scenario == "no_path":
        output.update(status="NO_PATH", waypoints_m=[])
    if context.task_id == "plan_path" and scenario == "bad_data":
        output["map_version"] = 999
    return {**output, "schema_version": SCHEMA_VERSION,
            "simulated": True, "attempt_id": context.run_id}


def stop_navigation(inputs, context):
    # This acknowledgement simulates a robot-controller response.
    return {"stopped": True, "request_id": context.request_id,
            "simulated": True, "reason": context.stop_reason}
