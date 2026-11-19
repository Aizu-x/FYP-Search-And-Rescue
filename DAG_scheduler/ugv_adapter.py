"""Connect the supplied A* and YOLOv5n implementations without motor commands.

The reference runner uses saved occupancy/images. The mission wrappers require
real map/pose inputs and request-linked UGV captures; neither invents arrival.
"""
from pathlib import Path
import hashlib
import json
import math
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from adapters import planning_output, result
from dag_scheduler.engine import ConfigurationError, InvalidResult, atomic_json


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def planner_settings(values=None):
    from ugv_planner.config import Footprint, PlannerConfig
    values = values or {}
    allowed = {"radius_m", "margin_m", "connectivity", "clearance_weight", "preferred_clearance_m"}
    if not isinstance(values, dict) or set(values) - allowed:
        raise ConfigurationError("Unsupported planner setting")
    return PlannerConfig(
        connectivity=values.get("connectivity", 8),
        footprint=Footprint(values.get("radius_m", 0.143), values.get("margin_m", 0.02), verified=False),
        inflation=True, clearance_weight=values.get("clearance_weight", 4.0),
        preferred_clearance_m=values.get("preferred_clearance_m", 0.20))


def calculate_route(map_data, start, goal, settings=None):
    """Use the teammate's original A*, then check exact endpoint connectors."""
    import numpy as np
    from contracts import inside, map_grid
    from ugv_planner.grid import Grid
    from ugv_planner.interfaces import NavigationService
    from ugv_nav.path_processing import path_length, validate_path
    map_grid(map_data, "blocked_grid", {0, 1})
    inside(start, map_data, "Start")
    inside(goal, map_data, "Goal")
    grid = Grid(np.asarray(map_data["blocked_grid"]), map_data["resolution_m"], tuple(map_data["origin_m"]))
    config = planner_settings(settings)
    route = NavigationService(grid, tuple(start), tuple(goal), config).replan()
    if not route.found:
        raise InvalidResult(f"A* did not find a usable route: {route.status}: {route.message}")
    points = [list(p) for p in route.waypoints]
    if points[-1] != list(goal):
        points.append(list(goal))
    valid, reason = validate_path(points, grid, config.planning_radius)
    if not valid:
        raise InvalidResult("A* route rejected: " + reason)
    return {"waypoints_m": points, "distance_m": path_length(points),
            "planner": "ugv_planner.astar.plan", "planner_status": route.status,
            "expanded_nodes": route.expanded_nodes, "planning_ms": route.computation_ms,
            "planning_radius_m": config.planning_radius, "collision_checked": True,
            "goal_connector_checked": True, "robot_dimensions_physically_verified": False}


def plan_path(inputs, context):
    from contracts import inside, map_identity, validate_pose_age
    grid, pose, target = (inputs[k] for k in ("prepare_planning_map", "get_ugv_pose", "select_target"))
    map_identity(pose, grid)
    map_identity(target, grid)
    validate_pose_age(pose, time.time(), context.parameters.get("max_pose_age_s", 5.0))
    calculated = calculate_route(grid, pose["position_m"], target["position_m"], context.parameters.get("planner"))
    output = planning_output(inputs, context, calculated.pop("waypoints_m"))
    output.update(calculated)
    atomic_json(Path(context.directory) / "planned_route.json", output)
    return output


def detect_images(images, context, *, input_kind):
    """Finite batch of BGR images; actual ONNX inference, no pixel-to-map guess."""
    import cv2
    from ugv_nav.vision import load_detector
    if not isinstance(images, list) or not images:
        raise InvalidResult("YOLO requires at least one image")
    options = context.parameters.get("yolo", {})
    model = Path(options.get("model") or ROOT / "third_party/wro_yolo/models/best.onnx").resolve()
    confidence = options.get("confidence", 0.25)
    if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 < confidence < 1:
        raise ConfigurationError("YOLO confidence must be between 0 and 1")
    detector = load_detector(model, conf_threshold=confidence)
    frames = []
    for item in images:
        if context.seconds_left() <= 0:
            raise TimeoutError("YOLO image batch exceeded its time limit")
        path = Path(item["image_ref"]).resolve()
        image_hash = sha256(path)
        if item.get("sha256") and item["sha256"] != image_hash:
            raise InvalidResult("Inspection image differs from its capture hash")
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise InvalidResult(f"Cannot decode inspection image: {path}")
        started = time.monotonic()
        detections = detector.predict(image)
        height, width = image.shape[:2]
        for detection in detections:
            validate_detection(detection, width, height)
        frames.append({"image_ref": str(path), "sha256": image_hash,
                       "frame_id": item.get("frame_id", path.name),
                       "capture_timestamp_unix_s": item.get("capture_timestamp_unix_s"),
                       "width": width, "height": height, "detections": detections,
                       "inference_s": time.monotonic() - started})
    count = sum(len(frame["detections"]) for frame in frames)
    output = result(context, detector="YOLOv5n", runtime="ONNX Runtime CPU",
                    model_ref=str(model), model_sha256=sha256(model), confidence_threshold=confidence,
                    nms_iou_threshold=0.45, input_kind=input_kind, frames=frames,
                    detection_count=count, image_count=len(frames), coordinates="original_image_pixels",
                    inspection_outcome="detections_present" if count else "no_detections",
                    detections_projected_to_map=False, robot_motion_executed=False)
    atomic_json(Path(context.directory) / "detections.json", output)
    return output


def validate_detection(detection, width, height):
    classes = ("green_obstacle", "red_obstacle")
    cls = detection.get("class_id")
    if type(cls) is not int or cls not in (0, 1) or detection.get("class_name") != classes[cls]:
        raise InvalidResult("Detector returned an unknown class")
    score = detection.get("confidence")
    if type(score) not in (int, float) or not math.isfinite(score) or not 0 <= score <= 1:
        raise InvalidResult("Detector returned invalid confidence")
    box = detection.get("bbox_xyxy")
    if not isinstance(box, list) or len(box) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in box):
        raise InvalidResult("Detector returned an invalid box")
    x1, y1, x2, y2 = box
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise InvalidResult("Detection box is outside its image")


def inspection_frames(navigation, now, max_age=30):
    """Do not use unrelated saved example images as a real target inspection."""
    if navigation.get("arrived") is not True or not navigation.get("request_id"):
        raise InvalidResult("Inspection needs confirmed arrival and its navigation request ID")
    arrival = navigation.get("arrival_observed_at_unix_s")
    if type(arrival) not in (int, float) or not math.isfinite(arrival) or arrival > now + 1:
        raise InvalidResult("Inspection needs an actual arrival timestamp")
    if type(max_age) not in (int, float) or not math.isfinite(max_age) or max_age <= 0:
        raise ConfigurationError("inspection_max_age_s must be positive and finite")
    frames = navigation.get("inspection_frames")
    if not isinstance(frames, list) or not frames:
        raise InvalidResult("Navigation must supply a finite batch of UGV camera frames after arrival")
    for frame in frames:
        captured = frame.get("capture_timestamp_unix_s")
        if (frame.get("source") != "ugv_camera" or frame.get("navigation_request_id") != navigation["request_id"]
                or not frame.get("sha256") or type(captured) not in (int, float) or not math.isfinite(captured)
                or not arrival <= captured <= now + 1 or now - captured > max_age):
            raise InvalidResult("UGV inspection frame is stale, unmatched, or captured before arrival")
    return frames


def inspect_target(inputs, context):
    navigation = inputs["navigate"]
    frames = inspection_frames(navigation, time.time(), context.parameters.get("inspection_max_age_s", 30))
    detections = detect_images(frames, context, input_kind="request_linked_ugv_camera")
    # Recheck freshness after model load/inference, before completing the task.
    inspection_frames(navigation, time.time(), context.parameters.get("inspection_max_age_s", 30))
    output = result(context, **{k: navigation[k] for k in ("map_id", "map_version", "frame", "target_id", "path_id")},
                    position_m=navigation["final_position_m"], inspection_complete=True,
                    finding=detections["inspection_outcome"], detector="YOLOv5n",
                    detections_ref=str(Path(context.directory) / "detections.json"),
                    detection_count=detections["detection_count"])
    return output


def validate_output(output, inputs, context):
    from contracts import validate
    validate(output, inputs, context)
    if context.task_id == "plan_path":
        from ugv_planner.grid import Grid
        from ugv_nav.path_processing import validate_path
        import numpy as np
        data = inputs["prepare_planning_map"]
        grid = Grid(np.asarray(data["blocked_grid"]), data["resolution_m"], tuple(data["origin_m"]))
        radius = planner_settings(context.parameters.get("planner")).planning_radius
        valid, reason = validate_path(output["waypoints_m"], grid, radius)
        if not valid or output.get("planning_radius_m") != radius:
            raise InvalidResult("Planner output failed independent route validation: " + reason)
    elif context.task_id == "inspect_target":
        path = Path(output["detections_ref"]).resolve()
        if not path.is_relative_to(Path(context.directory).resolve()) or not path.is_file():
            raise InvalidResult("Inspection data must belong to this task")
