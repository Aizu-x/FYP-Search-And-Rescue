"""Component integration using real saved inputs, without commanding a robot."""
from pathlib import Path
import json
import math

from adapters import result
from dag_scheduler.engine import InvalidResult, atomic_json
from ugv_adapter import ROOT, calculate_route, detect_images, sha256, planner_settings


def load_navigation_inputs(inputs, context):
    import numpy as np
    from ugv_nav.map_store import ENV_QUARTER_TURNS
    from ugv_planner.grid import Grid
    from ugv_planner.config import EXAMPLES
    options = context.parameters["reference"]
    environment = options["environment"]
    if environment not in EXAMPLES:
        raise InvalidResult("Unknown reference environment")
    path = ROOT / "data" / f"{environment}_occupancy.npy"
    # Load occupancy only. GT is not an input to A* or endpoint selection.
    raw = np.load(path, allow_pickle=False)
    if raw.shape != (100, 100) or not np.isin(raw, [0, 1]).all():
        raise InvalidResult("Reference occupancy must be a 100x100 binary map")
    turns = ENV_QUARTER_TURNS[environment]
    grid = Grid(np.rot90(np.flipud(raw), turns).astype(np.uint8), 0.03)
    defaults = []
    for point in EXAMPLES[environment]:
        mark = np.zeros(raw.shape, dtype=np.uint8)
        # Original example positions use the bottom-first, unrotated grid.
        mark[Grid(np.zeros(raw.shape), 0.03).world_to_cell(point)] = 1
        r, c = np.argwhere(np.rot90(mark, turns))[0]
        defaults.append(list(grid.cell_to_world((int(r), int(c)))))
    output = result(context, map_id=f"{environment}_{sha256(path)[:12]}", map_version=1, frame="arena",
                    width=grid.shape[1], height=grid.shape[0], resolution_m=grid.resolution,
                    origin_m=list(grid.origin), blocked_grid=grid.cells.astype(int).tolist(), inflation_radius_m=0.0,
                    start_m=options.get("start_m") or defaults[0], goal_m=options.get("goal_m") or defaults[1],
                    start_is_measured=False, metric_scale_physically_verified=False,
                    source_ref=str(path), source_sha256=sha256(path), environment=environment,
                    source_kind="supplied_saved_occupancy", derived_from_current_orb=False,
                    row_order="bottom_first_y_increases", orientation_quarter_turns=turns,
                    navigation_ready=False)
    atomic_json(Path(context.directory) / "navigation_inputs.json", output)
    return output


def plan_reference_route(inputs, context):
    data = inputs["load_navigation_inputs"]
    route = calculate_route(data, data["start_m"], data["goal_m"], context.parameters.get("planner"))
    output = result(context, **route, map_id=data["map_id"], map_version=data["map_version"],
                    frame=data["frame"], navigation_ready=False, robot_motion_executed=False,
                    start_is_measured=False, derived_from_current_orb=False)
    atomic_json(Path(context.directory) / "planned_route.json", output)
    return output


def inspect_images(inputs, context):
    images = context.parameters["reference"].get("images")
    if images:
        kind = "user_supplied_saved_images"
        paths = [Path(p) for p in images]
    else:
        kind = "repository_annotated_example_images"
        paths = sorted((ROOT / "third_party/wro_yolo/examples").glob("*.jpeg"))
    return detect_images([{"image_ref": str(p), "frame_id": p.name} for p in paths], context, input_kind=kind)


def write_integration_report(inputs, context):
    slam, nav, plan, vision = (inputs[k] for k in ("run_slam", "load_navigation_inputs", "plan_reference_route", "inspect_images"))
    output = result(context, status="COMPONENT_INTEGRATION_SUCCEEDED", scope="reference_components_only",
                    orb={"execution_kind": slam["execution_kind"], "fresh_slam_run": slam["fresh_slam_run"],
                         "keyframes": slam["keyframe_count"], "map_points": slam["map_point_count"],
                         "poses_ref": slam["poses_ref"], "geometry_ref": slam["geometry_ref"],
                         "coordinate_units": slam["coordinate_units"]},
                    planning={"environment": nav["environment"], "map_id": nav["map_id"],
                              "source": nav["source_kind"], "grid_size": [nav["height"], nav["width"]],
                              "resolution_m": nav["resolution_m"], "waypoints": len(plan["waypoints_m"]),
                              "distance_m": plan["distance_m"], "planning_radius_m": plan["planning_radius_m"],
                              "collision_checked": plan["collision_checked"], "derived_from_current_orb": False},
                    vision={"detector": vision["detector"], "input_kind": vision["input_kind"],
                            "images": vision["image_count"], "detections": vision["detection_count"],
                            "coordinates": vision["coordinates"], "model_sha256": vision["model_sha256"]},
                    robot_motion_executed=False, navigation_ready=False, orb_to_grid_fusion_implemented=False,
                    remaining=["Fresh native ORB run on Linux/WSL", "Camera calibration and metric alignment",
                               "ORB/semantic-map fusion", "Request-linked UGV pose, arrival, capture and stop adapters",
                               "Physical MentorPi validation"])
    atomic_json(Path(context.directory) / "combined_summary.json", output)
    return output


def validate_output(output, inputs, context):
    if output.get("attempt_id") != context.run_id or output.get("schema_version") != 2 or output.get("simulated") is not False:
        raise InvalidResult("Combined output has the wrong execution identity")
    if context.task_id == "load_navigation_inputs":
        from contracts import map_grid, inside
        map_grid(output, "blocked_grid", {0, 1})
        inside(output["start_m"], output, "Reference start")
        inside(output["goal_m"], output, "Reference goal")
        if output.get("derived_from_current_orb") is not False or output.get("start_is_measured") is not False:
            raise InvalidResult("Reference inputs must not claim measured localization or ORB fusion")
    elif context.task_id == "plan_reference_route":
        import numpy as np
        from ugv_planner.grid import Grid
        from ugv_nav.path_processing import validate_path
        data = inputs["load_navigation_inputs"]
        grid = Grid(np.asarray(data["blocked_grid"]), data["resolution_m"], tuple(data["origin_m"]))
        points = output["waypoints_m"]
        valid, reason = validate_path(points, grid, planner_settings(context.parameters.get("planner")).planning_radius)
        if not valid or points[0] != data["start_m"] or points[-1] != data["goal_m"] or output.get("map_id") != data["map_id"]:
            raise InvalidResult("Invalid reference route: " + reason)
    elif context.task_id == "inspect_images":
        from ugv_adapter import validate_detection
        if output.get("detector") != "YOLOv5n" or output.get("detections_projected_to_map") is not False:
            raise InvalidResult("Detector model/coordinate claims do not match supplied code")
        frames = output["frames"]
        if not frames or output["image_count"] != len(frames) or output["detection_count"] != sum(len(f["detections"]) for f in frames):
            raise InvalidResult("Detection counts do not match images")
        for frame in frames:
            for detection in frame["detections"]:
                validate_detection(detection, frame["width"], frame["height"])
    elif context.task_id == "write_integration_report":
        if (output.get("robot_motion_executed") is not False or output.get("navigation_ready") is not False
                or output.get("orb_to_grid_fusion_implemented") is not False):
            raise InvalidResult("Component report must not claim completed physical navigation")
    else:
        raise InvalidResult("Unexpected combined task")
