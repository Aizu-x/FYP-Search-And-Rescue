"""Helpers for thin wrappers around colleagues' code. No algorithms here."""
from uuid import uuid4
from dag_scheduler.engine import InvalidResult


def result(context, **data):
    """Envelope for a synchronous computation performed for this exact request.

    For network responses, call check_reply FIRST; do not relabel a delayed reply.
    """
    return {**data, "schema_version": 2, "simulated": context.mode == "demo",
            "attempt_id": context.run_id}


def check_reply(reply, context):
    """A remote service must echo the ID sent with the original request."""
    if reply.get("request_id") != context.request_id:
        raise InvalidResult("Remote response belongs to a different task request")
    return reply


def planning_output(inputs, context, waypoints_m, status="PATH_FOUND"):
    grid = inputs["prepare_planning_map"]
    target = inputs["select_target"]
    pose = inputs["get_ugv_pose"]
    return result(context, **{key: grid[key] for key in ("map_id", "map_version", "frame")},
                  target_id=target["target_id"], path_id=uuid4().hex,
                  status=status, waypoints_m=waypoints_m,
                  pose_observed_at_unix_s=pose["observed_at_unix_s"])


def navigation_output(inputs, context, arrived, final_position_m):
    path = inputs["plan_path"]
    return result(context, **{key: path[key] for key in ("map_id", "map_version", "frame", "target_id", "path_id")},
                  arrived=arrived, final_position_m=final_position_m)
