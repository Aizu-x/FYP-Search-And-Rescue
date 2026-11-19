import asyncio
import os
from pathlib import Path
import time
from dag_scheduler import ReplanRequested


def ok(inputs, context):
    return {"ok": True, "seen": list(inputs), "nested": {"value": 1},
            "request_id": context.request_id, "attempt": context.task_attempt,
            "replan_event": context.parameters.get("replan_event")}


async def async_ok(inputs, context):
    await asyncio.sleep(0)
    return {"ok": True}


def fail(inputs, context):
    raise RuntimeError("Expected fixture failure")


def crash(inputs, context):
    os._exit(3)


def fail_once(inputs, context):
    if context.task_attempt == 1:
        raise RuntimeError("Temporary fixture failure")
    return ok(inputs, context)


def hang(inputs, context):
    if context.parameters.get("marker"):
        Path(context.parameters["marker"]).write_text("started", encoding="utf-8")
    time.sleep(60)
    return {"ok": True}


def stop(inputs, context):
    return {"stopped": True, "request_id": context.request_id, "reason": context.stop_reason}


def stop_false(inputs, context):
    return {"stopped": False, "request_id": context.request_id}


def stop_stale(inputs, context):
    return {"stopped": True, "request_id": "previous-request"}


def replan(inputs, context):
    if context.mission_attempt == 1:
        raise ReplanRequested("Expected test obstacle", {"blocked_cell": [4, 8]})
    return ok(inputs, context)


def reject(output, inputs, context):
    raise ValueError("Rejected fixture output")


def reject_before(inputs, context):
    raise ValueError("Rejected inputs before the operation")


def nan(inputs, context):
    return {"bad": float("nan")}


def wrong_type(inputs, context):
    return [1, 2]


def wrong_arguments(one):
    return {}


def mutate(inputs, context):
    inputs["root"]["nested"]["value"] = 99
    return {"ok": True}
