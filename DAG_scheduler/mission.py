"""The FYP graph and the outer controller for bounded replanning."""
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
from uuid import uuid4

from dag_scheduler import Scheduler, Task
from dag_scheduler.engine import ConfigurationError, atomic_json


DEPENDENCIES = {
    "prepare_recording": (),
    "run_slam": ("prepare_recording",),
    "run_erfnet": ("run_slam",),
    "fuse_map": ("run_slam", "run_erfnet"),
    "prepare_planning_map": ("fuse_map",),
    "select_target": ("fuse_map",),
    "get_ugv_pose": ("prepare_planning_map",),
    "plan_path": ("prepare_planning_map", "select_target", "get_ugv_pose"),
    "navigate": ("plan_path",),
    "inspect_target": ("navigate",),
    "save_report": ("inspect_target",),
}


def report_summary(inputs, context):
    """Prepare the completed inspection summary. The runner writes the report."""
    return {"schema_version": 2, "simulated": context.mode == "demo",
            "attempt_id": context.run_id, "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "mission_status": "completed", "inspection": inputs["inspect_target"]}


def build_tasks(mode, bindings=None, scenario="success"):
    bindings = bindings or {}
    handlers = bindings.get("handlers", {})
    timeouts = bindings.get("timeouts_s", {})
    if not isinstance(handlers, dict) or not isinstance(timeouts, dict):
        raise ConfigurationError("handlers and timeouts_s must be dictionaries")
    unknown = (set(handlers) | set(timeouts)) - set(DEPENDENCIES)
    if unknown:
        raise ConfigurationError("Unknown task names in bindings: " + ", ".join(sorted(unknown)))
    tasks = []
    for name, needs in DEPENDENCIES.items():
        handler = "demo_modules:execute" if mode == "demo" else handlers.get(name)
        if name == "save_report":
            handler = "mission:report_summary"
        if mode == "real" and isinstance(handler, str) and handler.startswith("demo_modules:"):
            raise ConfigurationError("Demo modules cannot be used in real mode")
        stop = ("demo_modules:stop_navigation" if mode == "demo" else bindings.get("stop_navigation")) if name == "navigate" else None
        if mode == "real" and name == "run_slam" and handler == "team_adapters:run_slam":
            stop = "orb_adapter:stop_slam"
        if mode == "real" and isinstance(stop, str) and stop.startswith("demo_modules:"):
            raise ConfigurationError("Real navigation needs a real stop acknowledgement")
        timeout = timeouts.get(name, 300.0 if mode == "real" and name == "run_slam" else 120.0 if name == "navigate" else 60.0)
        if mode == "demo" and name == "navigate" and scenario == "timeout":
            timeout = 0.75
        validator = "orb_adapter:validate_output" if mode == "real" and handler in ("team_adapters:prepare_recording", "team_adapters:run_slam") else "contracts:validate"
        if mode == "real" and handler in ("team_adapters:plan_path", "team_adapters:inspect_target"):
            validator = "ugv_adapter:validate_output"
        tasks.append(Task(name, needs, handler, validator, before="contracts:before",
                          timeout_s=timeout, on_stop=stop, requires_stop=(name == "navigate" or stop == "orb_adapter:stop_slam"),
                          stop_timeout_s=15.0 if name == "run_slam" else 10.0))
    return tasks


def run_mission(tasks, output_root, *, mode="demo", parameters=None, max_replans=0,
                cancellation=None, emit=print):
    if type(max_replans) is not int or max_replans < 0:
        raise ConfigurationError("max_replans must be a nonnegative integer")
    scheduler = Scheduler(tasks)
    scheduler.preflight()
    cancellation = cancellation if cancellation is not None else threading.Event()
    parameters = dict(parameters or {})
    mission_id = uuid4().hex
    folder = Path(output_root).resolve() / f"mission_{mission_id}"
    folder.mkdir(parents=True, exist_ok=False)
    summary = {"mission_id": mission_id, "mode": mode, "status": "RUNNING", "attempts": []}
    atomic_json(folder / "mission.json", summary)
    for number in range(1, max_replans + 2):
        report = scheduler.run(folder / f"attempt_{number}", mode=mode, mission_id=mission_id,
                               mission_attempt=number, parameters=parameters,
                               cancellation=cancellation, emit=emit)
        summary["attempts"].append({"number": number, "run_id": report["run_id"],
                                    "status": report["status"], "report": f"attempt_{number}/report.json",
                                    "duration_s": report["duration_s"],
                                    "intervention_required": report["intervention_required"]})
        summary["status"] = report["status"]
        atomic_json(folder / "mission.json", summary)
        if (report["status"] != "REPLAN_REQUIRED" or report["intervention_required"]
                or cancellation.is_set() or number > max_replans):
            break
        parameters["replan_event"] = report.get("replan_event", {})
        if emit:
            emit("Starting a fresh DAG attempt with new inputs; no old task results are reused.")
    return summary, folder
