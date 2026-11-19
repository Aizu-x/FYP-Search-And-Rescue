"""A sequential DAG executor with isolated, time-limited task workers.

Only successful, validated outputs release successors. No algorithm or robot
implementation belongs in this file. Each task is an importable function.
Workers are separate processes so a hung Python/native function can be stopped.
Stopping a worker does NOT stop a remote robot: use a verified stop callback.
"""

from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
import importlib
import inspect
import json
import math
import multiprocessing as mp
from pathlib import Path
import re
import signal
import threading
import time
import traceback
from uuid import uuid4


class ConfigurationError(ValueError):
    pass


class InvalidResult(ValueError):
    pass


class ReplanRequested(Exception):
    """An adapter requests a fresh mission attempt with current map/pose data."""

    def __init__(self, reason, event=None):
        super().__init__(reason)
        if event is not None and not isinstance(event, dict):
            raise InvalidResult("Replanning event must be a dictionary")
        self.event = json.loads(json.dumps(event or {}, allow_nan=False))


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, value):
    """Never expose a partially written result as a completed result."""
    path = Path(path)
    temporary = path.with_name("write_" + uuid4().hex[:12] + ".tmp")
    try:
        temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def resolve(reference):
    if not isinstance(reference, str) or reference.count(":") != 1:
        raise ConfigurationError(f"Expected module:function, got {reference!r}")
    module, name = reference.split(":")
    try:
        value = getattr(importlib.import_module(module), name)
    except (ImportError, AttributeError) as error:
        raise ConfigurationError(f"Cannot load {reference}: {error}") from error
    if not callable(value):
        raise ConfigurationError(f"Not a function: {reference}")
    if inspect.iscoroutinefunction(value):
        raise ConfigurationError(f"{reference}: use a synchronous adapter that waits for a finite result")
    if getattr(value, "__dag_unconfigured__", False):
        raise ConfigurationError(f"Adapter still needs implementation: {reference}")
    return value


@dataclass(frozen=True)
class Task:
    name: str
    needs: tuple = ()
    handler: str | None = None
    validator: str | None = None
    before: str | None = None
    timeout_s: float = 60.0
    max_attempts: int = 1
    idempotent: bool = False
    retry_delay_s: float = 0.0
    on_stop: str | None = None
    requires_stop: bool = False
    stop_timeout_s: float = 10.0


@dataclass(frozen=True)
class TaskContext:
    mission_id: str
    run_id: str
    task_id: str
    request_id: str
    mission_attempt: int
    task_attempt: int
    mode: str
    deadline_monotonic: float
    directory: str
    parameters: dict = field(default_factory=dict)
    stop_reason: str | None = None
    operation_directory: str | None = None

    def seconds_left(self):
        return max(0.0, self.deadline_monotonic - time.monotonic())


def _graph(tasks):
    tasks = list(tasks)
    if any(not isinstance(task, Task) or not isinstance(task.name, str) for task in tasks):
        raise ConfigurationError("Every graph entry must be a Task with a string name")
    names = [task.name for task in tasks]
    if not tasks or len(set(names)) != len(names):
        raise ConfigurationError("The graph must contain tasks with unique names")
    for task in tasks:
        if not isinstance(task.name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", task.name):
            raise ConfigurationError(f"Invalid task name: {task.name!r}")
        if not isinstance(task.needs, (tuple, list)) or any(not isinstance(n, str) for n in task.needs):
            raise ConfigurationError(f"{task.name}: needs must list task names")
    successors = {name: [] for name in names}
    counts = {}
    for task in tasks:
        unique = tuple(dict.fromkeys(task.needs))
        counts[task.name] = len(unique)
        for prerequisite in unique:
            if prerequisite not in successors:
                raise ConfigurationError(f"{task.name}: unknown prerequisite {prerequisite}")
            successors[prerequisite].append(task.name)
    return tasks, successors, counts


def topological_order(tasks):
    """Kahn's algorithm. O(V + E) time for V tasks and E dependency edges."""
    tasks, successors, counts = _graph(tasks)
    ready = deque(name for name, count in counts.items() if count == 0)
    order = []
    while ready:
        name = ready.popleft()
        order.append(name)
        for successor in successors[name]:
            counts[successor] -= 1
            if counts[successor] == 0:
                ready.append(successor)
    if len(order) != len(tasks):
        waiting = [name for name, count in counts.items() if count]
        raise ConfigurationError("Cycle detected; no task may start. Waiting: " + ", ".join(waiting))
    return order


def _call(reference, *args):
    value = resolve(reference)(*args)
    if inspect.isawaitable(value):
        if inspect.iscoroutine(value):
            value.close()
        raise InvalidResult("An adapter must return completed data, not an awaitable")
    return value


def _worker(handler, validator, inputs, context, result_file, before):
    # Ctrl+C is handled by the parent, which terminates the worker and calls stop.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        if before:
            try:
                _call(before, inputs, context)
            except Exception as error:
                raise InvalidResult(str(error)) from error
        output = _call(handler, inputs, context)
        if not isinstance(output, dict):
            raise InvalidResult("A task must return a dictionary")
        try:
            json.dumps(output, allow_nan=False)
        except (TypeError, ValueError) as error:
            raise InvalidResult(f"Output must be finite JSON data: {error}") from error
        if validator:
            try:
                _call(validator, output, inputs, context)
            except Exception as error:
                raise InvalidResult(str(error)) from error
        outcome = {"status": "SUCCEEDED", "output": output}
    except ReplanRequested as error:
        outcome = {"status": "REPLAN_REQUIRED", "error": str(error), "event": error.event}
    except InvalidResult as error:
        outcome = {"status": "INVALID_RESULT", "error": str(error)}
    except BaseException as error:
        outcome = {"status": "FAILED", "error": f"{type(error).__name__}: {error}",
                   "traceback": traceback.format_exc()}
    outcome.update(request_id=context.request_id, task_id=context.task_id)
    atomic_json(result_file, outcome)


def _execute(handler, validator, inputs, context, timeout_s, cancellation=None, before=None):
    """Return only after the worker has exited or been terminated and joined."""
    start = time.monotonic()
    if cancellation is not None and cancellation.is_set():
        return {"status": "CANCELLED", "error": "Cancelled before worker start", "duration_s": 0.0}
    context = replace(context, deadline_monotonic=start + timeout_s)
    folder = Path(context.directory)
    folder.mkdir(parents=True, exist_ok=False)
    result_file = folder / "worker_result.json"
    process = mp.get_context("spawn").Process(
        target=_worker, args=(handler, validator, inputs, context, str(result_file), before),
        name=f"dag-{context.task_id}", daemon=True,
    )
    outcome = None
    try:
        process.start()
        while True:
            if cancellation is not None and cancellation.is_set():
                outcome = {"status": "CANCELLED", "error": "Cancellation requested"}
                break
            if time.monotonic() >= context.deadline_monotonic:
                outcome = {"status": "TIMED_OUT", "error": f"Exceeded {timeout_s:g} seconds"}
                break
            if not process.is_alive():
                process.join()
                if process.exitcode != 0 or not result_file.is_file():
                    outcome = {"status": "FAILED", "error": f"Worker exited without a usable result (code {process.exitcode})"}
                else:
                    try:
                        outcome = json.loads(result_file.read_text(encoding="utf-8"))
                        if outcome.get("request_id") != context.request_id or outcome.get("task_id") != context.task_id:
                            raise InvalidResult("Worker result belongs to another request")
                    except (ValueError, OSError, AttributeError) as error:
                        outcome = {"status": "INVALID_RESULT", "error": str(error)}
                break
            process.join(timeout=min(0.05, max(0.0, context.seconds_left())))
    except KeyboardInterrupt:
        if cancellation is not None:
            cancellation.set()
        outcome = {"status": "CANCELLED", "error": "Keyboard cancellation requested"}
    except Exception as error:
        outcome = {"status": "FAILED", "error": f"Cannot execute worker: {error}"}
    finally:
        if process.pid is not None:
            if process.is_alive():
                process.terminate()
            process.join(timeout=2)
            if process.is_alive():
                process.kill()
                process.join(timeout=2)
            if process.is_alive():
                raise RuntimeError("Worker termination could not be confirmed; stop further execution")
            process.close()
    outcome["duration_s"] = time.monotonic() - start
    return outcome


class Scheduler:
    def __init__(self, tasks):
        self.tasks = list(tasks)
        self.order = topological_order(self.tasks)

    def preflight(self):
        """Validate ALL bindings and policies before executing ANY task."""
        errors = []
        for task in self.tasks:
            if type(task.idempotent) is not bool or type(task.requires_stop) is not bool:
                errors.append(f"{task.name}: idempotent and requires_stop must be booleans")
            for label, number in (("timeout_s", task.timeout_s), ("stop_timeout_s", task.stop_timeout_s)):
                if type(number) not in (int, float) or not math.isfinite(number) or number <= 0:
                    errors.append(f"{task.name}: {label} must be positive and finite")
            if type(task.retry_delay_s) not in (int, float) or not math.isfinite(task.retry_delay_s) or task.retry_delay_s < 0:
                errors.append(f"{task.name}: retry delay must be finite and nonnegative")
            if type(task.max_attempts) is not int or task.max_attempts < 1:
                errors.append(f"{task.name}: max_attempts must be a positive integer")
            elif task.max_attempts > 1 and not task.idempotent:
                errors.append(f"{task.name}: retries require idempotent=True")
            if task.requires_stop and not task.on_stop:
                errors.append(f"{task.name}: a stop callback is required")
            for label, reference in (("handler", task.handler), ("validator", task.validator),
                                     ("before", task.before), ("on_stop", task.on_stop)):
                if reference is None and label != "handler":
                    continue
                if reference is None:
                    errors.append(f"{task.name}: no real module connected")
                    break
                try:
                    function = resolve(reference)
                    try:
                        signature = inspect.signature(function)
                    except (TypeError, ValueError):
                        signature = None  # Some extension functions do not expose signatures.
                    if signature is not None:
                        try:
                            signature.bind(*([None] * (3 if label == "validator" else 2)))
                        except TypeError as error:
                            raise ConfigurationError(f"Wrong arguments for {reference}: {error}") from error
                except ConfigurationError as error:
                    errors.append(f"{task.name}: {error}")
        if errors:
            raise ConfigurationError("\n".join(errors))

    def run(self, directory, *, mode="demo", mission_id=None, mission_attempt=1,
            parameters=None, cancellation=None, emit=print):
        self.preflight()
        if mode not in ("demo", "real"):
            raise ConfigurationError("Mode must be demo or real")
        parameters = json.loads(json.dumps(parameters or {}, allow_nan=False))
        cancellation = cancellation if cancellation is not None else threading.Event()
        directory = Path(directory).resolve()
        directory.mkdir(parents=True, exist_ok=False)
        run_id = uuid4().hex
        mission_id = mission_id or uuid4().hex
        start = time.monotonic()
        tasks, successors, counts = _graph(self.tasks)
        by_name = {task.name: task for task in tasks}
        task_index = {task.name: index for index, task in enumerate(tasks, 1)}
        records = {task.name: {"state": "PENDING", "attempts": [], "output": None} for task in tasks}
        outputs = {}
        report = {"format_version": 1, "mission_id": mission_id, "run_id": run_id,
                  "mission_attempt": mission_attempt, "mode": mode, "started_at_utc": utc_now(),
                  "status": "RUNNING", "intervention_required": False, "tasks": records}
        events = (directory / "events.jsonl").open("x", encoding="utf-8", buffering=1)
        active_task = active_context = active_inputs = None

        def event(name, state, **extra):
            records[name]["state"] = state
            item = {"time_utc": utc_now(), "elapsed_s": time.monotonic() - start,
                    "run_id": run_id, "task": name, "state": state, **extra}
            try:
                events.write(json.dumps(item, allow_nan=False) + "\n")
            except OSError as error:
                report["logging_error"] = str(error)
                cancellation.set()
            if emit:
                try:
                    emit(f"{state:<16} {name}" + (f" - {extra['error']}" if "error" in extra else ""))
                except Exception:
                    pass  # A display callback must not control execution.

        def block_remaining(failed):
            affected = set()
            pending = list(successors[failed]) if failed else []
            while pending:
                name = pending.pop()
                if name not in affected:
                    affected.add(name)
                    pending.extend(successors[name])
            for name, record in records.items():
                if record["state"] in ("PENDING", "READY", "RETRY_WAIT"):
                    event(name, "BLOCKED" if name in affected else "CANCELLED")

        ready = deque(name for name, count in counts.items() if count == 0)
        try:
            for name in ready:
                event(name, "READY")
            while ready:
                if cancellation.is_set():
                    report["status"] = "CANCELLED"
                    block_remaining(None)
                    break
                name = ready.popleft()
                task = by_name[name]
                inputs = {parent: outputs[parent] for parent in dict.fromkeys(task.needs)}
                outcome = None
                for attempt in range(1, task.max_attempts + 1):
                    if cancellation.is_set():
                        outcome = {"status": "CANCELLED", "error": "Cancelled before execution"}
                        break
                    request_id = uuid4().hex
                    context = TaskContext(mission_id, run_id, name, request_id,
                                          mission_attempt, attempt, mode, 0.0,
                                          str(directory / f"w{task_index[name]:03}_{attempt}"), parameters)
                    event(name, "RUNNING", attempt=attempt, request_id=request_id)
                    started_at = utc_now()
                    active_task, active_context, active_inputs = task, context, inputs
                    outcome = _execute(task.handler, task.validator, inputs, context, task.timeout_s, cancellation, task.before)
                    entry = {"number": attempt, "request_id": request_id,
                             "started_at_utc": started_at, "finished_at_utc": utc_now(),
                             **{k: v for k, v in outcome.items() if k != "output"}}
                    records[name]["attempts"].append(entry)
                    # A failed or interrupted operation may still be active remotely.
                    if outcome["status"] != "SUCCEEDED" and task.on_stop:
                        event(name, "STOPPING")
                        stop_context = replace(context, stop_reason=outcome.get("error", outcome["status"]),
                                               operation_directory=context.directory,
                                               directory=str(directory / f"s{task_index[name]:03}_{attempt}"))
                        stopped = _execute(task.on_stop, None, inputs, stop_context, task.stop_timeout_s)
                        acknowledged = (stopped["status"] == "SUCCEEDED"
                                        and stopped["output"].get("stopped") is True
                                        and stopped["output"].get("request_id") == request_id)
                        entry["stop"] = {**stopped, "confirmed": acknowledged}
                        if not acknowledged:
                            report["intervention_required"] = True
                    active_task = active_context = active_inputs = None
                    if outcome["status"] == "SUCCEEDED":
                        break
                    can_retry = (attempt < task.max_attempts and task.idempotent
                                 and outcome["status"] in ("FAILED", "TIMED_OUT")
                                 and not report["intervention_required"] and not cancellation.is_set())
                    if not can_retry:
                        break
                    event(name, "RETRY_WAIT", error=outcome.get("error", ""))
                    try:
                        cancellation.wait(task.retry_delay_s)
                    except KeyboardInterrupt:
                        cancellation.set()
                if outcome["status"] != "SUCCEEDED":
                    if outcome["status"] == "REPLAN_REQUIRED":
                        report["replan_event"] = outcome.get("event", {})
                    state = "FAILED" if outcome["status"] == "INVALID_RESULT" else outcome["status"]
                    event(name, state, error=outcome.get("error", ""))
                    report["status"] = state if state in ("CANCELLED", "REPLAN_REQUIRED") else "FAILED"
                    block_remaining(name)
                    break
                outputs[name] = outcome["output"]
                records[name]["output"] = outcome["output"]
                event(name, "SUCCEEDED")
                for successor in successors[name]:
                    counts[successor] -= 1
                    if counts[successor] == 0:
                        event(successor, "READY")
                        ready.append(successor)
            if all(record["state"] == "SUCCEEDED" for record in records.values()):
                report["status"] = "SUCCEEDED"
        except KeyboardInterrupt:
            cancellation.set()
            report["status"] = "CANCELLED"
            block_remaining(None)
        except Exception as error:
            report["status"] = "FAILED"
            report["scheduler_error"] = f"{type(error).__name__}: {error}"
            block_remaining(None)
        finally:
            # An interrupt/I/O failure between worker execution and normal cleanup
            # must not skip the remote stop request for an active operation.
            if active_task is not None:
                records[active_task.name]["state"] = "CANCELLED" if cancellation.is_set() else "FAILED"
                if active_task.on_stop:
                    try:
                        emergency_context = replace(active_context, stop_reason="Scheduler interrupted",
                            operation_directory=active_context.directory,
                            directory=str(directory / ("emergency_" + uuid4().hex[:8])))
                        stopped = _execute(active_task.on_stop, None, active_inputs, emergency_context, active_task.stop_timeout_s)
                        confirmed = (stopped["status"] == "SUCCEEDED" and stopped["output"].get("stopped") is True
                                     and stopped["output"].get("request_id") == active_context.request_id)
                        report["emergency_stop"] = {**stopped, "confirmed": confirmed}
                        report["intervention_required"] |= not confirmed
                    except Exception as error:
                        report["intervention_required"] = True
                        report["emergency_stop"] = {"confirmed": False, "error": str(error)}
            if report.get("logging_error"):
                report["status"] = "FAILED"
            report["finished_at_utc"] = utc_now()
            report["duration_s"] = time.monotonic() - start
            events.close()
            atomic_json(directory / "report.json", report)
        return report
