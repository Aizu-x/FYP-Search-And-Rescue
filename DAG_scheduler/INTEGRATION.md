# Connecting your colleagues' algorithms

Update, 17 November 2026: the ORB recording and SLAM adapters are implemented. See ORB_INTEGRATION.md for the connected stage and verified archived-data results. The instructions below continue to apply to the remaining modules.

The scheduler core is implemented. Integration means writing a small wrapper around each colleague's function and testing the data passed between them. We will do this when you provide their code. Arbitrary code cannot be made compatible solely by putting its file in the same folder.

## What to provide

For each module, provide its Python files or repository, dependencies/model files, how to run it, one sample input, one successful output, and a failure example. For movement, also provide position feedback, arrival/failure feedback, and the stop/cancel interface. If a module runs on the MentorPi instead of the laptop, provide its request/response or ROS interface.

## The common interface

Every adapter is a normal, importable Python function:

```python
def module_adapter(inputs, context):
    # Read this task's completed prerequisites from inputs.
    # Call the real module and wait for a defined result.
    # Translate that result into the documented output format.
    return output_dictionary
```

`inputs` contains only the direct prerequisites listed in `mission.py`. These are copies; changing them does not change another task's stored result. `context` supplies mission/run/request IDs, task name, attempt numbers, demo/real mode, a task output directory, parameters, and `seconds_left()` for bounded communication timeouts.

Normal success means returning the required dictionary. Raise an exception for failure. A missing route returns `status="NO_PATH"` with no waypoints; the mission validator rejects it as a planning failure and blocks navigation. Never return a made-up route to force the scheduler to continue.

Do not return camera handles, tensors, model objects, numpy arrays, open connections, or an endless generator. Convert small data to ordinary dictionaries/lists/numbers, and save large data to files with absolute path references. Functions must finish; coroutine functions require a synchronous wrapper that waits for their completed result.

## Connecting A* as an example

The A* adapter receives:

```python
planning_map = inputs["prepare_planning_map"]
start = inputs["get_ugv_pose"]["position_m"]
goal = inputs["select_target"]["position_m"]
```

We call the colleague's actual planner using its required arguments, converting metre coordinates to grid cells if necessary. After converting its returned route back to metres, use:

```python
from adapters import planning_output

# route_in_metres is the actual result from the colleague's planner.
return planning_output(inputs, context, route_in_metres)
```

If it reports no route:

```python
return planning_output(inputs, context, [], status="NO_PATH")
```

This helper supplies the map/target identifiers, a new path ID, and the start-pose timestamp. It does not calculate or collision-check the route.

When `team_adapters.plan_path` has been implemented and tested, remove its `@not_connected` marker and set this entry inside `real_bindings.json`:

```json
"plan_path": "team_adapters:plan_path"
```

Repeat for the other tasks. The dictionary values are `module_name:function_name`; imports must work from this project folder. Modules must not start hardware operations merely when imported, since preflight imports them to check their function definitions.

## Register and test before the complete mission

1. Test each colleague's algorithm independently.
2. Write its adapter using the fields in DATA_FORMATS.md.
3. Test the adapter with known inputs, including missing data and failure cases.
4. Register the adapter in real_bindings.json.
5. Use `python run_mission.py --mode real --check` to verify all required bindings and policies.
6. Once all modules and stop feedback are ready, run `python run_mission.py --mode real`.

Real mode has no fallback to sample modules. Incomplete real integration can be tested as a smaller DAG with only its required fixtures and honest demo labels; a complete real mission needs every required module connected. The final summary task is already supplied by the scheduler.

## Navigation and stopping

The navigation adapter must wait for actual arrival or failure. Merely accepting or sending a route is not success. Send `context.request_id` with the command and check that responses belong to that request using `adapters.check_reply`. Do not change an old response's ID to make it appear current.

Set `stop_navigation` in real_bindings.json to the independent stop adapter. It runs if navigation fails, times out, is cancelled, or requests replanning. It receives the SAME request ID and must return:

```python
{"stopped": True, "request_id": context.request_id}
```

Return this only after the controller confirms stopping. The stop must be idempotent: it must also handle a request that never started or already finished. The controller must remember cancelled request IDs so a late movement command cannot restart them. On communication loss, do not claim stopping without confirmation. A failed/incorrect/timed-out stop acknowledgement blocks automatic replanning and marks intervention as required.

Terminating a local Python worker does not stop a remote robot or detached process. The real controller also needs its own loss-of-contact behaviour. Stop callbacks run in a fresh process and must reconnect independently; they cannot rely on a connection object from the navigation worker.

## Replanning after a new obstacle

The navigation adapter or its monitoring service can request a new attempt:

```python
from dag_scheduler import ReplanRequested

raise ReplanRequested("New obstacle", {
    "frame": "arena",
    "blocked_cells": [[30, 30]],
    "observed_at_unix_s": observation_time,
})
```

With `--max-replans 1`, the controller first requires confirmed stopping, then starts a fresh acyclic graph. The event is available to all new adapters in `context.parameters["replan_event"]`. Mapping must actually incorporate that observation, and localization must supply a fresh pose. The scheduler forwards the event but does not interpret real obstacle geometry itself.

All tasks rerun in this first single-target controller; no old outputs are silently reused. Ordinary failures and NO_PATH do not automatically cause replanning. The limit counts additional mission attempts, separately from a task's optional retries.

## Execution details to account for

- Each task uses a new spawned process. Functions must be importable from files; avoid lambdas and notebook-only definitions. Programs using the Python API must call it inside `if __name__ == "__main__":` on Windows.
- Worker startup and model loading count toward the timeout. Defaults are 60 seconds per task and 120 for navigation. Override task names in the `timeouts_s` dictionary after measuring real performance.
- These workers do not support spawning additional multiprocessing children. For models using multiprocessing data loaders, use zero loader workers or host the model as a persistent service and call it through an adapter.
- Long-lived video capture, ROS nodes, model services, and continuous YOLO monitoring belong outside a finite DAG task. Give their adapter a finite request/result and a bounded timeout. The project demo inspects after arrival, but that does not prevent a separately managed detector from monitoring during motion.
- Current pose-age and arrival-tolerance defaults are 5 seconds and 0.05 metres. Review them with the team; `parameters.max_pose_age_s` and `parameters.arrival_tolerance_m` configure them. Pose timestamps must use the edge device's clock or be translated into it.
- The current data contract fixes the proposed 3 m arena as a 60x60 grid with 0.05 m cells. Change the contract and adapters together if the final map changes. Metric scale, alignment, clearance, collisions, and detector accuracy remain responsibilities of the real modules and integration tests.

When your colleagues' files arrive, the work should be concentrated in the adapters, dependencies, and these agreed data formats. Kahn's scheduling logic can remain unchanged.
