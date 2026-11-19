# DAG scheduler - ready to run

Combined-project update: ORB, the supplied A*, and YOLOv5n now have adapters. Start with [../EDGE_SETUP.md](../EDGE_SETUP.md) and [../COMBINED_INTEGRATION.md](../COMBINED_INTEGRATION.md). The supplied detector is v5n, not the earlier planned v8. The instructions below retain the standalone demonstration.

Updated 17 November 2026: ORB-SLAM3 input/export adapters are now connected. Read [ORB_INTEGRATION.md](ORB_INTEGRATION.md) and run RUN_ORB_ARCHIVES.cmd to process both supplied saved-result sets through the DAG. A fresh native Linux execution path is implemented but has not been verified with the compiled ORB binary here.

The original full simulated mission remains available below. Integration of the remaining modules continues as their interfaces are supplied.

## Run now

On this Windows laptop, double-click **RUN_DEMO.cmd**. It looks for the available bundled Python runtime, then a normal Python installation.

Or open this folder in your editor and run:

```text
python run_mission.py
```

Python 3.10 or newer is required. The scheduler itself needs no extra packages. A successful demonstration completes eleven tasks and saves results under `runs/mission_<id>/`. All algorithm and robot responses in this mode are sample data.

## What is implemented

- Kahn's algorithm checks for circular dependencies and finds which tasks may run.
- Only successfully validated results release dependent tasks.
- Task states, execution durations, per-attempt IDs, event logs, and JSON reports.
- Time limits that include worker startup, task execution, result validation, and result saving.
- Cancellation using Ctrl+C, with an independent stop callback for navigation.
- Optional retries only for tasks explicitly declared safe to repeat. The FYP graph defaults to one attempt per task, so movement is never automatically retried.
- A separate mission controller that can start a fresh DAG after an explicit replanning event, within a chosen limit. It forwards the event to the new attempt and refuses to replan if stopping was not confirmed.
- A real-module registry and preflight checks that reject missing or unimplemented adapters.

The engine runs one ready task at a time. It does not optimize CPU/GPU allocation or run independent branches concurrently. Each task runs in a separate worker process; its returned result must be JSON data or references to files.

## Try the other examples

```text
python run_mission.py --scenario no_path
python run_mission.py --scenario fail_slam
python run_mission.py --scenario bad_data
python run_mission.py --scenario timeout
python run_mission.py --scenario replan --max-replans 1
python run_mission.py --show-order
```

Failures intentionally return exit code 1. A cancelled mission returns 130. A configuration error returns 2. The replan example simulates an obstacle, confirms a simulated stop, then uses a new map revision and robot position for a second attempt. Automatic replanning is disabled unless `--max-replans` is set.

## Files to understand first

| File | What it does |
|---|---|
| `run_mission.py` | Starts the program |
| `mission.py` | Defines the task graph and controls fresh planning attempts |
| `dag_scheduler/engine.py` | Implements Kahn scheduling and task execution |
| `demo_modules.py` | Supplies sample data for demonstrations |
| `team_adapters.py` | Where we will connect colleagues' code |
| `real_bindings.json` | Names the real adapter for each task |
| `contracts.py` | Checks that results contain consistent information |
| `adapters.py` | Helpers for formatting results and rejecting old replies |
| `INTEGRATION.md` | Explains the connection process |
| `TEAM_HANDOFF.md` | Detailed checklist of what each colleague should send |
| `run_orb.py` | Runs the connected ORB-only graph |
| `orb_adapter.py` | Validates recordings, imports saved exports, or starts a native run |
| `orb_supervisor.py` | Owns the native process and confirms stopping |
| `ORB_INTEGRATION.md` | ORB usage, verification, data format, and remaining requirements |
| `ORB_TEST_RESULTS.md` | Current integration checks and their limits |
| `DATA_FORMATS.md` | Lists what each module must return |
| `tests/` | Automated verification using test fixtures |

## Read the results

`mission.json` shows the overall status and every mission attempt. Each `attempt_N/report.json` contains task states, durations, outputs, and errors. `events.jsonl` records state changes in order. The short `w...` and `s...` folders hold worker and stop-callback results; request IDs are stored inside the files.

`SUCCEEDED` in demo mode means the simulated workflow finished. In real mode, it means all configured module checks passed. It does not independently establish perception accuracy or physical path clearance.

## What still needs the team's code

The ORB sequence adapters and the supplied source/assets are connected as described in ORB_INTEGRATION.md. A live Wi-Fi receiver, ERFNet inference, map construction, localization, A*, movement controller, and YOLOv8 remain separate connections. `python run_mission.py --mode real --check` lists the remaining missing modules; use run_orb.py to run the ready ORB stage independently.

The project assumptions remain: Tello connects directly to the edge/laptop over Wi-Fi through Python; ORB-SLAM3 and ERFNet run on the edge; MentorPi M1 uses the team's custom A* and YOLOv8; labels are free space, obstacle, and blocked.

The current mission inspects one target. Continuous capture/detection should run as separate services, with adapters returning a defined result. Multiple-target scheduling and hardware-specific communications will be added when their interfaces are known. The supplied path is a two-point demonstration, and the supplied map has no robot-footprint inflation.

## Verify the code

```text
python -m unittest discover -s tests -v
```

Automated tests use controlled fixtures, not a physical robot. See ORB_TEST_RESULTS.md for the current checks and the separate integration runs with the supplied ORB data. TEST_RESULTS.md preserves the October baseline.
