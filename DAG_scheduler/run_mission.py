"""Run the DAG scheduler. Default: simulated modules, no hardware commands."""
import argparse
import json
import multiprocessing
from pathlib import Path
import sys

from dag_scheduler import Scheduler
from dag_scheduler.engine import ConfigurationError
from mission import build_tasks, run_mission


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("demo", "real"), default="demo")
    parser.add_argument("--bindings", type=Path, default=Path(__file__).with_name("real_bindings.json"))
    parser.add_argument("--scenario", choices=("success", "no_path", "fail_slam", "bad_data", "timeout", "replan"), default="success")
    parser.add_argument("--max-replans", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("runs"))
    parser.add_argument("--check", action="store_true", help="Check all module connections without running tasks")
    parser.add_argument("--show-order", action="store_true", help="Show a valid Kahn order without running tasks")
    args = parser.parse_args(argv)
    try:
        if args.mode == "real" and args.scenario != "success":
            raise ConfigurationError("Demonstration scenarios are available only in demo mode")
        if args.max_replans < 0:
            raise ConfigurationError("max-replans cannot be negative")
        bindings = json.loads(args.bindings.read_text(encoding="utf-8")) if args.mode == "real" else {}
        if not isinstance(bindings, dict):
            raise ConfigurationError("Bindings file must contain a JSON object")
        tasks = build_tasks(args.mode, bindings, args.scenario)
        scheduler = Scheduler(tasks)
        if args.show_order:
            print("\n".join(f"{i}. {name}" for i, name in enumerate(scheduler.order, 1)))
            return 0
        scheduler.preflight()
        if args.check:
            print("All task bindings, dependencies, and execution policies are valid. No tasks ran.")
            return 0
        parameters = bindings.get("parameters", {})
        if not isinstance(parameters, dict):
            raise ConfigurationError("parameters must be a JSON object")
        if args.mode == "demo":
            parameters = {"scenario": args.scenario}
        print(f"DAG SCHEDULER | mode={args.mode}")
        summary, folder = run_mission(tasks, args.output, mode=args.mode, parameters=parameters,
                                       max_replans=args.max_replans)
        print(f"\nMission status: {summary['status']}")
        print(f"Results: {folder}")
        if args.mode == "demo":
            print("All algorithm and movement results were simulated.")
        if any(a["intervention_required"] for a in summary["attempts"]):
            print("Stop was NOT confirmed. Further automatic attempts are blocked.")
        return 0 if summary["status"] == "SUCCEEDED" else 130 if summary["status"] == "CANCELLED" else 1
    except (ConfigurationError, ValueError, OSError) as error:
        print(f"Cannot run: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
