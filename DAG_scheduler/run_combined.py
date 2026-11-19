"""Run ORB, the supplied A*, and YOLOv5n under one Kahn scheduler. No motors."""
import argparse
import json
import multiprocessing
from pathlib import Path
from uuid import uuid4

from dag_scheduler import Scheduler, Task
from dag_scheduler.engine import atomic_json
from orb_adapter import PROJECT, asset_paths, check_runtime, verify_inputs
from ugv_adapter import ROOT, planner_settings


def combined_tasks():
    return [
        Task("prepare_recording", handler="team_adapters:prepare_recording", validator="orb_adapter:validate_output", timeout_s=120),
        Task("run_slam", ("prepare_recording",), "team_adapters:run_slam", validator="orb_adapter:validate_output",
             timeout_s=300, on_stop="orb_adapter:stop_slam", requires_stop=True, stop_timeout_s=15),
        Task("load_navigation_inputs", handler="combined_adapter:load_navigation_inputs", validator="combined_adapter:validate_output"),
        Task("plan_reference_route", ("load_navigation_inputs",), "combined_adapter:plan_reference_route",
             validator="combined_adapter:validate_output", timeout_s=120),
        Task("inspect_images", handler="combined_adapter:inspect_images", validator="combined_adapter:validate_output", timeout_s=180),
        Task("write_integration_report", ("run_slam", "load_navigation_inputs", "plan_reference_route", "inspect_images"),
             "combined_adapter:write_integration_report", validator="combined_adapter:validate_output"),
    ]


def default_orb_root():
    candidates = [ROOT / "ORB_handoff/ORB_SLAM3", ROOT.parent / "ORB_handoff/ORB_SLAM3"]
    return next((p for p in candidates if p.is_dir()), candidates[0])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orb-root", type=Path, default=default_orb_root())
    parser.add_argument("--orb-source", choices=("archive", "execute"), default="archive")
    parser.add_argument("--sequence", choices=("video1", "video2"), default="video1")
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--environment", choices=("ENV1", "ENV2", "ENV3"), default="ENV1")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--images", type=Path, nargs="+")
    parser.add_argument("--radius", type=float, default=0.143)
    parser.add_argument("--margin", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=ROOT / "combined_runs")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--show-order", action="store_true")
    args = parser.parse_args(argv)
    try:
        scheduler = Scheduler(combined_tasks())
        if args.show_order:
            print("\n".join(scheduler.order))
            return 0
        scheduler.preflight()
        planner = {"radius_m": args.radius, "margin_m": args.margin}
        planner_settings(planner)
        if args.orb_source == "execute":
            check_runtime(args.binary or asset_paths(args.orb_root, args.sequence)["binary"])
        if args.check:
            from ugv_nav.vision import load_detector
            verify_inputs(args.orb_root.resolve(), args.sequence)
            load_detector()
            print("Inputs and detector load checked. Fresh SLAM, path planning, and robot movement were not run.")
            return 0
        parameters = {"orb": {"root": str(args.orb_root.resolve()), "sequence": args.sequence, "source": args.orb_source,
                              "binary": str(args.binary.resolve()) if args.binary else None},
                      "planner": planner, "yolo": {"confidence": 0.25},
                      "reference": {"environment": args.environment, "start_m": args.start, "goal_m": args.goal,
                                    "images": [str(p.resolve()) for p in args.images] if args.images else None}}
        folder = args.output.resolve() / f"combined_{uuid4().hex[:12]}"
        print("COMPONENT CHECK: ORB + supplied A* + YOLOv5n. Saved map; no robot movement; no ORB-to-grid fusion.")
        report = scheduler.run(folder, mode="real", parameters=parameters)
        summary = {"status": report["status"], "scope": "reference_components_only",
                   "navigation_ready": False, "report_ref": str(folder / "report.json")}
        if report["status"] == "SUCCEEDED":
            summary.update(report["tasks"]["write_integration_report"]["output"])
        atomic_json(folder / "combined_summary.json", summary)
        print(f"Status: {summary['status']}\nResults: {folder}")
        return 0 if report["status"] == "SUCCEEDED" else 130 if report["status"] == "CANCELLED" else 1
    except (OSError, ValueError, ImportError) as error:
        print(f"Cannot run combined stage: {error}")
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
