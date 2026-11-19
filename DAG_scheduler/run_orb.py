"""Run only the connected ORB stage through the existing Kahn scheduler.

Default: import both supplied archived results. Use --source execute inside
Linux/WSL for a fresh C++ run. The full navigation mission is not run here.
"""
import argparse
import json
import math
import multiprocessing
from pathlib import Path
from uuid import uuid4

from dag_scheduler import Scheduler, Task
from dag_scheduler.engine import ConfigurationError, atomic_json
from orb_adapter import PROJECT, asset_paths, check_runtime, verify_inputs


def orb_tasks(timeout_s=300.0):
    return [
        Task("prepare_recording", handler="team_adapters:prepare_recording",
             validator="orb_adapter:validate_output", timeout_s=120.0),
        Task("run_slam", ("prepare_recording",), "team_adapters:run_slam",
             validator="orb_adapter:validate_output", timeout_s=timeout_s,
             on_stop="orb_adapter:stop_slam", requires_stop=True, stop_timeout_s=15),
        Task("publish_slam", ("run_slam",), "orb_adapter:publish_slam",
             validator="orb_adapter:validate_output", timeout_s=30),
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("archive", "execute"), default="archive")
    parser.add_argument("--sequence", choices=("video1", "video2", "all"), default="all")
    parser.add_argument("--orb-root", type=Path, default=PROJECT.parent / "ORB_handoff/ORB_SLAM3")
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--output", type=Path, default=PROJECT / "runs")
    parser.add_argument("--check", action="store_true", help="Validate input assets and runtime availability without running tasks")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.timeout) or args.timeout < 10:
            raise ConfigurationError("ORB timeout must be at least 10 finite seconds, including cleanup reserve")
        root = args.orb_root.resolve()
        sequences = ("video1", "video2") if args.sequence == "all" else (args.sequence,)
        if args.source == "execute":
            check_runtime(args.binary or asset_paths(root, sequences[0])["binary"])
        if args.check:
            for sequence in sequences:
                _, _, report = verify_inputs(root, sequence)
                print(f"{sequence}: {report['available_frames']} frames verified; inputs ready; no SLAM run performed")
            return 0
        scheduler = Scheduler(orb_tasks(args.timeout))
        scheduler.preflight()
        for sequence in sequences:
            folder = args.output.resolve() / f"orb_{sequence}_{uuid4().hex[:12]}"
            parameters = {"orb": {"root": str(root), "sequence": sequence, "source": args.source,
                                  "binary": str(args.binary.resolve()) if args.binary else None}}
            print(f"ORB STAGE: {sequence}; {'IMPORT SAVED EXPORTS - no fresh SLAM run' if args.source == 'archive' else 'FRESH NATIVE SLAM EXECUTION'}")
            report = scheduler.run(folder, mode="real", parameters=parameters)
            summary = {"status": report["status"], "scope": "ORB stage only", "sequence": sequence,
                       "source": args.source, "fresh_slam_run": False,
                       "intervention_required": report["intervention_required"], "report_ref": str(folder / "report.json")}
            if report["status"] == "SUCCEEDED":
                output = report["tasks"]["publish_slam"]["output"]
                summary.update({key: output[key] for key in ("execution_kind", "fresh_slam_run", "keyframe_count", "map_point_count", "coordinate_units", "navigation_ready", "poses_ref", "geometry_ref")})
            atomic_json(folder / "orb_stage.json", summary)
            print(json.dumps(summary, indent=2))
            if report["status"] != "SUCCEEDED":
                return 130 if report["status"] == "CANCELLED" else 1
        return 0
    except (OSError, ValueError, KeyError, ConfigurationError) as error:
        print(f"ORB stage cannot run: {error}")
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
