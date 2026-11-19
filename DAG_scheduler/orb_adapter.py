"""ORB-SLAM3 sequence input, archived-export import, and native Linux execution.

Saved real data is explicitly marked archived, never reported as a fresh SLAM
run. Poses/points remain in arbitrary monocular units until calibrated/aligned.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from adapters import result
from dag_scheduler.engine import ConfigurationError, InvalidResult, atomic_json
from orb_io.preflight import check_dataset, manifest_index, read_rgb_list, sha256_file
from orb_io.read_exports import read_exports

PROJECT = Path(__file__).resolve().parent
ARCHIVES = {"video1": "video1_3D_export", "video2": "video2_stability_test"}


def settings(context):
    configuration = context.parameters.get("orb", {})
    if not isinstance(configuration, dict):
        raise ConfigurationError("parameters.orb must be an object")
    root = Path(configuration.get("root", "../ORB_handoff/ORB_SLAM3")).expanduser()
    if not root.is_absolute():
        root = PROJECT / root
    root = root.resolve()
    sequence = configuration.get("sequence", "video1")
    source = configuration.get("source", "execute")
    if sequence not in ARCHIVES or source not in ("archive", "execute"):
        raise ConfigurationError("ORB sequence must be video1/video2; source must be archive/execute")
    return configuration, root, sequence, source


def checked_file(path, references, key):
    path = Path(path)
    record = references.get(key)
    if not path.is_file() or not record or path.stat().st_size != record["bytes"] or sha256_file(path) != record["sha256"]:
        raise InvalidResult(f"File is missing or differs from the supplied collection: {path}")
    return str(path.resolve())


def asset_paths(root, sequence):
    source = root / "source/ORB_SLAM3"
    return {"sequence": root / f"datasets/ENV1/{sequence}_first20s",
            "manifest": root / "evidence/COLLECTION_MANIFEST.json",
            "settings": source / "Examples/Monocular/Tello_ENV1_provisional.yaml",
            "vocabulary": source / "Vocabulary/ORBvoc.txt",
            "binary": source / "Examples/Monocular/mono_tum"}


def verify_inputs(root, sequence):
    paths = asset_paths(root, sequence)
    references = manifest_index(paths["manifest"])
    report = check_dataset(paths["sequence"], references)
    if report["status"] != "inputs_valid":
        raise InvalidResult("ORB input check failed: " + "; ".join(report["errors"][:5]))
    checked_file(paths["settings"], references, "ORB_SLAM3/Examples/Monocular/Tello_ENV1_provisional.yaml")
    checked_file(paths["vocabulary"], references, "ORB_SLAM3/Vocabulary/ORBvoc.txt")
    return paths, references, report


def prepare_recording(inputs, context):
    if context.mode != "real":
        raise ConfigurationError("ORB adapters handle recorded real data; use mode=real")
    configuration, root, sequence, source = settings(context)
    paths, references, report = verify_inputs(root, sequence)
    recording_id = f"{sequence}_{sha256_file(paths['sequence'] / 'rgb.txt')[:12]}"
    frames = [{"frame_id": f"{recording_id}:{row['input_frame_index']:06}",
               "timestamp_s": row["timestamp_s"],
               "image_ref": str((paths["sequence"] / row["relative_path"]).resolve()),
               "sha256": row["sha256"], "image_relative_path": row["relative_path"]}
              for row in report["available_frame_details"]]
    atomic_json(Path(context.directory) / "input_check.json", report)
    return result(context, recording_id=recording_id, calibration_ref=str(paths["settings"]),
                  calibration_verified=False, frames=frames, sequence_id=sequence,
                  sequence_ref=str(paths["sequence"]), rgb_list_ref=str(paths["sequence"] / "rgb.txt"),
                  collection_manifest_ref=str(paths["manifest"]), input_frame_count=len(frames),
                  data_origin="recorded_Tello_video", requested_source=source)


def check_runtime(binary):
    if not sys.platform.startswith("linux"):
        raise ConfigurationError("Fresh ORB execution must run inside Linux/WSL using Linux Python; archived import works on Windows")
    binary = Path(binary).resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ConfigurationError(f"Build or configure the Linux mono_tum executable first: {binary}")
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise ConfigurationError("The supplied ORB runner enables its viewer; a working OpenGL display/WSLg session is required")
    return binary


def execute_command(command, folder, context):
    """Use a supervisor that outlives a killed worker and owns the native child."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    request_path = folder / "launch_intent.json"
    request = {"request_id": context.request_id, "directory": str(folder), "command": command,
               "deadline_monotonic": context.deadline_monotonic - 4.0}
    if request["deadline_monotonic"] <= time.monotonic():
        raise TimeoutError("Too little time remains to launch and clean up ORB")
    atomic_json(request_path, request)
    supervisor = None
    try:
        with (folder / "supervisor.log").open("wb") as supervisor_log:
            supervisor = subprocess.Popen([sys.executable, "-B", str(PROJECT / "orb_supervisor.py"), str(request_path)],
                                          stdin=subprocess.PIPE, stdout=supervisor_log, stderr=subprocess.STDOUT)
        # stdin stays OPEN until completion. A worker crash closes it in the OS,
        # triggering the independent supervisor's cancellation/cleanup path.
        while supervisor.poll() is None:
            if context.seconds_left() < 3.0:
                supervisor.stdin.close()
                break
            time.sleep(0.04)
        supervisor.wait(timeout=max(0.1, context.seconds_left()))
        report = json.loads((folder / "process_status.json").read_text(encoding="utf-8"))
        if report.get("request_id") != context.request_id or not report.get("termination_confirmed"):
            raise InvalidResult(f"Native process termination was not confirmed for this request; see {folder / 'supervisor.log'}")
        if report["state"] != "COMPLETED" or report["return_code"] != 0 or supervisor.returncode != 0:
            raise RuntimeError(f"ORB did not complete: {report['state']}; exit={report['return_code']}; see {folder / 'run.log'}")
        return report
    finally:
        if supervisor is not None:
            if supervisor.stdin and not supervisor.stdin.closed:
                supervisor.stdin.close()
            # Do not kill the supervisor: it must finish cleaning its child.
            if supervisor.poll() is None:
                try:
                    supervisor.wait(timeout=max(0.1, context.seconds_left()))
                except subprocess.TimeoutExpired:
                    pass  # stop_slam will require its persisted acknowledgement.


def _parse_and_publish(poses, points, log, recording, context, source, process=None):
    parsed = read_exports(poses, points, recording["rgb_list_ref"], recording["collection_manifest_ref"])
    by_time = {frame["timestamp_s"]: frame for frame in recording["frames"]}
    selected = []
    for item in parsed["keyframes"]:
        matches = [frame for stamp, frame in by_time.items() if abs(stamp - item["timestamp_s"]) <= 0.0000005]
        if len(matches) != 1 or not item.get("image_available") or not item.get("image_hash_matches_collection_manifest"):
            raise InvalidResult("An exported keyframe is not linked to one verified input image")
        selected.append({**matches[0], "export_row_index": item["export_row_index"],
                         "internal_orb_keyframe_id": None})
    folder = Path(context.directory)
    poses_ref, geometry_ref = folder / "poses.json", folder / "geometry.json"
    atomic_json(poses_ref, {"pose_convention": "T_wc camera-to-world", "coordinate_units": parsed["coordinate_units"],
                            "keyframes": parsed["keyframes"], "metric_calibrated": False})
    atomic_json(geometry_ref, {"coordinate_units": parsed["coordinate_units"], "metric_calibrated": False,
                              "map_points_xyz": parsed["map_points_xyz"], "map_scope": "current_Atlas_map_only"})
    output = result(context, recording_id=recording["recording_id"], producer="ORB_SLAM3",
                    keyframe_ids=[row["frame_id"] for row in selected], selected_frames=selected,
                    keyframe_identity_kind="input_frame_ids_not_internal_ORB_ids",
                    keyframe_count=parsed["keyframe_count"], map_point_count=parsed["map_point_count"],
                    poses_ref=str(poses_ref), geometry_ref=str(geometry_ref),
                    coordinate_units=parsed["coordinate_units"], pose_convention="T_wc camera-to-world",
                    metric_calibrated=False, arena_alignment_verified=False, calibration_verified=False,
                    navigation_ready=False, tracking_states_available=False,
                    execution_kind="archived_export_import" if source == "archive" else "fresh_native_execution",
                    fresh_slam_run=(source == "execute"), source_poses_sha256=parsed["poses_sha256"],
                    source_points_sha256=parsed["points_sha256"], source_poses_ref=str(poses),
                    source_points_ref=str(points), log_ref=str(log), process=process,
                    archive_exit_code_recorded=False if source == "archive" else None)
    atomic_json(folder / "slam_handoff.json", output)
    return output


def run_slam(inputs, context):
    configuration, root, sequence, source = settings(context)
    recording = inputs["prepare_recording"]
    if recording["sequence_id"] != sequence or recording["requested_source"] != source:
        raise InvalidResult("ORB configuration changed after preparing the recording")
    paths, references, report = verify_inputs(root, sequence)
    if str(paths["sequence"].resolve()) != recording["sequence_ref"]:
        raise InvalidResult("ORB recording path does not match the prepared input")
    process = None
    if source == "archive":
        run_name = ARCHIVES[sequence]
        archive = root / "historical_results" / run_name
        for name in ("KeyFrameTrajectory.txt", "MapPoints.xyz", "run.log"):
            checked_file(archive / name, references, f"results/ENV1/{run_name}/{name}")
        log_text = (archive / "run.log").read_text(errors="replace").lower()
        if any(marker in log_text for marker in ("segmentation fault", "core dumped", "terminate called")):
            raise InvalidResult("The archived run log indicates a crash")
        raw = Path(context.directory) / "archived"
        raw.mkdir(exist_ok=False)
        for name in ("KeyFrameTrajectory.txt", "MapPoints.xyz", "run.log"):
            shutil.copy2(archive / name, raw / name)
            checked_file(raw / name, references, f"results/ENV1/{run_name}/{name}")
    else:
        binary = check_runtime(configuration.get("binary") or paths["binary"])
        raw = Path(context.directory) / "native"
        process = execute_command([str(binary), str(paths["vocabulary"]), str(paths["settings"]), str(paths["sequence"])], raw, context)
    return _parse_and_publish(raw / "KeyFrameTrajectory.txt", raw / "MapPoints.xyz", raw / "run.log",
                              recording, context, source, process)


def stop_slam(inputs, context):
    operation = context.operation_directory
    if not operation:
        return {"stopped": False, "request_id": context.request_id, "reason": "Original operation directory unavailable"}
    native = Path(operation) / "native"
    if not (native / "launch_intent.json").exists():
        return {"stopped": True, "request_id": context.request_id, "reason": "No native launch was initiated"}
    while context.seconds_left() > 0.1:
        try:
            report = json.loads((native / "process_status.json").read_text(encoding="utf-8"))
            if report.get("request_id") == context.request_id and report.get("termination_confirmed") is True:
                return {"stopped": True, "request_id": context.request_id, "native_state": report["state"]}
        except (OSError, ValueError):
            pass
        time.sleep(0.05)
    return {"stopped": False, "request_id": context.request_id, "reason": "Native-process exit could not be confirmed"}


def publish_slam(inputs, context):
    slam = inputs["run_slam"]
    summary = {"stage": "ORB_SLAM3_ONLY", "execution_kind": slam["execution_kind"],
               "fresh_slam_run": slam["fresh_slam_run"], "keyframe_count": slam["keyframe_count"],
               "map_point_count": slam["map_point_count"], "coordinate_units": slam["coordinate_units"],
               "navigation_ready": False, "poses_ref": slam["poses_ref"], "geometry_ref": slam["geometry_ref"],
               "selected_frames": slam["selected_frames"]}
    atomic_json(Path(context.directory) / "orb_stage_summary.json", summary)
    return result(context, **summary)


def validate_output(output, inputs, context):
    from contracts import validate
    if context.task_id == "publish_slam":
        expected = inputs["run_slam"]
        if (output.get("attempt_id") != context.run_id or output.get("simulated") is not False
                or output.get("stage") != "ORB_SLAM3_ONLY" or output.get("navigation_ready") is not False):
            raise InvalidResult("Invalid ORB-stage summary")
        for key in ("keyframe_count", "map_point_count", "execution_kind", "fresh_slam_run", "poses_ref", "geometry_ref"):
            if output.get(key) != expected[key]:
                raise InvalidResult(f"ORB summary does not match accepted result: {key}")
        return
    validate(output, inputs, context)
    if context.task_id != "run_slam":
        return
    if output.get("producer") != "ORB_SLAM3" or output.get("navigation_ready") is not False:
        raise InvalidResult("ORB raw geometry must not be labelled navigation-ready")
    if output.get("coordinate_units") != "arbitrary_monocular_slam_units":
        raise InvalidResult("The supplied monocular exports are not measured metre coordinates")
    for flag in ("metric_calibrated", "arena_alignment_verified", "calibration_verified", "tracking_states_available"):
        if output.get(flag) is not False:
            raise InvalidResult(f"The supplied ORB handoff does not establish {flag}")
    expected_frames = set(output["keyframe_ids"])
    if len(output.get("selected_frames", [])) != len(expected_frames) or output["keyframe_count"] != len(expected_frames):
        raise InvalidResult("Keyframe counts and selected images disagree")
    if {frame["frame_id"] for frame in output["selected_frames"]} != expected_frames:
        raise InvalidResult("Selected images differ from the keyframe list")
    for key in ("poses_ref", "geometry_ref"):
        path = Path(output[key]).resolve()
        if not path.is_relative_to(Path(context.directory).resolve()) or not path.is_file():
            raise InvalidResult("ORB output must exist in this task's own directory")
    geometry = json.loads(Path(output["geometry_ref"]).read_text(encoding="utf-8"))
    if len(geometry["map_points_xyz"]) != output["map_point_count"] or output["map_point_count"] < 1:
        raise InvalidResult("Point count does not match the saved geometry")
    if output["fresh_slam_run"]:
        process = output.get("process") or {}
        if (output["execution_kind"] != "fresh_native_execution" or process.get("return_code") != 0
                or process.get("termination_confirmed") is not True or process.get("request_id") != context.request_id):
            raise InvalidResult("A fresh run requires this request's successful process exit")
    elif output["execution_kind"] != "archived_export_import" or output.get("process") is not None:
        raise InvalidResult("Saved exports must be identified as archived data")
