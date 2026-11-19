from dataclasses import replace
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from dag_scheduler import Scheduler, Task, TaskContext
from dag_scheduler.engine import InvalidResult
from orb_adapter import checked_file, execute_command, stop_slam, validate_output, verify_inputs
from orb_io.preflight import read_rgb_list
from orb_io.read_exports import read_exports
from install_orb_handoff import destination
from mission import build_tasks
from run_orb import orb_tasks
from orb_supervisor import write_status


class OrbTests(unittest.TestCase):
    def setUp(self):
        self.scratch = Path(__file__).resolve().parent / "_scratch"
        self.scratch.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=self.scratch)
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.assertTrue(self.root.resolve().is_relative_to(self.scratch.resolve()))
        self.temporary.cleanup()

    def context(self, folder=None, seconds=15):
        return TaskContext("mission", "run", "run_slam", "request", 1, 1, "real",
                           time.monotonic() + seconds, str(folder or self.root), {})

    def exports(self, poses="0.000000 0 0 0 0 0 0 1\n", points="1 2 3\n"):
        p, q = self.root / "poses.txt", self.root / "points.xyz"
        p.write_text(poses, encoding="utf-8")
        q.write_text(points, encoding="utf-8")
        return p, q

    def test_valid_exports_keep_arbitrary_units(self):
        p, q = self.exports()
        output = read_exports(p, q)
        self.assertEqual(output["coordinate_units"], "arbitrary_monocular_slam_units")
        self.assertFalse(output["metric_calibrated"])
        self.assertFalse(output["keyframe_id_available"])

    def test_empty_exports_rejected(self):
        for poses, points in (("", "1 2 3\n"), ("0 0 0 0 0 0 0 1\n", "")):
            with self.subTest(poses=poses, points=points):
                p, q = self.exports(poses, points)
                with self.assertRaises(ValueError):
                    read_exports(p, q)

    def test_bad_numeric_and_quaternion_exports_rejected(self):
        for poses in ("0 nan 0 0 0 0 0 1\n", "0 0 0 0 0 0 0 0\n", "0 0 0\n",
                      "0 0 0 0 0 0 0 1\n0 1 0 0 0 0 0 1\n"):
            with self.subTest(poses=poses):
                p, q = self.exports(poses)
                with self.assertRaises(ValueError):
                    read_exports(p, q)

    def test_keyframe_timestamp_must_match_input(self):
        rgb = self.root / "rgb.txt"
        rgb.write_text("# one\n# two\n# three\n0.05 rgb/000002.png\n")
        p, q = self.exports()
        with self.assertRaisesRegex(ValueError, "unique match"):
            read_exports(p, q, rgb)

    def test_bad_rgb_list_rejected(self):
        rgb = self.root / "rgb.txt"
        for text in ("0 rgb/one.png\n", "#\n#\n#\n0 ../escape.png\n", "#\n#\n#\n0 one.png\n0 two.png\n"):
            rgb.write_text(text)
            with self.subTest(text=text), self.assertRaises(ValueError):
                read_rgb_list(rgb)

    def test_archive_path_escape_rejected(self):
        for name in ("../outside", "/absolute", "C:/drive", "back\\slash"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                destination(self.root, name)

    def test_file_tamper_rejected(self):
        path = self.root / "original.txt"
        path.write_bytes(b"original")
        refs = {"original": {"bytes": 8, "sha256": hashlib.sha256(b"original").hexdigest()}}
        checked_file(path, refs, "original")
        path.write_bytes(b"modified")
        with self.assertRaises(InvalidResult):
            checked_file(path, refs, "original")

    def test_incomplete_dataset_blocks_slam(self):
        from orb_io.preflight import check_dataset
        dataset = self.root / "video1_first20s"
        dataset.mkdir()
        (dataset / "rgb.txt").write_text("#\n#\n#\n" + "".join(f"{i/20:.6f} rgb/{i+1:06}.png\n" for i in range(400)))
        report = check_dataset(dataset)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["missing_frames"], 400)

    def test_empty_result_blocks_downstream_in_scheduler(self):
        graph = [Task("prepare_recording", handler="tests.orb_fixtures:short_recording"),
                 Task("run_slam", ("prepare_recording",), "tests.orb_fixtures:empty_exports"),
                 Task("next_module", ("run_slam",), "tests.fixtures:ok")]
        report = Scheduler(graph).run(self.root / "attempt", emit=None)
        self.assertEqual(report["tasks"]["run_slam"]["state"], "FAILED")
        self.assertEqual(report["tasks"]["next_module"]["state"], "BLOCKED")
        self.assertIsNone(report["tasks"]["run_slam"]["output"])

    def test_real_graph_connects_orb_and_requires_process_cleanup(self):
        bindings = {"handlers": {"prepare_recording": "team_adapters:prepare_recording", "run_slam": "team_adapters:run_slam"}}
        tasks = {task.name: task for task in build_tasks("real", bindings)}
        self.assertEqual(tasks["run_slam"].on_stop, "orb_adapter:stop_slam")
        self.assertTrue(tasks["run_slam"].requires_stop)
        self.assertIsNone(tasks["run_erfnet"].handler)
        self.assertEqual(Scheduler(orb_tasks()).order, ["prepare_recording", "run_slam", "publish_slam"])

    def test_supervised_command_receives_literal_arguments(self):
        folder = self.root / "native"
        code = "import pathlib,sys;pathlib.Path('arguments.json').write_text(__import__('json').dumps(sys.argv[1:]))"
        args = ["path with spaces", "literal;$value", "quoted\"argument"]
        report = execute_command([sys.executable, "-c", code, *args], folder, self.context())
        self.assertEqual(report["state"], "COMPLETED")
        self.assertTrue(report["termination_confirmed"])
        self.assertEqual(json.loads((folder / "arguments.json").read_text()), args)

    def test_status_publication_handles_reader_lock_without_hiding_permanent_failure(self):
        status = self.root / "status.json"
        status.write_text('{"state": "STARTING"}', encoding="utf-8")
        from dag_scheduler.engine import atomic_json
        attempts = []

        def temporarily_locked(path, value):
            attempts.append(True)
            if len(attempts) < 3:
                self.assertEqual(json.loads(status.read_text())["state"], "STARTING")
                raise PermissionError("File held by reader")
            atomic_json(path, value)

        with patch("orb_supervisor.atomic_json", side_effect=temporarily_locked):
            write_status(status, {"state": "RUNNING"})
        self.assertEqual(json.loads(status.read_text())["state"], "RUNNING")
        with patch("orb_supervisor.atomic_json", side_effect=PermissionError("Persistent lock")), \
                patch("orb_supervisor.time.monotonic", side_effect=[0, 2]):
            with self.assertRaises(PermissionError):
                write_status(status, {"state": "COMPLETED"})
        self.assertEqual(json.loads(status.read_text())["state"], "RUNNING")

    def test_nonzero_process_exit_is_not_success(self):
        folder = self.root / "native"
        with self.assertRaisesRegex(RuntimeError, "exit=7"):
            execute_command([sys.executable, "-c", "raise SystemExit(7)"], folder, self.context())
        report = json.loads((folder / "process_status.json").read_text())
        self.assertTrue(report["termination_confirmed"])
        self.assertEqual(report["return_code"], 7)

    def test_native_deadline_stops_child(self):
        folder = self.root / "native"
        with self.assertRaisesRegex(RuntimeError, "TIMED_OUT"):
            execute_command([sys.executable, "-c", "import time;time.sleep(60)"], folder, self.context(seconds=5.5))
        report = json.loads((folder / "process_status.json").read_text())
        self.assertTrue(report["termination_confirmed"])

    def test_killed_worker_lifeline_stops_native_child(self):
        cancel = threading.Event()
        folder = self.root / "attempt"
        status = folder / "w001_1/native/process_status.json"
        observed_running = []

        def watch():
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                try:
                    if json.loads(status.read_text())["state"] == "RUNNING":
                        observed_running.append(True)
                        break
                except (OSError, ValueError):
                    pass
                time.sleep(0.02)
            cancel.set()

        thread = threading.Thread(target=watch)
        thread.start()
        try:
            report = Scheduler([Task("run_slam", handler="tests.orb_fixtures:long_native", timeout_s=20,
                                     on_stop="orb_adapter:stop_slam", requires_stop=True, stop_timeout_s=10)]).run(
                                         folder, mode="real", cancellation=cancel, emit=None)
        finally:
            thread.join()
        diagnostic = {"scheduler": report, "native": status.read_text() if status.exists() else "missing"}
        self.assertTrue(observed_running, json.dumps(diagnostic, indent=2))
        self.assertEqual(report["status"], "CANCELLED")
        self.assertFalse(report["intervention_required"])
        self.assertTrue(report["tasks"]["run_slam"]["attempts"][0]["stop"]["confirmed"])
        native = json.loads(status.read_text())
        self.assertTrue(native["termination_confirmed"])
        self.assertEqual(native["state"], "CANCELLED")

    def test_unconfirmed_native_stop_is_not_accepted(self):
        operation = self.root / "operation"
        (operation / "native").mkdir(parents=True)
        (operation / "native/launch_intent.json").write_text("{}")
        context = replace(self.context(seconds=0.2), operation_directory=str(operation))
        self.assertFalse(stop_slam({}, context)["stopped"])

    def test_runtime_unavailable_has_clear_error(self):
        script = Path(__file__).resolve().parents[1] / "run_orb.py"
        result = subprocess.run([sys.executable, "-B", str(script), "--source", "execute", "--binary", str(self.root / "missing_binary"), "--check"],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn("Linux", result.stdout)


if __name__ == "__main__":
    unittest.main()
