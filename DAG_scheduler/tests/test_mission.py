from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from adapters import check_reply
from contracts import validate, before
from dag_scheduler import Scheduler, TaskContext
from dag_scheduler.engine import ConfigurationError, InvalidResult
from mission import build_tasks, run_mission


class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scratch = Path(__file__).resolve().parent / "_scratch"
        cls.scratch.mkdir(exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(dir=cls.scratch)
        cls.root = Path(cls.temporary.name)
        cls.baseline = Scheduler(build_tasks("demo")).run(cls.root / "baseline", emit=None)

    @classmethod
    def tearDownClass(cls):
        assert cls.root.resolve().is_relative_to(cls.scratch.resolve())
        cls.temporary.cleanup()

    def context(self, name, mode="demo"):
        return TaskContext("test", self.baseline["run_id"], name, "request-123", 1, 1,
                           mode, 0, str(self.root), {})

    def test_complete_mission(self):
        self.assertEqual(self.baseline["status"], "SUCCEEDED")
        self.assertEqual(len(self.baseline["tasks"]), 11)
        self.assertEqual(self.baseline["tasks"]["run_erfnet"]["output"]["class_names"],
                         ["free space", "obstacle", "blocked"])

    def test_invalid_module_data(self):
        cases = [
            ("prepare_recording", lambda v: v.update(schema_version=1)),
            ("prepare_recording", lambda v: v.update(simulated=False)),
            ("prepare_recording", lambda v: v.update(attempt_id="old-run")),
            ("prepare_recording", lambda v: v["frames"][1].update(timestamp_s=0)),
            ("run_slam", lambda v: v.update(keyframe_ids=["unknown"])),
            ("run_erfnet", lambda v: v.update(class_names=["wrong"])),
            ("fuse_map", lambda v: v["semantic_grid"][0].__setitem__(0, 3)),
            ("fuse_map", lambda v: v.update(width=59)),
            ("prepare_planning_map", lambda v: v["blocked_grid"][20].__setitem__(20, 0)),
            ("select_target", lambda v: v.update(position_m=[3.0, 0.0])),
            ("get_ugv_pose", lambda v: v.update(frame="camera")),
            ("get_ugv_pose", lambda v: v.update(observed_at_unix_s=0)),
            ("plan_path", lambda v: v.update(waypoints_m=[])),
            ("plan_path", lambda v: v.update(map_version=999)),
            ("plan_path", lambda v: v.update(status="NO_PATH")),
            ("plan_path", lambda v: v.update(pose_observed_at_unix_s=0)),
            ("navigate", lambda v: v.update(arrived=False)),
            ("navigate", lambda v: v.update(path_id="old-path")),
            ("navigate", lambda v: v.update(final_position_m=[2.0, 2.0])),
            ("inspect_target", lambda v: v.update(inspection_complete=False)),
        ]
        for name, corrupt in cases:
            with self.subTest(task=name):
                value = deepcopy(self.baseline["tasks"][name]["output"])
                corrupt(value)
                inputs = {n: row["output"] for n, row in self.baseline["tasks"].items()}
                fresh_time = inputs["get_ugv_pose"]["observed_at_unix_s"]
                with patch("contracts.time.time", return_value=fresh_time), self.assertRaises(InvalidResult):
                    validate(value, inputs, self.context(name))

    def test_all_baseline_contracts_accept_valid_outputs(self):
        inputs = {n: row["output"] for n, row in self.baseline["tasks"].items()}
        fresh_time = inputs["get_ugv_pose"]["observed_at_unix_s"]
        with patch("contracts.time.time", return_value=fresh_time):
            for name, value in inputs.items():
                validate(value, inputs, self.context(name))

    def test_real_contract_does_not_accept_simulation(self):
        data = deepcopy(self.baseline["tasks"]["prepare_recording"]["output"])
        with self.assertRaises(InvalidResult):
            validate(data, {}, self.context("prepare_recording", "real"))
        data["simulated"] = False
        validate(data, {}, self.context("prepare_recording", "real"))

    def test_stale_route_rejected_before_navigation(self):
        path = deepcopy(self.baseline["tasks"]["plan_path"]["output"])
        path["pose_observed_at_unix_s"] = 0
        with self.assertRaises(InvalidResult):
            before({"plan_path": path}, self.context("navigate"))

    def test_delayed_remote_reply_is_rejected(self):
        with self.assertRaises(InvalidResult):
            check_reply({"request_id": "previous-request"}, self.context("navigate"))

    def test_pose_age_setting_is_checked_and_used(self):
        path = deepcopy(self.baseline["tasks"]["plan_path"]["output"])
        path["pose_observed_at_unix_s"] = 100.0
        with patch("contracts.time.time", return_value=106.0):
            with self.assertRaises(InvalidResult):
                before({"plan_path": path}, self.context("navigate"))
            configured = replace(self.context("navigate"), parameters={"max_pose_age_s": 10.0})
            before({"plan_path": path}, configured)
            with self.assertRaises(InvalidResult):
                before({"plan_path": path}, replace(configured, parameters={"max_pose_age_s": float("nan")}))

    def test_missing_real_modules_rejected_before_run(self):
        with self.assertRaises(ConfigurationError):
            Scheduler(build_tasks("real")).preflight()

    def test_real_mode_disallows_demo_bindings(self):
        with self.assertRaises(ConfigurationError):
            build_tasks("real", {"handlers": {"plan_path": "demo_modules:execute"}})

    def test_no_path_blocks_navigation(self):
        report = Scheduler(build_tasks("demo")).run(self.root / "no_path", emit=None,
                                                     parameters={"scenario": "no_path"})
        self.assertEqual(report["status"], "FAILED")
        self.assertEqual(report["tasks"]["navigate"]["state"], "BLOCKED")
        self.assertIsNone(report["tasks"]["plan_path"]["output"])

    def test_full_replan_refreshes_map_and_pose(self):
        summary, folder = run_mission(build_tasks("demo"), self.root, emit=None,
                                      parameters={"scenario": "replan"}, max_replans=1)
        self.assertEqual(summary["status"], "SUCCEEDED")
        self.assertEqual(len(summary["attempts"]), 2)
        second = json.loads((folder / "attempt_2/report.json").read_text())
        self.assertEqual(second["tasks"]["fuse_map"]["output"]["map_version"], 2)
        self.assertEqual(second["tasks"]["fuse_map"]["output"]["semantic_grid"][30][30], 2)
        self.assertEqual(second["tasks"]["get_ugv_pose"]["output"]["position_m"], [0.4, 0.25])

    def test_cli_preflight_and_graph(self):
        script = str(Path(__file__).resolve().parents[1] / "run_mission.py")
        for args, expected in ((["--check"], 0), (["--mode", "real", "--check"], 2),
                               (["--show-order"], 0), (["--max-replans", "-1"], 2)):
            with self.subTest(args=args):
                result = subprocess.run([sys.executable, "-B", script, *args], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, expected, result.stderr)


if __name__ == "__main__":
    unittest.main()
