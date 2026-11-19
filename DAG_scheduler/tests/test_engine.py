from dataclasses import replace
import json
import multiprocessing
from pathlib import Path
import tempfile
import threading
import time
import unittest

from dag_scheduler import Scheduler, Task, topological_order
from dag_scheduler.engine import ConfigurationError
from mission import run_mission


def task(name="one", needs=(), handler="ok", **kwargs):
    return Task(name, needs, "tests.fixtures:" + handler, timeout_s=5, **kwargs)


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.scratch = Path(__file__).resolve().parent / "_scratch"
        self.scratch.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=self.scratch)
        self.folder = Path(self.temporary.name)

    def tearDown(self):
        self.assertFalse([child for child in multiprocessing.active_children() if child.name.startswith("dag-")])
        self.assertTrue(self.folder.resolve().is_relative_to(self.scratch.resolve()))
        self.temporary.cleanup()

    def run_tasks(self, tasks, **kwargs):
        return Scheduler(tasks).run(self.folder / "attempt", emit=None, **kwargs)

    def test_kahn_reversed_graph_and_join(self):
        tasks = [task("join", ("left", "right", "left")), task("right"), task("left")]
        report = self.run_tasks(tasks)
        self.assertEqual(report["status"], "SUCCEEDED")
        self.assertEqual(set(report["tasks"]["join"]["output"]["seen"]), {"left", "right"})
        events = [json.loads(line) for line in (self.folder / "attempt/events.jsonl").read_text().splitlines()]
        started = [e["task"] for e in events if e["state"] == "RUNNING"]
        self.assertEqual(started[-1], "join")
        self.assertGreater(report["duration_s"], 0)
        self.assertTrue(all(r["attempts"][0]["duration_s"] > 0 for r in report["tasks"].values()))
        self.assertEqual(json.loads((self.folder / "attempt/report.json").read_text())["status"], "SUCCEEDED")

    def test_invalid_graphs_rejected(self):
        for graph in ([], [task(), task()], [task(needs=("missing",))],
                      [task(needs=("one",))], [task("a", ("b",)), task("b", ("a",))],
                      [task("../escape")]):
            with self.subTest(graph=graph), self.assertRaises(ConfigurationError):
                topological_order(graph)

    def test_preflight_checks_all_bindings(self):
        for changed in (replace(task("later"), handler=None),
                        replace(task("later"), handler="missing_module:missing"),
                        replace(task("later"), handler="tests.fixtures:wrong_arguments"),
                        replace(task("later"), handler="team_adapters:fuse_map")):
            with self.subTest(changed=changed), self.assertRaises(ConfigurationError):
                self.run_tasks([task("first"), changed])
            self.assertFalse((self.folder / "attempt").exists())

    def test_invalid_policies(self):
        for changes in ({"timeout_s": 0}, {"timeout_s": float("nan")},
                        {"max_attempts": 2}, {"max_attempts": True},
                        {"retry_delay_s": -1}, {"requires_stop": True}, {"idempotent": "false"}):
            with self.subTest(changes=changes), self.assertRaises(ConfigurationError):
                Scheduler([replace(task(), **changes)]).preflight()

    def test_failure_blocks_descendants_and_cancels_unrelated(self):
        report = self.run_tasks([task("bad", handler="fail"), task("child", ("bad",)), task("other")])
        states = {name: row["state"] for name, row in report["tasks"].items()}
        self.assertEqual(states, {"bad": "FAILED", "child": "BLOCKED", "other": "CANCELLED"})
        self.assertIsNone(report["tasks"]["bad"]["output"])

    def test_validation_prevents_publishing(self):
        report = self.run_tasks([task(validator="tests.fixtures:reject"), task("child", ("one",))])
        self.assertEqual(report["tasks"]["one"]["attempts"][0]["status"], "INVALID_RESULT")
        self.assertIsNone(report["tasks"]["one"]["output"])
        self.assertEqual(report["tasks"]["child"]["state"], "BLOCKED")

    def test_input_validation_before_call(self):
        marker = self.folder / "started.txt"
        report = self.run_tasks([task(handler="hang", before="tests.fixtures:reject_before")], parameters={"marker": str(marker)})
        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(marker.exists())

    def test_nonfinite_json_rejected(self):
        report = self.run_tasks([task(handler="nan")])
        self.assertEqual(report["tasks"]["one"]["attempts"][0]["status"], "INVALID_RESULT")

    def test_non_dictionary_rejected(self):
        report = self.run_tasks([task(handler="wrong_type")])
        self.assertEqual(report["status"], "FAILED")

    def test_crash_reported(self):
        report = self.run_tasks([task(handler="crash")])
        self.assertEqual(report["status"], "FAILED")
        self.assertIn("code 3", report["tasks"]["one"]["attempts"][0]["error"])

    def test_timeout_terminates_worker_and_confirms_stop(self):
        slow = replace(task(handler="hang", on_stop="tests.fixtures:stop", requires_stop=True), timeout_s=0.7)
        report = self.run_tasks([slow, task("child", ("one",))])
        record = report["tasks"]["one"]
        self.assertEqual(record["state"], "TIMED_OUT")
        self.assertTrue(record["attempts"][0]["stop"]["confirmed"])
        self.assertFalse(report["intervention_required"])
        self.assertEqual(report["tasks"]["child"]["state"], "BLOCKED")

    def test_active_cancellation_and_stop(self):
        marker = self.folder / "started.txt"
        cancel = threading.Event()

        def cancel_when_running():
            deadline = time.monotonic() + 5
            while not marker.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            cancel.set()

        watcher = threading.Thread(target=cancel_when_running)
        watcher.start()
        try:
            report = self.run_tasks([task(handler="hang", on_stop="tests.fixtures:stop", requires_stop=True)],
                                    parameters={"marker": str(marker)}, cancellation=cancel)
        finally:
            watcher.join()
        self.assertTrue(marker.exists())
        self.assertEqual(report["status"], "CANCELLED")
        self.assertTrue(report["tasks"]["one"]["attempts"][0]["stop"]["confirmed"])

    def test_cancel_before_start(self):
        cancel = threading.Event()
        cancel.set()
        report = self.run_tasks([task()], cancellation=cancel)
        self.assertEqual(report["status"], "CANCELLED")
        self.assertEqual(report["tasks"]["one"]["attempts"], [])

    def test_retry_only_after_retryable_failure(self):
        report = self.run_tasks([task(handler="fail_once", max_attempts=2, idempotent=True)])
        attempts = report["tasks"]["one"]["attempts"]
        self.assertEqual([a["status"] for a in attempts], ["FAILED", "SUCCEEDED"])
        self.assertNotEqual(attempts[0]["request_id"], attempts[1]["request_id"])

    def test_invalid_output_is_not_retried(self):
        report = self.run_tasks([task(validator="tests.fixtures:reject", max_attempts=2, idempotent=True)])
        self.assertEqual(len(report["tasks"]["one"]["attempts"]), 1)

    def test_inputs_are_isolated(self):
        report = self.run_tasks([task("root"), task("child", ("root",), handler="mutate")])
        self.assertEqual(report["tasks"]["root"]["output"]["nested"]["value"], 1)

    def test_async_handler_requires_an_adapter(self):
        with self.assertRaisesRegex(ConfigurationError, "synchronous adapter"):
            Scheduler([task(handler="async_ok")]).preflight()

    def test_replanning_uses_new_run_and_acknowledged_stop(self):
        tasks = [task(handler="replan", on_stop="tests.fixtures:stop", requires_stop=True)]
        summary, folder = run_mission(tasks, self.folder, max_replans=1, emit=None)
        self.assertEqual([a["status"] for a in summary["attempts"]], ["REPLAN_REQUIRED", "SUCCEEDED"])
        self.assertNotEqual(summary["attempts"][0]["run_id"], summary["attempts"][1]["run_id"])
        report = json.loads((folder / "attempt_2/report.json").read_text())
        self.assertEqual(report["tasks"]["one"]["output"]["replan_event"], {"blocked_cell": [4, 8]})

    def test_unconfirmed_stop_blocks_replanning(self):
        for stop in ("stop_false", "stop_stale", "fail", "hang"):
            with self.subTest(stop=stop):
                operation = replace(task(handler="replan", on_stop=f"tests.fixtures:{stop}", requires_stop=True), stop_timeout_s=0.7)
                summary, folder = run_mission([operation], self.folder, max_replans=2, emit=None)
                self.assertEqual(len(summary["attempts"]), 1)
                self.assertTrue(summary["attempts"][0]["intervention_required"])

    def test_replanning_budget_is_enforced(self):
        summary, folder = run_mission([task(handler="replan")], self.folder, max_replans=0, emit=None)
        self.assertEqual(len(summary["attempts"]), 1)
        self.assertEqual(summary["status"], "REPLAN_REQUIRED")


if __name__ == "__main__":
    unittest.main()
