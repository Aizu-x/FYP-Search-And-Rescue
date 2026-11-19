from dataclasses import replace
import json
from pathlib import Path
import tempfile
import time
import unittest

from dag_scheduler import Scheduler, Task, TaskContext
from dag_scheduler.engine import InvalidResult
from contracts import map_grid
from ugv_adapter import calculate_route, inspection_frames, planner_settings, validate_detection, ROOT
from combined_adapter import load_navigation_inputs
from run_combined import combined_tasks


def grid(size=10, resolution=0.1):
    return {"map_id": "test", "map_version": 1, "frame": "arena", "width": size, "height": size,
            "resolution_m": resolution, "origin_m": [0.0, 0.0], "inflation_radius_m": 0.0,
            "blocked_grid": [[0] * size for _ in range(size)]}


class UgvTests(unittest.TestCase):
    def setUp(self):
        self.scratch = Path(__file__).resolve().parent / "_scratch"
        self.scratch.mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=self.scratch)
        self.root = Path(self.tmp.name)

    def tearDown(self):
        assert self.root.resolve().is_relative_to(self.scratch.resolve())
        self.tmp.cleanup()

    def context(self, task="load_navigation_inputs", parameters=None):
        return TaskContext("mission", "run", task, "request", 1, 1, "real", time.monotonic() + 30,
                           str(self.root), parameters or {})

    def test_both_map_resolutions_are_preserved(self):
        for size, resolution in ((60, 0.05), (100, 0.03)):
            data = grid(size, resolution)
            map_grid(data, "blocked_grid", {0, 1})
            self.assertEqual(data["resolution_m"], resolution)

    def test_inconsistent_dimensions_and_unknown_occupancy_rejected(self):
        for key, value in (("width", 9), ("height", 0), ("resolution_m", 0)):
            data = grid(); data[key] = value
            with self.assertRaises(InvalidResult):
                map_grid(data, "blocked_grid", {0, 1})
        data = grid(); data["blocked_grid"][2][2] = -1
        with self.assertRaises(InvalidResult):
            calculate_route(data, [.15, .15], [.85, .85])

    def test_original_astar_checks_exact_endpoints(self):
        output = calculate_route(grid(), [.16, .16], [.84, .84], {"radius_m": 0, "margin_m": 0, "clearance_weight": 0})
        self.assertEqual(output["waypoints_m"][0], [.16, .16])
        self.assertEqual(output["waypoints_m"][-1], [.84, .84])
        self.assertTrue(output["collision_checked"])
        self.assertEqual(output["planner"], "ugv_planner.astar.plan")

    def test_disconnected_map_has_no_usable_route(self):
        data = grid(); data["blocked_grid"][5] = [1] * 10
        with self.assertRaisesRegex(InvalidResult, "did not find"):
            calculate_route(data, [.15, .15], [.85, .85], {"radius_m": 0, "margin_m": 0})

    def test_goal_connector_cannot_enter_clearance_zone(self):
        data = grid(); data["blocked_grid"][7][7] = 1
        with self.assertRaises(InvalidResult):
            calculate_route(data, [.25, .25], [.699, .75], {"radius_m": .02, "margin_m": 0})

    def test_same_cell_retains_distinct_exact_goal(self):
        output = calculate_route(grid(), [.42, .43], [.47, .48], {"radius_m": 0, "margin_m": 0})
        self.assertEqual(output["waypoints_m"][0], [.42, .43])
        self.assertEqual(output["waypoints_m"][-1], [.47, .48])

    def test_reference_maps_match_upstream_orientation_without_gt_input(self):
        import numpy as np
        from ugv_nav.map_store import load_env
        for name in ("ENV1", "ENV2", "ENV3"):
            data = load_navigation_inputs({}, self.context(parameters={"reference": {"environment": name}}))
            expected, _ = load_env(name)
            np.testing.assert_array_equal(data["blocked_grid"], expected.cells)
            self.assertFalse(data["derived_from_current_orb"])
            self.assertFalse(data["start_is_measured"])
            self.assertNotIn("GT", data["source_ref"])

    def test_bad_radius_or_configuration_rejected(self):
        for settings in ({"radius_m": -1}, {"margin_m": float("nan")}, {"unknown": 3}):
            with self.assertRaises(ValueError):
                planner_settings(settings)

    def navigation(self):
        return {"arrived": True, "request_id": "navigation-123", "arrival_observed_at_unix_s": 100,
                "inspection_frames": [{"source": "ugv_camera", "navigation_request_id": "navigation-123",
                                       "capture_timestamp_unix_s": 101, "sha256": "abc", "image_ref": "capture.png"}]}

    def test_inspection_requires_matching_fresh_post_arrival_capture(self):
        navigation = self.navigation()
        self.assertEqual(len(inspection_frames(navigation, 102)), 1)
        for field, bad in (("source", "repository_example"), ("navigation_request_id", "old"),
                           ("capture_timestamp_unix_s", 99), ("capture_timestamp_unix_s", 104), ("sha256", "")):
            navigation = self.navigation()
            navigation["inspection_frames"][0][field] = bad
            with self.subTest(field=field, bad=bad), self.assertRaises(InvalidResult):
                inspection_frames(navigation, 102)
        with self.assertRaises(InvalidResult):
            inspection_frames(self.navigation(), 200)

    def test_inspection_cannot_invent_arrival(self):
        navigation = self.navigation(); navigation["arrived"] = False
        with self.assertRaises(InvalidResult):
            inspection_frames(navigation, 102)

    def test_detections_validate_class_confidence_and_coordinates(self):
        detection = {"class_id": 0, "class_name": "green_obstacle", "confidence": .8, "bbox_xyxy": [1, 2, 10, 20]}
        validate_detection(detection, 100, 100)
        for key, value in (("class_name", "person"), ("confidence", float("nan")), ("bbox_xyxy", [-1, 0, 10, 20])):
            with self.assertRaises(InvalidResult):
                validate_detection({**detection, key: value}, 100, 100)

    def test_combined_graph_joins_all_three_components(self):
        tasks = combined_tasks()
        scheduler = Scheduler(tasks)
        scheduler.preflight()
        parents = set(tasks[-1].needs)
        self.assertEqual(parents, {"run_slam", "load_navigation_inputs", "plan_reference_route", "inspect_images"})
        self.assertNotIn("navigate", scheduler.order)

    def test_astar_failure_blocks_combined_report(self):
        tasks = [Task("load_navigation_inputs", handler="combined_adapter:load_navigation_inputs", validator="combined_adapter:validate_output"),
                 Task("plan_reference_route", ("load_navigation_inputs",), "combined_adapter:plan_reference_route", validator="combined_adapter:validate_output"),
                 Task("finish", ("plan_reference_route",), "tests.fixtures:ok")]
        report = Scheduler(tasks).run(self.root / "run", mode="real", emit=None,
            parameters={"reference": {"environment": "ENV1", "start_m": [.015, .015], "goal_m": [2.5, 2.5]}})
        self.assertEqual(report["tasks"]["plan_reference_route"]["state"], "FAILED")
        self.assertEqual(report["tasks"]["finish"]["state"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
