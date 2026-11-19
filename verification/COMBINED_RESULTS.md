# Combined integration verification

Date: 17 November 2026. Platform: Windows, Python 3.12.14. Upstream UGV revision: `e9ed19bbf93e812a4863055751381f70ab4d6e61`.

## Completed checks

- **62 scheduler/adapter tests passed** in 35.682 seconds: 49 previous scheduler/mission/ORB checks and 13 new UGV integration checks.
- **23 upstream system tests passed** in 12.360 seconds, covering A* optimality against an independent Dijkstra reference, collision geometry, original datasets, endpoint connections and map adapters. This is not a rerun of the entire upstream console/robot test suite.
- Two full six-task component runs succeeded with actual supplied files. ORB imported saved exports; A* and YOLO inference ran during these checks.

| Recording | Saved ORB poses | Saved map points | A* map | Route waypoints | Route length | YOLO result |
|---|---:|---:|---|---:|---:|---|
| Video 1 | 17 | 1,755 | ENV1 saved occupancy | 126 | 4.19735 m nominal | 4 detections across 3 images |
| Video 2 | 21 | 1,943 | ENV1 saved occupancy | 126 | 4.19735 m nominal | 4 detections across 3 images |

The complete local runs are `combined_79a26e1a6e23` and `combined_49565f869083` under the ignored combined_runs folder. The committed summary records are portable excerpts, not fresh native SLAM evidence.

The planner used the supplied 0.143 m radius plus 0.02 m margin and checked the exact endpoint connections and route segments. The model SHA-256 was `a52ddba025c6b54cd0a2326905cafe8d377d0f906c91bcac3669cdfa4c4bcaad`.

A separate ENV2 run rejected its supplied example goal as occupied/within planning inflation. Its combined report task remained BLOCKED, as required. The endpoint was not silently changed and the radius was not reduced.

Map orientation checks cover ENV1, ENV2 and ENV3. New tests also reject inconsistent map dimensions, unknown occupancy, disconnected routes, unsafe goal connectors, stale/mismatched inspection frames and invalid detector classes/boxes. A failed A* task blocks its dependent report.

## Limits

This does not verify new ORB tracking, ORB-to-grid fusion, camera calibration, real-world alignment, human detection accuracy or physical MentorPi operation. All three components ran under one scheduler with explicit separate data provenance. No robot commands were sent. The production A* and inspection wrappers are registered; the complete real mission still fails preflight because other task interfaces are absent.

The upstream controller/console remains included and unchanged. Its reported hardware history belongs to the upstream project and was not reverified here. Native ORB execution and Linux process-group cleanup remain target-device checks.

## Reproduce

After EDGE_SETUP.md:

```text
python scripts/verify_combined.py
python -m unittest discover -s tests -p test_system.py -v
python DAG_scheduler/run_combined.py --sequence video1 --environment ENV1
python DAG_scheduler/run_combined.py --sequence video2 --environment ENV1
```

Main packages: NumPy 2.5.5, Matplotlib 3.11.2, OpenCV headless 4.14.0.94, ONNX Runtime 1.32.0. Full local test logs are under verification/local and are excluded from Git.

## Re-run in the FYP_SRM folder

Date: 18 November 2026. Platform: Windows 11, Python 3.14.6. UGV revision: `cb9fe2947c893178888d4f071a7ca05ca2b2c79b` (newer than above: mission video recording and the UAV section were added). Main packages: NumPy 2.5.4, Matplotlib 3.11.2, OpenCV headless 4.14.0.94, ONNX Runtime 1.31.1.

- **62 scheduler/adapter tests passed** (`python scripts/verify_combined.py`, 18 seconds), with no change to the scheduler or adapter code.
- Two full six-task component runs succeeded and gave the same figures as the table above: Video 1, 17 poses and 1,755 points; Video 2, 21 poses and 1,943 points; both with 126 waypoints, 4.19735 m nominal, and 4 detections across 3 images. The model SHA-256 began `a52ddba025c6`, as above.
- `python DAG_scheduler/run_mission.py --mode real --check` reported the same missing bindings as before (ERFNet, fusion, planning map, target, pose, navigation and its stop callback) and exited with code 2, as intended.
- The standalone simulated mission (`python DAG_scheduler/run_mission.py`) succeeded.

The limits above are unchanged. No robot or drone commands were sent, and no fresh ORB tracking was run. Details of the merge: [../MERGE_NOTES.md](../MERGE_NOTES.md).
