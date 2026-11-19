# Connections and remaining requirements

Updated 17 November 2026. This combines the earlier DAG/ORB package with Aizu-x/UGV_A-Star-YOLOV5 at commit `e9ed19bbf93e812a4863055751381f70ab4d6e61`. Its planner, detector, console and bridge are preserved.

> **FYP_SRM note, 18 November 2026.** In this folder the package sits on a newer version of the UGV project
> (commit `cb9fe29`: mission video recording and the UAV section were added after `e9ed19b`). The planner,
> navigation, detector and map files that the adapters call are the same, byte for byte, in both versions, and
> the checks were re-run on the newer one. What was merged and changed: [MERGE_NOTES.md](MERGE_NOTES.md).

## What the combined runner does

The DAG describes prerequisites. Kahn's algorithm identifies tasks whose prerequisites have finished. The scheduler runs one ready task at a time, validates its output and blocks dependent tasks after failure.

The new runner checks three components in one execution:

1. Prepare a supplied Tello recording, then import its saved ORB result or run ORB inside Linux.
2. Load a supplied occupancy map and calculate a route using the colleague's original A*.
3. Run the actual YOLOv5n ONNX model on a finite batch of saved images.
4. Join all accepted outputs into one report.

The occupancy map and detector images are separate inputs. They are not claimed to come from the current ORB run or from a UGV that has arrived somewhere. The joined result establishes component integration, not an autonomous physical mission.

## Files that connect the modules

| File | Purpose |
|---|---|
| DAG_scheduler/dag_scheduler/engine.py | Kahn scheduling, task workers, timeouts, cancellation, reports |
| DAG_scheduler/orb_adapter.py | ORB input checks, saved exports and native Linux execution |
| DAG_scheduler/orb_supervisor.py | Native process lifetime and confirmed cleanup |
| DAG_scheduler/ugv_adapter.py | Actual A* and YOLO calls; production mission wrappers |
| DAG_scheduler/combined_adapter.py | Reference inputs and combined report |
| DAG_scheduler/run_combined.py | Six-task component graph |
| DAG_scheduler/team_adapters.py | Entry points for the larger mission |
| DAG_scheduler/real_bindings.json | Four connected handlers; missing ones remain null |
| ugv_planner/astar.py | Colleague's original A*, unchanged |
| third_party/wro_yolo/wro_detector.py | Colleague's YOLOv5n inference, unchanged |

## Map and route handling

The previous prototype used 60x60 cells at 0.05 m. The supplied UGV maps use 100x100 cells at 0.03 m. The contract now validates explicit dimensions, resolution, origin and row lengths. It does not force a 3 cm map into the previous 5 cm format.

The reference loader reads only `ENVn_occupancy.npy`, flips image row order and applies the same ENV2/ENV3 rotations as the upstream console. Ground truth is not used to plan or choose endpoints. Console map edits and physical field-size adjustments are not automatically imported. Scale remains nominal until measured.

The wrapper calls `NavigationService.replan()`, which calls `ugv_planner.astar.plan`. It checks the connector from the exact start and to the exact requested goal, then validates every route segment with the supplied collision checker. Default radius is 0.143 m plus a 0.02 m margin, from the supplied configuration and still requiring physical verification. A pre-inflated map receives additional conservative clearance; prefer uninflated binary occupancy with `inflation_radius_m=0`.

Upstream must explicitly convert unknown/non-free semantic cells to occupied cells. A* receives binary occupancy: 0 free, 1 occupied. Do not cast a three-class mask directly into this format.

Reference endpoints are examples, not measured UGV positions. Some fail at the configured radius. ENV2's default goal was rejected during testing because it is within the inflated obstacle region. Supply a checked goal in the same map coordinates; the adapter does not shrink the robot or move a goal silently.

## Detector handling

The model returns `green_obstacle` and `red_obstacle` boxes in original-image pixels. The console uses green blocks as survivor markers; this is not a trained person detector. The runner saves detections, image/model hashes, dimensions and timing. Included example images already contain annotations and do not establish independent detection accuracy.

Zero detections is a completed inference result, not proof that the scene is clear. Pixel boxes are not automatically converted into map positions.

The production `inspect_target` wrapper requires a finite batch of fresh UGV images captured after confirmed arrival and associated with that navigation request. Saved examples cannot replace a real target inspection. In addition to the existing arrival/map/path fields, the navigation adapter must provide:

```json
{
  "request_id": "original-navigation-request-id",
  "arrival_observed_at_unix_s": 1234567890.0,
  "inspection_frames": [{
    "source": "ugv_camera",
    "navigation_request_id": "original-navigation-request-id",
    "frame_id": "camera-frame-id",
    "capture_timestamp_unix_s": 1234567890.1,
    "image_ref": "/path/to/captured-frame.png",
    "sha256": "captured-file-sha256"
  }]
}
```

These are example definitions, not actual timestamps or captures. Use the edge clock or an explicitly synchronized clock. The default freshness window is 30 seconds, including model loading/inference.

## What still needs finishing

| Item | Reason and current material |
|---|---|
| Confirm detector version | Supplied code/weights are v5n. If v8 is mandatory, send the actual v8 model and inference code |
| Fresh ORB run | Source/assets are supplied; native compilation and tracking remain unverified here |
| Tello calibration | Supplied YAML is provisional |
| Metric scale and alignment | Monocular ORB points are not metres and are not aligned with arena/UGV coordinates |
| ORB/ERFNet ground-map fusion | Saved occupancy maps exist, but sparse ORB points are not themselves a traversable map |
| Recording interface | Current ORB adapter accepts the two fixed 400-frame handoffs; a new recording/live capture needs an extended input adapter |
| Request-bound UGV task service | The underlying controller/bridge exists, but DAG pose, navigation, arrival, post-arrival capture and confirmed-stop adapters remain unconnected |
| Physical validation | Check actual dimensions, camera mounting, alignment, stopping and route clearance on the MentorPi |

Do not replace original recording files with new images: their hash checks intentionally reject changed inputs.

`python DAG_scheduler/run_mission.py --mode real --check` deliberately reports the missing ERFNet, fusion, planning-map, target, localization, navigation and stop bindings. A* and inspection wrappers are registered; the incomplete full mission cannot run. The original complete simulated mission remains available separately.

The included console retains its own simulator, authorization, stop behavior and physical-control workflow. It is not automatically run by the component scheduler, and no motors were commanded during this integration.
