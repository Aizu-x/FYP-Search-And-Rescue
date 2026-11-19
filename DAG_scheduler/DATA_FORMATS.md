# Module results - current FYP contract

All results are dictionaries of finite JSON-compatible values. The base validator is contracts.py. The connected ORB adapters also use orb_adapter.validate_output; other module formats remain proposed contracts until their adapters are implemented.

## Shared fields

- `schema_version`: integer 2, for the three-class format.
- `simulated`: true for demo data, false for real-module data.
- `attempt_id`: the current `context.run_id`.

`adapters.result(context, **data)` supplies these fields for a synchronous computation performed for this request. For remote responses, first verify the original response's request ID with `check_reply`. This is essential because delayed replies must not be relabelled as current.

## Tasks and their outputs

| Task | Receives | Returns, in addition to shared fields |
|---|---|---|
| prepare_recording | Context/configuration | recording_id, calibration_ref, frames |
| run_slam | prepare_recording | recording_id, keyframe_ids, poses_ref, geometry_ref |
| run_erfnet | run_slam | recording_id, matching keyframe_ids, class_names, labels_ref |
| fuse_map | run_slam, run_erfnet | recording_id, map identity/geometry, semantic_grid, alignment_ref |
| prepare_planning_map | fuse_map | matching map identity/geometry, blocked_grid, inflation_radius_m |
| select_target | fuse_map | matching map identity, target_id, position_m |
| get_ugv_pose | prepare_planning_map | matching map identity, position_m, yaw_rad, observed_at_unix_s |
| plan_path | prepare_planning_map, select_target, get_ugv_pose | matching map identity, target_id, path_id, status, waypoints_m, pose_observed_at_unix_s |
| navigate | plan_path | matching map identity, target_id, path_id, arrived, final_position_m |
| inspect_target | navigate | matching map identity, target_id, path_id, position_m, inspection_complete, finding |
| save_report | inspect_target | Created by mission.report_summary; no colleague's module needed |

Map identity means `map_id`, positive integer `map_version`, and `frame="arena"`. Map geometry carries explicit positive integer width/height, positive resolution_m, and finite origin_m=[x,y]. The old demo uses 60x60 at 0.05 m; supplied UGV maps use 100x100 at 0.03 m. Rows must match height and columns width. Planning-map preparation preserves the source map geometry. Start/goal/waypoint positions are `[x,y]` in metres, not pixels or row/column indices.

Each recording frame contains `frame_id`, `timestamp_s`, and `image_ref`. IDs must be unique; times must be finite, nonnegative, and strictly increasing. `*_ref` values are strings pointing to files or service-managed data. The base validator checks presence/identity. The ORB adapter additionally checks file hashes, finite export values, quaternion norms, matching timestamps, and selected images; those checks do not establish reconstruction accuracy. SLAM geometry must be scaled/aligned into the arena before a usable ground map can be produced.

The connected ORB result identifies `execution_kind` (archived_export_import or fresh_native_execution), `fresh_slam_run`, selected input images, and camera-to-world poses in arbitrary monocular units. `keyframe_ids` identify matched input frames, not internal ORB IDs. Calibration, metric scale, arena alignment, and navigation readiness remain false/unverified. See ORB_INTEGRATION.md for the full handoff. `simulated=false` means genuine data; it does not by itself mean ORB was freshly executed.

## Three semantic classes

| Value | Meaning | Planning policy |
|---|---|---|
| 0 | free space | May be traversable after clearance checks |
| 1 | obstacle | Forbidden |
| 2 | blocked | Forbidden |
| -1 | Unknown map data | Forbidden; not a fourth segmentation class |

`class_names` must be `["free space", "obstacle", "blocked"]` in that order. The planning grid uses 0 for free and 1 for blocked; all unknown/non-free semantic cells must remain blocked. Additional blocked cells may account for the robot's footprint. Grid indexing is `grid[row][column]`, rows increase with arena y, columns with arena x.

## Completion rules

- Planning succeeds only with PATH_FOUND, a nonempty route, consistent map/target identity, endpoints near the supplied start/goal, and a fresh start pose. NO_PATH blocks navigation.
- `pose_observed_at_unix_s` must match the input pose timestamp; it is checked again before navigation starts. It must not be replaced with the current time merely to hide stale localization.
- Navigation requires `arrived=True` and a final position near the route endpoint. Progress or command acknowledgement is not arrival.
- Inspection requires `inspection_complete=True`. Finding no victim may be a valid completed inspection; define the inspection time/window with the YOLO team.
- A module exception, invalid result, or expired time limit prevents success and blocks dependent tasks. Task results are stored only after validation.

These checks do not calculate A*, evaluate segmentation accuracy, project the aerial map, or verify that every route segment is collision-free. Those belong to the modules and their integration tests.
