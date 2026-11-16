# Integration boundary and remaining deployment work

The current executable runs entirely on the laptop. The eventual deployment is:

`UAV / ORB-SLAM3 → edge ERFNet + calibrated map conversion → UGV grid receiver → A* → UGV motion controller`

Keep GT evaluation on the experiment/reporting side. The physical robot will not have access to ground truth during deployment.

## Reuse without the GUI

```python
from ugv_planner.interfaces import NavigationService
from ugv_planner import PlannerConfig, Footprint

navigation = NavigationService(config=PlannerConfig(footprint=Footprint()))
# When actual messages arrive, the surrounding integration process calls:
# navigation.receive_grid(grid)       # Grid in a consistent metric map frame
# navigation.update_pose((x_m, y_m))  # localization transformed into that frame
# navigation.set_goal((gx_m, gy_m))
# controller.stop()
# result = navigation.replan()
# if result.found:
#     controller.follow_waypoints(result.waypoints)
```

These comments show the transport/controller boundary; they are not a ROS node or a working hardware controller. The planner and NavigationService themselves are complete and exercised by tests. `MotionController` is a typing protocol specifying the future `follow_waypoints` and `stop` contract, not an implemented motor driver.

`grid_from_ros_values()` is an independently testable data adapter for flattened bottom-first occupancy values: unknown `-1` is blocked, occupancy ≥ threshold is blocked, default threshold 50. It accepts resolution and a translational origin. It rejects invalid occupancy values and payload lengths. It does not subscribe, publish, parse ROS messages, or handle rotated origins. Transform/rectify rotated grids upstream before calling it.

## ROS2 / MentorPi work still required — untested

1. Verify the MentorPi software, supported ROS2 distribution and message/topic conventions on the actual hardware. No version or vendor API is assumed here.
2. Agree an edge-to-UGV map transport. Include resolution, frame ID, origin orientation, timestamp, map version, and unknown-cell semantics. Binary ERFNet classes alone are insufficient for metric registration.
3. Confirm source image orientation, scale, extrinsic calibration and GT/occupancy registration. ORB-SLAM3 outputs must be transformed into the same frame as the navigation grid and UGV pose.
4. Implement ROS subscriptions and TF transforms around `NavigationService`; validate timestamps and reject stale/unsupported frames. Invoke stop before replacing maps, goals or plans. Replan from latest localization, not from assumed open-loop motion.
5. Implement/publish waypoint paths in the selected map frame. This project's `(x,y)` output has no heading requirement; ROS poses need a deliberate orientation policy. A mecanum robot can translate sideways but still has orientation-dependent hardware clearance.
6. Measure robot body/attachments and choose a conservative bounding radius plus localization/calibration/tracking safety margins. Validate rotational clearance. A future rectangular/polygon footprint needs yaw-aware state and swept polygon geometry, or a conservative enclosing circle.
7. Replace `SimulatedUGV` with a closed-loop waypoint controller using odometry/IMU/localization. Define speed/acceleration limits, heading behavior, waypoint acceptance, tracking error bounds, timeout handling, stop commands and recovery states.
8. Add LiDAR-based local obstacle handling, emergency stops, dynamic-map changes, and controller watchdogs. A* is the global planner and cannot supply these responsibilities.
9. Protect concurrency with immutable map snapshots/version IDs. Discard stale planning results if map or goal changes. Add asynchronous cancellation and telemetry as needed on the actual processor.
10. Test sensor/frame correctness, stationary geometry, then low-speed motion under supervision, before any search-and-rescue demonstration. Record localization/tracking errors and clearance margins.

Successful offline simulation proves the executed software cases only. Neither ROS communication nor real motion, sensor fusion, obstacle avoidance, or hardware safety has been tested here.
