# What colleagues need to send for DAG integration

Combined-project update: the supplied UGV repository is now included and its A* and YOLOv5n wrappers are connected. See ../COMBINED_INTEGRATION.md for the current, reduced list of missing items. This older checklist is retained as background; it is not a request to resend files already supplied.

Prepared 31 October 2026.

Update, 17 November: the ORB source, assets, both complete recordings, and saved results have been received. Their DAG adapter is connected and tested using archived outputs. Remaining ORB requirements are a verified fresh Linux run, physical camera calibration, and metric/arena alignment before navigation; see ORB_INTEGRATION.md. The checklist below records the original requests and remains relevant to the other modules.

The standalone DAG/Kahn scheduler is implemented and runnable. Its October release passed 32 automated tests plus three command-line failure checks; current ORB checks are recorded separately in ORB_TEST_RESULTS.md. It covers the current single-target mission, dependencies, states, timing, time limits, cancellation, controlled retries, bounded replanning, validation, and reports.

The remaining DAG/integration work is to connect the other modules and test the system together. ORB saved-result integration is verified; fresh ORB execution and the physical robot remain unverified. Colleagues can send their existing input/output formats; they do not need to rewrite their algorithms around the scheduler.

## Send this for every module

1. Complete runnable source code or repository, identifying the version handed over.
2. Python/software versions, dependencies, operating system, and execution device: edge/laptop or MentorPi. State any GPU requirement.
3. Required model weights, configuration and calibration files. The whole training dataset is not needed just to connect trained inference.
4. Exact function and arguments, launch command, or service request. Explain how to start a continuous service and request one completed operation.
5. One small sample input that reproduces a working run.
6. One actual successful output, explaining fields, units, coordinates, and what result to expect from the sample.
7. A failure example and the way failure is reported: exception, status, message, or file.
8. Startup/model-loading time, normal runtime, progress/completion signals, and cancellation method where relevant.
9. Known limitations or unfinished features.

Please send working files and saved outputs, not only screenshots or a GUI demonstration.

## Tello Wi-Fi / recording reception

- The Python code that connects directly to Tello Wi-Fi and receives the camera data.
- Whether it produces a saved video, image files, or a frame stream, and where the scheduler obtains a ready recording or finite frame batch.
- One short sample, its image dimensions, recording format, and frame timestamps/frame rate.
- How reception starts/stops, how readiness is signalled, and what happens on connection loss or incomplete recording.

There is no phone-app dependency in this design. Receiving data and choosing when to process it are separate decisions.

## ORB-SLAM3 / geometry

- Runnable code or executable, launch instructions, calibration/settings, and required vocabulary files.
- Accepted input format, timestamps, and preprocessing.
- One successful output containing camera poses and geometric/map data.
- Frame/keyframe IDs linked to the correct images and timestamps.
- Meaning of pose fields, origin, axes, units, and how the output is aligned/scaled into the arena.
- Completion, lost-tracking, initialization-failure, and stop behaviour.

The current DAG sends SLAM-selected keyframes to ERFNet. Tell us if the team instead processes the two algorithms independently, so we can adjust that handoff.

## ERFNet / three-class segmentation

- Inference code, model definition/settings, and trained weights.
- Resizing, normalization, channel order, and other required preprocessing.
- Actual class-to-number mapping for free space, obstacle, and blocked, with examples distinguishing obstacle from blocked.
- One input image and its numerical predicted label mask, including dimensions and file/data format.
- How the mask is linked to the source frame; explain resizing or cropping that changes pixel coordinates.
- Inference time, completion and failure behaviour.

The current DAG expects 0=free space, 1=obstacle, 2=blocked. Other existing model codes can be remapped explicitly. A coloured visualization alone is not an unambiguous numeric label mask. Unknown map data is -1, not a fourth segmentation class.

## Map fusion and planning-map preparation

Assign an owner for this part and send:

- Code that combines geometry and image labels into the ground map, its inputs, and one actual resulting grid.
- Grid dimensions, cell size, origin, axis directions, row/column convention, and conversion to metres.
- Free/obstacle/blocked/unknown definitions. Non-free and unknown regions must remain forbidden to planning.
- How robot size and clearance are included, the chosen setting, and which module performs this step.
- How new obstacle observations update the map and its version.
- How the aerial map and UGV localization share the same physical coordinate system.

The proposed map is 60x60 cells with 0.05 m cells for a 3x3 m arena. If the actual format differs, send it; we will update the contract and adapters together. The DAG does not itself build the real map.

## Custom A* planner

- Code and exact function/command with all parameters.
- Required grid values and whether start/goal coordinates are cells or metres.
- One test map, start, target, and actual returned route.
- Route format, units, point ordering, and whether start and target are included.
- Exact no-route response and invalid-input behaviour.
- Whether the input map must already account for robot size.
- How to call it again with an updated map and current position, plus normal runtime.

A* supplies a route. A movement controller must make the MentorPi follow it.

## MentorPi localization, movement and obstacle feedback

- The code/interface supplying current x/y position, heading, timestamp, and localization validity.
- Its coordinate frame and the conversion to the aerial-map frame.
- How a route/request is sent from the edge to the robot, with example requests and responses. If using ROS, supply actual topic/service/action names and message types; otherwise supply the chosen protocol and message format.
- Progress, confirmed arrival, failure, and lost-connection responses. Command acceptance does not mean arrival.
- An independent stop/cancel request and a response confirming that the movement has stopped.
- Request IDs echoed in responses, and handling that prevents a delayed cancelled command from restarting movement.
- Robot behaviour after communication loss.
- Who detects new obstacles, the event fields/coordinates/timestamp, and how the event reaches mapping and the navigation adapter.

The stop adapter must confirm stopped=true for the same request ID. If it cannot confirm stopping, the scheduler blocks automatic replanning. Terminating the Python worker does not physically stop the robot.

## Custom YOLOv8 inspection

- Inference code, model weights/configuration, class names, dependencies, and thresholds.
- Which camera supplies the images and how the code receives them.
- One input and actual detections: class, confidence, bounding box, frame/timestamp, and any other fields the mission uses.
- The rule for reporting a finding and linking it to the inspection target/robot location.
- The rule for completing an inspection: a time window, number of frames, or another defined condition.
- How it distinguishes no detection from detector failure.
- Whether it also runs continuously during travel. If so, send that service's start/stop and event interface separately.

## Target selection and shared decisions

Agree how the target is chosen: a predefined coordinate, a list, or a selection algorithm. Provide one target ID/coordinate and its frame, or the code that produces it. The current workflow handles one target per mission.

Name owners for map fusion, planning-map preparation, robot localization, target selection, and route following. These connections are needed in addition to the named perception/planning/detection algorithms.

Agree map origin, axes, units, resolution, class codes, timestamp convention, arrival rule, pose freshness, and ownership of map updates. Existing scheduler thresholds are adjustable demonstration settings, not measured hardware values.

## What happens after the files arrive

1. Reproduce each module's supplied example.
2. Write its wrapper in team_adapters.py and translate its inputs/outputs.
3. Register it in real_bindings.json.
4. Test success, failure, and malformed data.
5. Connect modules progressively, then test the complete chain.
6. Verify real arrival, stopping, lost connection and replanning with the UGV.

A useful first handoff is A* plus a test map, start/target, expected route, and no-route example. Modules can arrive one at a time. Integration is finished only after the required real handoffs work together.

Suggested progress statement: "Completed the standalone DAG scheduler using Kahn's algorithm and verified scheduling and failure handling with simulated modules. Awaiting colleagues' modules for adapter integration and system testing."
