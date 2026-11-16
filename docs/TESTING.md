# Testing

```bash
python -m unittest discover -s tests
```
**218 tests, all passing** (Python 3.12, Linux; about 12 minutes: many of them drive real routes in the simulator).

> **Scope.** Everything below ran in software: automated tests, the built-in simulator and a headless
> browser. On the real MentorPi only the communication link has been verified. **Autonomous driving and
> YOLO on the real robot are not yet verified.**

## What the tests cover
| File | Verifies |
|---|---|
| `test_system`, `test_clearance`, `test_gui` | The A\* planner: optimality, no corner cutting, inflation, clearance preference, ground-truth evaluation, original planner window |
| `test_fast_planner` | Fast planner returns exactly the original planner's result on random queries and maps |
| `test_smoothing` | Smoothed routes keep their ends, stay collision-free and never cost more |
| `test_nav_protocol_bridge` | Signed framing, tampering, replay, wrong key; bridge starts disarmed; clamping; watchdog stops within 0.45 s; stale and malformed commands rejected |
| `test_nav_core` | Frame transforms, odometry → map, drift states, controller, safety zones |
| `test_nav_integration` | Full loop over TCP with a simulated robot: no motion without authorization, reaches the goal with odometry error, stops for an unexpected box, replans, auto-continue rules |
| `test_scan_match` | A LiDAR that sees the arena walls: pose recovered from 8 cm / 6° error; ambiguous corridor refused; long drive completed with LiDAR correction |
| `test_room_tracking` | A LiDAR that looks over the arena (simulated room, the arena walls are invisible to it): room drawing, people fade out, save / load; the robot is found anywhere without a starting guess, a room that looks the same both ways is not guessed, in a half-drawn room a wrong place is never accepted; the LiDAR mounting (centred, off-centre, backwards) is learnt from the robot's motion and nothing is corrected before it is verified; closed loop: 8.5 m of driving with 8 % odometry error stays within 7 cm, without tracking the drift limit stops the robot, a carried robot is found again, a new session needs no pin |
| `test_field_geometry` | Wall positions read from the maps (inner field 2.76 m), corner start poses, one block length from the walls is a valid A\* start, stretching the map to the real size, thin walls survive, saved edits follow the size, data files never change |
| `test_nav_vision` | Floor projection, calibration, tracker, camera stream, tilt from block size; a door, a round object and a wrong-coloured box are rejected; one block gives one pin |
| `test_nav_mission` | First green is approached, held 3 s, secured, mission ends; red is ignored; STOP and localization loss pause; automatic actions never clear STOP |
| `test_console_app` | Console commands, settings, map editing, orientation of ENV2 / ENV3, no preset goal, two-step authorization; corner start, Find robot, nudge, room remembered for the next session, real field size typed or measured with the robot in two corners |
| `test_console_server` | Local-only server, token / host / origin checks, static files, robot stopped when the page disappears |

## Browser end-to-end (headless Chromium)
Simulator run through the real page: place the robot, connect, confirm, camera, blocks, goal,
authorize + search → survivor found, approached, secured, mission complete, no page errors.
Rendering: 16.7 ms per frame (median) while the robot drives.

Second run for the position features: corner start (click, click again to turn), room tracking locks,
two drives, Find robot, nudge, real field size applied and reset, and a second session in which the robot
is found by itself without a pin.

## What the room-tracking numbers mean
All from the simulated room (4.5 × 4.1 m with furniture, LiDAR range 3 m, 1 cm range noise, at most 180 points):

| Measurement | Result |
|---|---|
| Robot placed at 220 random spots, room fully drawn | 220 found, all within 5 cm, none wrong |
| Same in a half-drawn room, automatic placement | no wrong place accepted; doubtful ones go to the operator |
| Rays passing through known walls | at most 3 % at the true place, 31-48 % at places that only look similar |
| LiDAR mounted 4.5 cm ahead, 1 cm left, turned 3° | learnt as 4.5 cm, 1.2 cm, 3.2° |
| Position error while driving, odometry 8 % off | 2-3 cm with tracking (odometry alone: 15 cm after 2 m) |

A real room has moving people, doors and clutter that the simulation only partly has. Expect to adjust.

## Closed-loop simulation (`python nav_main.py sim-demo`)
Plans on the occupancy map while the simulated robot moves in the ground-truth map, with 2 % odometry error.

| Run | Result | Final error | Re-localizations |
|---|---|---|---|
| ENV1, odometry only | stopped by the drift limit after about 2.6 m | - | 0 |
| ENV1 | arrived | 7.1 cm | 1 |
| ENV2 | arrived | 6.0 cm | 4 |
| ENV3 | arrived | 5.1 cm | 3 |

No contact with ground-truth obstacles in any run.

## Real MentorPi
| Stage | Status |
|---|---|
| Communication: authenticated link, 1-7 ms round trip, real odometry / IMU / LiDAR received, robot stays disarmed | ✅ passed |
| Wheels off the ground, floor moves, waypoint, arena route, physical safety tests | ⏳ not yet completed |

## YOLO model (from `third_party/wro_yolo/metrics.json`)
YOLOv5n, 2 classes. Held-out real photos (9 images, 11 objects), ONNX on CPU: precision 0.99,
recall 1.00, mAP50 0.995, about 31 ms per image. Small test set; not yet measured on the robot's camera.

## Still to verify on hardware
1. That commanded velocities move the robot as expected (signs, units, stopping distance).
2. Odometry accuracy and the LiDAR mounting pose.
3. Map ↔ arena registration (origin corner, axis directions, 3 cm scale).
4. YOLO accuracy and block position accuracy from the robot's camera.
