# Testing

```bash
python -m unittest discover -s tests
```
**339 tests** (about 18 minutes: many of them drive real routes in the simulator). Last full run, Windows with Python 3.14: all 337 pass; the 2 window tests of the original Tk planner (`test_gui`) were not part of that run because they open windows (their code is unchanged, they passed in the earlier Linux runs).

> **Scope.** Everything below ran in software: automated tests, the built-in simulator and a headless
> browser. On the real MentorPi only the communication link has been verified. **Autonomous driving and
> YOLO on the real robot are not yet verified.** The UAV section is tested with the simulated drone. Manual
> control has also been flown once with a real Tello; **planned flight has not been flown with a real Tello yet.**

## What the tests cover
| File | Verifies |
|---|---|
| `test_system`, `test_clearance`, `test_gui` | The A\* planner: optimality, no corner cutting, inflation, clearance preference, ground-truth evaluation, original planner window |
| `test_fast_planner` | Fast planner returns exactly the original planner's result on random queries and maps |
| `test_smoothing` | Smoothed routes keep their ends, stay collision-free and never cost more |
| `test_nav_protocol_bridge` | Signed framing, tampering, replay, wrong key; bridge starts disarmed; clamping; watchdog stops within 0.45 s; stale and malformed commands rejected |
| `test_nav_core` | Frame transforms, odometry → map, drift states, controller, safety zones; the drift alert can be ignored while stale / jumping odometry and a missing pose still stop the robot |
| `test_nav_integration` | Full loop over TCP with a simulated robot: no motion without authorization, reaches the goal with odometry error, stops for an unexpected box, replans, auto-continue rules |
| `test_scan_match` | A LiDAR that sees the arena walls: pose recovered from 8 cm / 6° error; ambiguous corridor refused; long drive completed with LiDAR correction |
| `test_room_tracking` | A LiDAR that looks over the arena (simulated room, the arena walls are invisible to it): room drawing, people fade out, save / load; the robot is found anywhere without a starting guess, a room that looks the same both ways is not guessed, in a half-drawn room a wrong place is never accepted; the LiDAR mounting (centred, off-centre, backwards) is learnt from the robot's motion and nothing is corrected before it is verified; closed loop: 8.5 m of driving with 8 % odometry error stays within 7 cm, without tracking the drift limit stops the robot, a carried robot is found again, a new session needs no pin |
| `test_field_geometry` | Wall positions read from the maps (inner field 2.76 m), corner start poses, one block length from the walls is a valid A\* start, stretching the map to the real size, thin walls survive, saved edits follow the size, data files never change |
| `test_nav_vision` | Floor projection, calibration, tracker, camera stream, tilt from block size; a door, a round object and a wrong-coloured box are rejected; one block gives one pin |
| `test_nav_mission` | First green is approached, held 3 s, secured, mission ends; red is ignored; STOP and localization loss pause; automatic actions never clear STOP |
| `test_recorder` | The video file, driven by explicit times (fast, deterministic): as long as the mission whatever the camera rate (8, 3 and 25 pictures a second), starts with the first picture, tail after the end, no file when no picture came, undecodable picture ignored, MP4 → AVI fallback, disk and length guards, clock jump, overlay text / strips / events / "frame is N s old" / wall clock, long text never runs into the clock, boxes in the live-view colours, plain footage. The worker process (real process, real time): length of the video, progress, tail, no pictures, a broken source, a worker that is killed is reported and nothing hangs, the video is saved when the console goes away, lower priority, start refused (disk, OpenCV, folder, no worker) |
| `test_console_app` | Console commands, settings, map editing, orientation of ENV2 / ENV3, no preset goal, two-step authorization; corner start, Find robot, nudge, room remembered for the next session, real field size typed or measured with the robot in two corners; *Ignore drift alerts* asks first, applies live, is remembered and never changes the loaded config; *Mission video*: a simulated mission is filmed and the file plays back (640 × 480, length = mission + tail), Stop mission saves it, STOP does not end it, Disconnect saves it at once, the switches and the folder setting (a file is refused) work |
| `test_console_server` | Local-only server, token / host / origin checks, static files, robot stopped when the page disappears |
| `test_uav_link` | The link to the drone against the simulated drone: status text and answers are read (also malformed ones), keys → stick values, only local addresses; "command" first, nothing sent before take-off, each key moves the drone the expected way, released keys and a silent page mean hover; "land" is repeated until it gets through, a landing is noticed when the answer is lost, LAND during take-off, answers to stick commands do not end a landing early; no take-off with a low or unknown battery, a refused or unanswered take-off; emergency; blocked status port → battery and height are asked for; silence → zeros, then land; forged datagrams are dropped; closing the link lands; the simulated drone's own 15 s safety landing |
| `test_uav_plan` | Flight plans: every kind of step is checked (ranges, whole numbers, "Land" last, at most 200 steps), steps in words, long distances in pieces of at most 1 m and 4 s, time estimate, plan names, saved plans (save / load / delete, damaged or foreign files refused). Flying with the simulated drone: the example *up 20 cm, turn 360°, forward 40 cm, land* sends exactly `takeoff, speed 30, up 20, cw 360, forward 40, land` and ends 40 cm ahead, facing as before, on the ground; no stick values between the moves; a plan started in the air ends with a hover and the keys work again; speed steps; **a flight key takes over**, STOP PLAN, LAND in the middle of a move (also when the drone refuses "land" until the move is done), a step the drone refuses, a stick complaint tried once more, no answer, low battery → landing, refused take-off, silent drone, blocked status port. A flight flown by hand is remembered (stick values and times, a photo, a silent page as a hover) and flown again to within 25 cm and 15° in the simulator |
| `test_uav_plan_console` | Planned flight through the console: flight mode setting, the plan on the page (checked on every change, a refused change changes nothing), saved plans with their confirmations, the plan survives a restart; with the simulated drone process: **two-step start that shows the whole plan** (a wrong code or a changed plan cancels it), no editing and no mode change while a plan flies, progress step by step, filmed automatically, the plan's photo; STOP PLAN, key take-over, a plan started from a hover, Esc lands; no plan while the robot moves or outside the UAV section; a manual flight becomes "Last manual flight" and is flown again with its photo; closing the console in mid-plan lands; **the plan stops when the page disappears**, then the drone is brought down |
| `test_uav_video` | The simulated camera's H.264 is read by a normal decoder; the drawn view follows the drone. The real worker process: preview pictures arrive, the picture comes back after the video stopped, footage is as long as the recording and plays back in full size, photo, nothing saved without pictures, unusable folder refused, video from another address dropped, footage saved and **the drone landed when the console dies**, a dead worker is reported, port in use explained |
| `test_uav_console` | Sections (UAV / UGV), remembered settings, settings checks; a whole flight through the console's commands with the simulated drone process: two-step take-off (a wrong code uses up the question), keys, stale key reports ignored, STOP lands, footage and photo in the chosen folder; one machine at a time (no take-off while the robot moves, no robot authorization and no leaving while the drone flies); emergency asks first; recording by hand and switched off; closing the console lands the drone and keeps the footage; the page gets the picture over `/api/uav_frame`; the drone lands when the page stays away |

## Browser end-to-end (headless Chromium)
Simulator run through the real page: place the robot, connect, confirm, camera, blocks, goal,
authorize + search → survivor found, approached, secured, mission complete, no page errors.
Rendering: 16.7 ms per frame (median) while the robot drives.

Second run for the position features: corner start (click, click again to turn), room tracking locks,
two drives, Find robot, nudge, real field size applied and reset, and a second session in which the robot
is found by itself without a pin.

Fourth run, planned flight with the simulated drone: the plan *Up 20, Turn right 360, Forward 40, Land* is built with
the page's own controls (a distance of 5 cm is refused with a message), a value is changed and a step moved in the
list, the plan is saved, RUN PLAN shows the question with every step, the list marks the step being flown, the
drone lands and the flight is filmed; a flight flown with real key presses is copied with **My last flight** and
flown again; a key pressed during a plan takes over; a plan is started from a hover and ended with STOP PLAN.
No page errors.

Third run for the UAV section, with the simulated drone: real clicks and real key presses. UAV button,
connect, live picture (640 × 480, 14 pictures a second), take-off question, W / S / A / D and the four arrow keys
each give the expected stick values, keys released when the window loses focus, no flying while typing in a text
field, the on-screen keys, leaving refused in flight, Esc lands, footage saved, back to the Planner, no page errors.

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

## Real Tello
| Stage | Status |
|---|---|
| Everything in the UAV section with the simulated drone (same UDP protocol, real H.264 video) | ✅ passed |
| Manual control with a real Tello: connect, status, live video, take-off, keys, landing confirmed by the drone, footage file | ✅ flown once by the project author |
| Planned flight with a real Tello (movement commands, copied flights) | ⏳ not yet done |

## YOLO model (from `third_party/wro_yolo/metrics.json`)
YOLOv5n, 2 classes. Held-out real photos (9 images, 11 objects), ONNX on CPU: precision 0.99,
recall 1.00, mAP50 0.995, about 31 ms per image. Small test set; not yet measured on the robot's camera.

## Still to verify on hardware
1. That commanded velocities move the robot as expected (signs, units, stopping distance).
2. Odometry accuracy and the LiDAR mounting pose.
3. Map ↔ arena registration (origin corner, axis directions, 3 cm scale).
4. YOLO accuracy and block position accuracy from the robot's camera.
5. Tello: planned flight with the real drone, its behaviour when the Wi-Fi drops, whether it lands in the middle
   of a movement command (see *Before the first real planned flight* in the [UAV guide](UAV.md)).
