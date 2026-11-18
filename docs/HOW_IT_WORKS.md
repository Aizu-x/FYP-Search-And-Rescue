# How it works

## 1. Architecture
```
EDGE LAPTOP (all planning and decisions)                MENTORPI-M1 (executor)
┌──────────────────────────────────────────┐          ┌─────────────────────────────────┐
│ ugv_console   web console                │          │ ugv_bridge                      │
│ ugv_planner   A* on the occupancy grid   │  Wi-Fi   │  ├ authentication, sequence, expiry
│ ugv_nav       localization (odom → map)  │◀──TCP──▶ │  ├ starts DISARMED              │
│               path following             │  HMAC    │  ├ speed limits, LiDAR stop zone│
│               safety monitor             │          │  ├ watchdog = only motor writer │
│               YOLO → map pins            │          │  └ ROS 2: cmd_vel, odom, imu,   │
│               search mission             │          │     scan, camera                │
└──────────────────────────────────────────┘          └─────────────────────────────────┘
```
The laptop needs no ROS. The robot needs only Python and the `rclpy` that HiWonder already ships.

## 2. Map and coordinates
* The data files are 100 × 100 cells. At 3 cm per cell that is 3 × 3 m, but the floor between the walls is
  only about 2.76 × 2.76 m in them. Origin is the lower-left corner, x to the right, y up.
* `data/ENVn_occupancy.npy` is the map the robot plans on (from the ERFNet segmentation).
  `data/ENVn_GT.npy` is ground truth, used only to evaluate routes and to drive the simulator.
* ENV2 and ENV3 are turned in memory to match the real arenas (`ugv_nav/map_store.py`). Files are never rewritten.
* **Real field size:** when the operator gives the real inner size, the map is stretched to it and cut into
  3 cm cells again (a cell becomes wall when at least 40 % of it was wall, so thin walls neither vanish nor
  double). Occupancy, ground truth and saved edits are stretched alike; every coordinate is then a real metre.
* **Walls from the map:** `inner_box` (median wall faces) and `corner_pose` give the pose of a robot that
  stands square in a corner at a known distance from both walls. That is the corner start.

## 3. A\* planner
`ugv_planner/astar.py`: 8-connected grid, octile heuristic, no corner cutting. Obstacles are inflated by the
robot radius + margin, and a clearance weight makes routes prefer open space.

| Addition (`ugv_nav/`) | What it does |
|---|---|
| `fast_planner.py` | The same search on flat arrays with cached inflation and distance fields. Results are identical to `ugv_planner` (tested field by field) at about 50× the speed. Every route is verified with the exact collision checker. |
| `smoothing.py` | Replaces zig-zags by straight segments only when the robot's swept disk stays clear **and** the planner's own cost does not get worse. Start and goal never move. |

## 4. Localization
The pose is the confirmed start pose plus the odometry since then. A drift estimate grows with distance and
turning; above 15 cm the robot stops. What keeps the estimate true depends on what the LiDAR can see.

### 4a. The LiDAR looks over the arena (MentorPi, default): `ugv_nav/room_map.py`
The arena walls and blocks are lower than the LiDAR, so the scan shows the room around the arena.

* `RoomMap`: a grid (3 cm, arena ± 4.5 m) of what the LiDAR has seen. Every scan adds evidence: cells the rays
  crossed are free, the ends are walls. A wall needs no second look; somebody walking by fades out again.
* `RoomTracker`: about twice a second the scan is matched to that drawing around the odometry prediction
  (± 9 cm, ± 6°; three refinement steps down to 3 mm). The result is the robot's true movement.
  Standing still with a view that no longer fits means the robot was moved by hand: a wider window, then a
  search of the whole arena.
* **Observe first.** Where the LiDAR sits on the robot is not known in advance, and a wrong assumption would
  bend every correction. The tracker therefore first only watches and learns the mounting from the motion:
  on straight pieces the tracked position must move along the heading (a sideways share is the yaw of the
  sensor); in turns with little driving a sensor that is off-centre travels on a circle (its radius and phase
  are the offset). After a measurement the room is drawn again and a second round must confirm it. Only then
  are corrections applied. A LiDAR mounted backwards or sideways is found the same way and reported, because
  the robot's own stop zone then looks the wrong way.
* **Find robot** (`ScanMatcher.locate`): every free spot of the arena on a 6 cm lattice × every 3° heading is
  scored with a blurred map; the best ten are refined. Places are dropped when the robot cannot stand there or
  when more than 12 % of the rays would pass *through* known walls (light does not do that; this is what
  separates the true place from one that merely looks similar). The robot is placed without asking only when
  the fit is at least 80 %, at least 60 % of the view was seen before, and no other place comes within 8 %.
  Otherwise the candidates are shown and the operator chooses.
* The drawing is saved per environment (`data/rooms/`) when it started from a corner start and the mounting is
  verified. The next session loads it and finds the robot without a pin.

### 4b. The LiDAR sees the arena walls (`lidar_localization.sees = "walls"`)
`scan_match.py` matches the scan against the arena map itself, on request (**Re-localize**) or about once a
second. Ambiguous matches such as a featureless corridor are refused.

## 5. Path following
`ugv_nav/controller.py`: lookahead tracker with PD heading control. The robot drives and steers together and
turns on the spot only above 35° heading error. It stops on more than 10 cm cross-track error, on no
progress, or when the LiDAR view does not change although odometry says the robot is moving.

## 6. Obstacles and replanning
* A LiDAR return that is not on the map and lies in the speed-dependent stop zone stops the robot.
* The obstacle is written into an in-memory copy of the map and A\* replans from the current pose.
* The new route needs a fresh authorization, unless **Auto-continue after replan** is on (limited to
  5 automatic replans per authorization).

## 7. Vision (`ugv_nav/vision.py`)
1. YOLOv5n (ONNX, CPU, about 30 ms per frame) returns boxes for `green_obstacle` and `red_obstacle`.
2. Boxes that cannot be a 5 × 5 × 10 cm block are discarded: wrong shape, wrong size for their floor
   distance, or not containing their colour.
3. The bottom edge of a box is projected onto the floor using the camera height, tilt and lens model.
   The tilt is measured automatically from the known block height.
4. A tracker confirms an object after 4 sightings, averages its position (close sightings weigh more),
   votes on its colour, and joins duplicate pins.
5. Confirmed blocks become obstacles for A\*; green ones become survivors for the mission.

## 8. Search mission (`ugv_nav/mission.py`)
```
TRANSIT ─▶ SCAN ─▶ (green seen) ─▶ APPROACH ─▶ FACE + HOLD ─▶ SECURED ─▶ next green, or COMPLETE
             └─▶ (nothing) ─▶ EXPLORE the floor the camera has not seen ─▶ SCAN ...
```
A green object confirmed at any time interrupts the current drive. Every motion goes through the same
navigator checks as a normal drive. Operator STOP, link loss and localization loss pause the mission;
only the operator can resume it.

### Mission video (`ugv_nav/recorder.py`)
When a search mission starts, the console starts a `MissionRecorder`. Its only job in the main program is a light
thread that, ten times a second, takes the newest camera frame the vision worker holds (the JPEG that YOLO looked at,
with its boxes) and the mission's state, and sends them with the time to a **separate worker process**
(`python -m ugv_nav.recorder --worker`, lower priority). The worker decodes, draws the overlay and writes the MP4
(`mp4v`, or Motion-JPEG AVI if that is not available).

Why a separate process: doing the video work inside the program that drives the robot made it stutter. In a test with
simulated missions the odometry went stale for up to 0.97 s (the limit is 0.5 s) and missions paused. With the worker
process the odometry age stays at 0.2-0.25 s, the same as without recording.

The video is built from the times the pictures carry, on a fixed clock of 10 frames a second, instead of "one frame per
camera frame". That keeps it exactly as long as the mission: a late picture means the previous one is repeated (and the
frame says how old it is), an early one is skipped. The console's ticker saves the video 2 s after the mission has ended
(complete or stopped); a disconnect saves it at once; if the console itself disappears the worker still finishes and
saves. The file is called `..._recording.mp4` until it is complete.

## 9. Link protocol (`ugv_bridge/protocol.py`)
* TCP, length-prefixed JSON frames, **HMAC-SHA256 on every frame**, mutual challenge-response with a
  32-byte shared key, strictly increasing sequence numbers.
* Every velocity command carries an expiry time in the robot's clock, so a command delayed by Wi-Fi
  is rejected instead of executed late.
* Messages are authenticated, not encrypted.

| On the robot | Bridge reaction |
|---|---|
| Start, new connection | DISARMED until the laptop arms it |
| No fresh command for 0.3 s | zero velocity |
| No valid command for 1 s | disarm |
| Bad signature, replay, malformed or out-of-range command | rejected, robot stopped and disarmed |
| Odometry or LiDAR stale | zero velocity, arming refused |
| LiDAR point in the stop zone | zero velocity |

## 10. Console (`ugv_console/`)
* `app.py`: every action is a validated command. `server.py`: standard-library HTTP server on
  127.0.0.1 only, with a per-run token and host / origin checks. `web/`: the page (plain HTML, CSS, JavaScript).
* State reaches the page as a 20 Hz event stream; the page renders at display rate and interpolates the pose.
* Authorization is two requests (question with a one-time code bound to the route, then confirmation).
* The open page is the dead-man switch: no page for 1.5 s while the robot moves means STOP.

## 11. UAV section (`uav/`, `ugv_console/uav_console.py`)
The drone is a Tello, controlled with the text commands of the official Tello SDK 1.3 over UDP. Nothing of this
runs on the robot, and the drone and the robot never talk to each other: the operator scouts with the drone,
then works with the robot.

```
page: which keys are held (on change, and every 150 ms while one is down)
  -> console: keys -> four stick values (W S forward/back, A D left/right, arrows up/down and turn) x strength
  -> uav/tello.py: "rc a b c d" 20 times a second while flying; zeros when no key is held
Tello -> port 8890: status text (battery, height ...)  -> readings on the page
Tello -> port 11111: H.264 video -> uav/video.py (own process): decode -> preview JPEGs, MP4 footage, photos
```

* **States**: disconnected, connecting, ready, taking off, flying, landing. Take-off is two requests (question
  with a one-time code, then confirmation), like the robot's AUTHORIZE. "land" is accepted in every state, sent at
  once and repeated every 2 s until the drone answers "ok" (or its reported height stays 0).
* **Hover by default**: stick values exist only while the page keeps reporting keys. No report for 0.5 s:
  zeros. Page gone for 5 s: land. Nothing heard from the drone for 2.5 s: zeros; for 5 s: land. If the console
  process itself dies during a flight, the video worker (which notices that its input closed) sends "land".
* **Answers without numbers**: the drone's answers carry no reference to the command. The link therefore keeps
  at most one question open, ignores answers that arrive right after "land" (they belong to something sent
  before it) and does not take the late "ok" of a take-off for a landing.
* **Status blocked** (a firewall, or the port in use): battery and height are asked for once a second instead
  ("battery?", "height?"), and the page says what to fix.
* **Video**: the worker receives the datagrams itself (only from the drone's address), passes them to OpenCV's
  FFmpeg decoder over a local port with low-delay settings and one decoder thread (measured with the simulated
  camera: about one picture of delay instead of one second), and writes the footage with the same file writer as
  the mission video (real-time length, MP4 or AVI, `_recording` until complete, disk guards).
* **Simulated drone** (`uav/sim.py`): same protocol on 127.0.0.1, status ten times a second, the 15 s safety
  landing, and a camera that draws a room and sends it as real H.264 (uncompressed macroblocks, so no encoder
  is needed). The link and the video worker cannot tell it from the real drone; how it flies is made up.

## 12. Known limits
* Wheel odometry alone drifts past the 15 cm limit after a few metres: long routes need live LiDAR tracking.
* Room tracking needs a room: enough walls or furniture within the LiDAR's reported range (3 m by default,
  see Robot setup), and a room that mostly stays as it is. Tested in a simulated room only.
* The LiDAR does not see the arena walls or the blocks. Walls are avoided by knowing the position, blocks by the camera.
* The map's registration to the real arena (origin, scale) must be checked: corner start and real field size.
* A small object placed against a mapped wall is only caught by the bridge's short stop zone.
* The YOLO model was trained on phone photos; its accuracy from the robot's low camera is not yet measured.
* The UAV section has only been run with the simulated drone. With a real Tello still open: which sign of each
  stick value is which direction (the SDK document does not say; the common convention is used), what the drone
  does with its last stick values when the Wi-Fi drops, and how its real video stream behaves in the decoder.
* The drone has its own Wi-Fi network. With one Wi-Fi adapter the laptop is either on the drone's network or on
  the robot's, not both.
