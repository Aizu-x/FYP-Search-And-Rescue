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
* 100 × 100 cells of 3 cm = 3 × 3 m. Origin is the lower-left corner, x to the right, y up.
* `data/ENVn_occupancy.npy` is the map the robot plans on (from the ERFNet segmentation).
  `data/ENVn_GT.npy` is ground truth, used only to evaluate routes and to drive the simulator.
* ENV2 and ENV3 are turned in memory to match the real arenas (`ugv_nav/map_store.py`). Files are never rewritten.

## 3. A\* planner
`ugv_planner/astar.py`: 8-connected grid, octile heuristic, no corner cutting. Obstacles are inflated by the
robot radius + margin, and a clearance weight makes routes prefer open space.

| Addition (`ugv_nav/`) | What it does |
|---|---|
| `fast_planner.py` | The same search on flat arrays with cached inflation and distance fields. Results are identical to `ugv_planner` (tested field by field) at about 50× the speed. Every route is verified with the exact collision checker. |
| `smoothing.py` | Replaces zig-zags by straight segments only when the robot's swept disk stays clear **and** the planner's own cost does not get worse. Start and goal never move. |

## 4. Localization
* The operator places the robot at a known pose and confirms it. From then on odometry is transformed
  into the map frame.
* A drift estimate grows with distance and turning. Above 15 cm the robot stops.
* `scan_match.py` matches the LiDAR scan against the map walls to correct the pose, on request
  (**Re-localize**) or continuously (beta). Ambiguous matches such as a featureless corridor are refused.

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

## 11. Known limits
* Wheel odometry alone drifts past the 15 cm limit after a few metres: long routes need LiDAR re-localization.
* The map's registration to the real arena (origin, scale) must be checked with a tape measure.
* A small object placed against a mapped wall is only caught by the bridge's short stop zone.
* The YOLO model was trained on phone photos; its accuracy from the robot's low camera is not yet measured.
