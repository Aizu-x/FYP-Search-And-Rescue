# Test report: edge A* navigation for the MentorPi-M1

## Version 2 (web console, planner, camera and LiDAR improvements) - 2026-11-16

**Environment:** cloud Linux container, Python 3.12, NumPy, Matplotlib, OpenCV, ONNX Runtime; the page was
driven in headless Chromium. **No MentorPi was reachable. Nothing in version 2 has been tested on the real
robot.** The robot-side bridge is a byte-identical copy, so the earlier Stage B result below still describes it.

| What | Result |
|---|---|
| Full automated suite (`python -m unittest discover -s tests`) | **186 tests, all pass** (original 31 planner/GUI tests included) |
| `test_fast_planner.py` (7) | fast planner returns field-by-field the same result as `ugv_planner` on random queries (3 maps x 8 settings), invalid inputs, pose connector, random maps; cache follows map content; faster than the original |
| `test_smoothing.py` (6) | ends never moved, every segment passes the exact swept-disk check, planner objective / length / total turning never worse; fewer sharp corners; off = exactly the A* route |
| `test_nav_vision.py` (25) | plus: tilt from one box, estimator confidence, distance weighting, class voting, shape filter, frame hand-over, wrong tilt found and corrected in closed loop |
| `test_scan_match.py` (7) | pose recovered from 8 cm / 6 deg error; corridor refused as ambiguous; operator re-localization; long drive stopped by the drift limit without correction and completed with it |
| `test_console_app.py` (20) | planner mode = original route; playback + export files; settings validation; map paint/save/reload/revert; settings remembered and old settings imported; simulator workflow with two-step authorization, STOP; paint while connected; LiDAR layer; re-localize; robot mode against a stand-in bridge (right key links and never arms, wrong key does not link); camera pins, auto tilt, manual calibration; search mission |
| `test_console_server.py` (6) | 127.0.0.1 only; token / host / origin checks; malformed and oversized requests; static files only from `web/`, nothing external; event stream; **robot stopped when the console page disappears** |
| `test_import_old.py` (3) | values taken from an old folder, old folder unchanged, existing files kept unless forced |
| Browser end-to-end (Chromium, not part of the unittest suite) | simulator: place robot by drag, connect, confirm, camera, blocks, goal, authorize + search: both survivors found and reached, no console errors; planner playback; painting with the mouse; rotated view keeps map coordinates; measure; light theme; drift stop then LiDAR re-localization; page closed while driving: robot stopped |
| Smoothness in the browser | 16.7 ms per frame (median) while the simulated robot drove; position steps of about 3-5 mm per frame |

Not tested: the real robot; the real camera/YOLO through the new console (the ONNX model was run on the three
example photos only); opening the app window on Windows.

---

# Earlier report (version 1, kept for reference)

**Date:** 2026-11-02 · **Environment:** cloud Linux container, Python 3.12/3.13, NumPy + Matplotlib,
Tk under Xvfb. **No MentorPi hardware was reachable from this environment.**

## Summary

| Stage | Status |
|---|---|
| A: software (planner, transforms, controller, safety logic) | **PASSED** (automated) |
| B: communication | **PASSED on the real MentorPi** (bridge in container `MentorPi`, ros2 backend): authenticated link, RTT 1.0–7.1 ms, telemetry ≤ 105 ms old, armed False, real /odom, /imu and /scan_raw (176 points) received, no rejected frames. |
| Closed-loop simulation (real bridge + protocol + watchdog, simulated robot) | **PASSED**, with caveats below |
| C: wheels off ground | **NOT RUN** (needs operator approval and hardware) |
| D: manual floor motion | **NOT RUN** |
| E: single waypoint | **NOT RUN** |
| F: multiple waypoints | **NOT RUN** |
| G: full arena | **NOT RUN** |
| H: physical safety tests | **NOT RUN** (the software equivalents passed, see below) |

**Physical navigation has not been tested and must not be considered working.**

## 1. Original project (unchanged)

All 31 original tests pass (`test_system`, `test_clearance`, `test_gui`), including the Tk GUI tests
under Xvfb. One caveat: `ugv_planner/batch.py` failed to extract from the uploaded RAR (decoder error
in both `unar` and 7-Zip). A clearly labelled stand-in was used **only in the test copy**, so that
`test_gui`'s batch button could run. It is not part of the deliverable, and your own `batch.py` is authoritative.

## 2. New automated tests

| File | What it verifies | Result |
|---|---|---|
| `test_nav_protocol_bridge.py` (24) | framing, MAC tamper, NaN/non-object rejection, oversize length, replay/order, handshake with wrong key / replayed auth, key parsing; bridge starts disarmed; wrong key refused; clamp + watchdog zero ≤ 0.45 s then disarm; STOP disarms; stale / far-future / malformed / bool / unknown commands rejected; > 1.5× limit disarms; lateral `vy` rejected; bad MAC closes and disarms; disconnect disarms and **reconnect stays disarmed**; heartbeat loss drops client; second client rejected; stale odometry blocks arm and motion; LiDAR stop ahead while reverse still allowed; fake-clock slew/hold/starvation | PASS |
| `test_nav_core.py` (16) | SE(2) identities; planner frame convention (0.015 m cell centres, (0.345, 0.315) → cell (10, 11)); odom→map calibration; STALE/UNCALIBRATED/LOW_CONFIDENCE/LOST transitions; IMU heading; collinear simplification keeps A* length exactly; shortcut stays collision-free on ENV1–3; blocked path rejected; controller arrives on an L-path starting backwards, stays within the cross-track limit, rotates in place first, tolerates 20% odometry scale error, detects no-progress and cross-track faults; edge LiDAR filter (mapped wall ignored, unmapped box blocks); stopping distance; LiDAR→map obstacle insertion without touching the source grid | PASS |
| `test_nav_integration.py` (10) | full loop over TCP with the simulated robot on the ENV1 GT world, planning on the ENV1 occupancy map: **no motion without authorization**; reaches goal with 3% odometry scale error + yaw drift, true trajectory never touches GT; unexpected box → stop before the box, edge map update + edge replan, stays disarmed; Wi-Fi loss → UGV output zero ≤ 1 s, client reconnects, **robot does not resume**; edge crash → bridge watchdog zero < 0.5 s then disarm; latched STOP needs replan + re-authorization; odometry jump → stop; invalid start/goal refused; unknown radius refused; planning radius smaller than turning envelope refused | PASS |
| `test_nav_integration.py`: auto-continue (+4) | auto-continue drives around an unmapped box to the goal without operator input (no collision, ≥1 automatic replan); limit 0 → waits for operator; STOP during the pending delay cancels it; box in a corridor too narrow to pass → "no alternative route", robot stays stopped. The manual-replan test now requires a successful new route (PLANNED) instead of also accepting "no route". | PASS (auto tests repeated 3×) |
| `test_nav_gui.py` (3) (version 1 only; replaced by `test_console_app.py`) | operator console through real Tk callbacks: offline mode unchanged; closed-loop connect → confirm pose → goal click plans (robot still disarmed) → decline/accept authorization → motion → **Esc STOP** zeroes output; physical mode requires host | PASS |

Full-suite results (original + new) are in section 5.

## 3. CLI / deployment checks (loopback)

* `run_bridge` refuses to start with the example config (`half_length_m` is null: must be measured).
* `run_bridge` refuses a group/world-readable key file.
* `run_bridge --backend dryrun` + `nav_main.py check-connection`: authenticated link, RTT 0.2–1.1 ms,
  telemetry age ≤ 91 ms, `armed: False`, odom/IMU/180 LiDAR points received.
* `check-connection` with the wrong key: `AuthError: MAC verification failed`, exit code 1.
* All Pi-side files parse as Python 3.8. Lint (pyflakes) is clean.

## 4. Closed-loop simulation demos (`nav_main.py sim-demo`)

Plans on the ERFNet occupancy map; the simulated robot moves in the GT map. Body 0.16 × 0.14 m,
planning radius 0.106 m + 0.02 m margin, 0.12 m/s, 2% odometry scale error.

| Run | Result | True error to goal | Re-localizations | GT contact |
|---|---|---|---|---|
| ENV1, odometry only | **STOPPED** by drift guard after ≈2.6 m | 1.71 m (did not finish) | 0 | none |
| ENV1, with re-localization | ARRIVED, 39 s | 0.071 m | 1 | none |
| ENV2, with re-localization | ARRIVED, 84 s | 0.060 m | 4 | none |
| ENV3, with re-localization | ARRIVED, 68 s | 0.051 m | 3 | none |

"Re-localization" emulates the operator re-placing and confirming the robot's pose when the drift
guard stops it. Figures: `nav_outputs/*/ENV*_closed_loop.png`.

**Finding 1:** with a conservative 5 cm/m drift budget and 15 cm limit, wheel odometry alone cannot
complete any full arena route. Physical full-arena runs need manual re-localization stops, or a
LiDAR-to-map localizer (recommended next step).

**Finding 2 (fixed during this work):** a fixed 25 cm forward LiDAR stop zone caused false stops when
the A* route turned near mapped walls or ended near a wall. The fix:
* both layers now use a speed-dependent zone;
* the edge ignores returns from mapped walls (the bridge still sees all returns);
* authorization now requires planning radius ≥ turning envelope.

## 5. Full suite

`python -m unittest discover -s tests -v` under Xvfb, run twice back to back:
**84 tests, 84 passed, 0 skipped** each time (31 original + 53 new, ≈64 s per run).
After adding auto-continue: **88 tests, 88 passed** (31 original + 57 new, ≈87 s).
After adding map editing, settings memory, scale calibration and the speed slider: **95 tests, 95 passed** (≈118 s).
The closed-loop calibration test starts with odometry that reads 10 % too far: the robot stops visibly short. The X run
measures scale 0.909 (expected 1/1.10). After applying it, the robot returns to within 6 cm of its target.

## 5b. Real MentorPi findings (Test B)

* Pi 5, Debian 12 host; ROS 2 Humble runs in Docker container `MentorPi` (image `ros:humble`, host network),
  started by `start_node.service`. `ROS_DOMAIN_ID=0`. Shared folder: host `~/docker/tmp` = container `/home/ubuntu/shared`.
* Topics used: `/controller/cmd_vel` (Twist; 1 subscriber = HiWonder controller, 5 idle publishers, no
  continuous traffic), `/odom` (Odometry), `/imu` (Imu), `/scan_raw` (LaserScan, MS200). Also available:
  `/odom_raw`, `/odom_rf2o` (LiDAR odometry), `/imu_corrected`.
* `tf2_echo base_footprint laser_frame` reported that `base_footprint` does not exist: LiDAR mount still unverified.

## 6. Requires physical verification before trusting anything

1. Actual ROS 2 topic names, message types, QoS, and whether ROS runs in Docker (`discover_mentorpi.sh`).
2. That publishing `geometry_msgs/Twist` on the discovered topic moves the MentorPi as expected
   (sign conventions, units, HiWonder's own timeout behaviour, conflicts with HiWonder apps).
3. Robot dimensions, LiDAR mount pose and LiDAR angle convention (box-in-front check).
4. Odometry accuracy (sets `drift_per_m`) and IMU vs odometry heading.
5. Map ↔ arena registration: origin corner, axis directions, scale (0.03 m/cell assumed, not proven).
6. Real Wi-Fi latency and jitter (clock-sync expiry margin), and real stopping distance at speed.


## Camera + YOLO obstacles (added later)
* Teammate's WRO YOLOv5n ONNX model (unchanged, `third_party/wro_yolo`) runs on the laptop CPU at
  about 25 ms per example photo; it detected all blocks on the 3 example photos.
* 22 new automated tests (109 total, all passing):
  * floor-projection round trip with the real MentorPi lens calibration
  * calibration fit: blocks at new positions are placed within 2 cm
  * tracker: confirm after several frames; remove only when the spot is in view and empty; a block
    so close that it is cut off at the image edge is never removed
  * the bridge streams camera frames larger than the old 64 KiB limit, only when requested, while
    telemetry keeps flowing
  * closed loop with the simulated camera: a block invisible to the LiDAR is mapped within 3 cm and
    avoided; a block dropped on the route while driving causes a stop and a replan; a removed block is
    cleared and the straight route restored; an out-of-view block is kept
  * console: camera panel, map pins and in-console calibration (wrong 5 deg mount fitted back to 15 deg;
    pins within 3 cm afterwards)
* **Not yet tested on the real robot:** YOLO accuracy on the MentorPi camera view (the model was trained
  on top-down phone photos), real projection accuracy, and Wi-Fi throughput of the camera stream.

## Survivor search mission + console redesign (added later)
* `tests/test_nav_mission.py`: turn-in-place tracker; survivor list rules (green only, merged within
  15 cm, kept when the tracker forgets them); camera coverage; closed-loop search in the simulator
  (real bridge, simulated robot/camera): drives to the goal, finds and reaches both green blocks, ignores
  the red one, never collides, keeps running, STOP pauses it (robot disarmed), resume continues, Stop
  mission ends it with STOP latched; localization loss pauses it; automatic actions cannot clear STOP.
* `tests/test_nav_gui.py::test_search_mission_from_console`: search refused without camera, AUTHORIZE +
  SEARCH starts it, survivor row/star/log appear, Esc pauses (button shows RESUME MISSION), CSV export.
* Simulation only. **Not yet tested on the real MentorPi.** Expect pauses from the drift limit on long
  searches, and wrong survivor positions until the camera is calibrated.
