# Edge-controlled navigation for the HiWonder MentorPi-M1

> **Version 2 note.** This folder is the new version with the web console. Sections 1-10 describe the
> navigation stack, which is the same as before except where section 11 says otherwise. The Tk console
> (`ugv_nav/operator_gui.py`) was replaced by `ugv_console/` (see section 11 and `WHAT_IS_NEW.md`).

This adds physical-UGV navigation to the existing `ugv_astar` project **without
modifying any original file**. Your A* planner (`ugv_planner/`), GUI, data,
example outputs and tests are reused as-is.

```
EDGE LAPTOP (all planning + navigation decisions)          MENTORPI-M1 (executor)
┌───────────────────────────────────────────────┐         ┌──────────────────────────────────┐
│ ugv_planner  (YOUR existing A*, unchanged)    │         │ ugv_bridge/  (stdlib + rclpy)    │
│   NavigationService.replan() ── path          │         │  ├ auth + framing + seq + expiry │
│ ugv_nav/path_processing  validate (swept disk)│  Wi-Fi  │  ├ starts DISARMED               │
│ ugv_nav/localization     odom → map (SE2)     │◀──TCP──▶│  ├ limits, accel, LiDAR stop zone│
│ ugv_nav/controller       feedback tracker     │  HMAC   │  ├ WATCHDOG = only motor writer  │
│ ugv_nav/safety_monitor   2nd safety layer     │         │  └ backend: ros2 / dryrun / sim  │
│ ugv_nav/navigator        state machine        │         │        │ /controller/cmd_vel (?)  │
│ ugv_nav/edge_client      link, clock sync     │         │        ▼ HiWonder controller      │
│ ugv_console/             web console (v2)     │         │   /odom /imu /scan (?) → telemetry│
└───────────────────────────────────────────────┘         └──────────────────────────────────┘
```
`(?)` = candidate ROS topic names, **not yet verified on your robot**.

## 1. What was reused from your project

| Existing file | How it is used |
|---|---|
| `ugv_planner/astar.py` → `plan()` | The only path planner. 8-connected, octile heuristic, no corner cutting, swept-disk edge checks, clearance preference. |
| `ugv_planner/interfaces.py` → `NavigationService` | Called with the **localized map pose** as start; it already adds a collision-checked connector from the exact pose. |
| `ugv_planner/grid.py` → `Grid`, `load_environment` | Map frame, `cell_to_world` / `world_to_cell`, 0.03 m cells, in-memory row flip. |
| `ugv_planner/inflation.py`, `collision.py` | Footprint inflation, path validation, remaining-route checks after map updates. |
| `ugv_planner/config.py` → `Footprint`, `PlannerConfig` | Robot radius + margin, connectivity, clearance weight. |
| `ugv_planner/gui.py` → `Application` | Still runs unchanged with `python main.py`. Version 2's *Planner* mode gives the same plan and GT report in the web console. |
| `ugv_planner/exporting.py` → `draw_map` | Used for the closed-loop demo figures. |

Your `simulation.py` (ideal holonomic follower) stays the offline simulator. The
new closed-loop mode replaces it only in the operator console.

## 2. Files added

```
ugv_bridge/                      → deploy ONLY this folder to the Raspberry Pi
  protocol.py                    framing, HMAC-SHA256 mutual auth, session keys, seq numbers
  server.py                      bridge: validation, arming, watchdog/publisher, LiDAR stop, telemetry
  safety.py                      pure geometry (scan→points, stop corridor, stopping distance, slew)
  run_bridge.py                  CLI (refuses to start without measured dimensions / with open key perms)
  backends/ros2_backend.py       rclpy: Twist publisher + Odometry/Imu/LaserScan subscribers (UNTESTED on robot)
  backends/dryrun_backend.py     no-motor backend for communication tests
  config/bridge_config.example.json
  tools/discover_mentorpi.sh     READ-ONLY discovery of OS / Docker / ROS 2 topics / QoS / rates
ugv_nav/                         edge side (no ROS)
  edge_client.py  localization.py  path_processing.py  controller.py  safety_monitor.py
  map_update.py   navigator.py     sim_backend.py      sim_session.py  config.py
  fast_planner.py smoothing.py     scan_match.py       mission.py      vision.py  vision_sim.py   (see section 11)
ugv_console/                     web console (v2): app.py, views.py, options.py, server.py, web/
config/navigation.toml           edge settings (network, robot dims, speeds, controller, safety)
nav_main.py                      gui | sim-demo | check-connection | motor-test | genkey
tests/test_nav_*.py, tests/nav_helpers.py
NAVIGATION.md, TEST_REPORT.md
```

No new edge dependencies (NumPy, Matplotlib as before; Tkinter only for the original `main.py`; `tomllib` is
standard library on Python ≥ 3.11). The Pi side needs only Python ≥ 3.8 and,
for the ROS backend, the `rclpy` that HiWonder already ships.

## 3. Communication protocol

* TCP, 4-byte length prefix, max 64 KiB frame, JSON payload, **HMAC-SHA256 on every frame**.
* Mutual challenge–response handshake with a 32-byte pre-shared key → fresh per-connection session key.
* Strictly increasing sequence numbers per direction (replays and reordering are rejected).
* Edge pings every 0.2 s (heartbeat). The lowest-RTT pong gives the clock offset, and
  every `cmd_vel` carries an **expiry time in the UGV's clock**, so commands delayed
  by a Wi-Fi stall are rejected as stale instead of executed late.
* Bad MAC / malformed JSON / NaN / replay → connection closed, robot stopped and disarmed.
* Messages are authenticated, **not encrypted** (telemetry is readable on the network).

### UGV-side safety (independent of the laptop)

| Condition | Bridge reaction |
|---|---|
| Startup, new connection, reconnect | DISARMED; motion rejected until `arm` |
| `stop` / `disarm` | zero velocity immediately + disarm |
| No fresh command for `cmd_hold` (0.3 s) | zero velocity (watchdog is the only motor writer) |
| No valid command for `disarm_after_no_cmd` (1.0 s) | disarm |
| No frame at all for `heartbeat_timeout` (1.0 s) | drop client + disarm |
| Command stale / too far in future / malformed | rejected |
| Command > 1.5 × limit | rejected + disarm; otherwise clamped to limits |
| Lateral `vy` while `max_vy = 0` | rejected + disarm (initial non-holonomic mode) |
| Odometry or LiDAR stale | zero velocity; arming refused |
| LiDAR point in stop zone ahead (`margin + v·t + v²/2a`) or inside turning circle | zero velocity |
| Bridge exits (Ctrl+C / SIGTERM) | zero velocity published |

## 4. Coordinate frames and localization

* **Map frame** = the `ugv_planner` frame: origin is the **lower-left corner** of the
  100 × 100 grid (after the in-memory flip of image row 0 = top), x right, y up, 0.03 m/cell, 3 m × 3 m.
  Cell (r, c) centre = ((c+0.5)·0.03, (r+0.5)·0.03).
* **Odom frame** = wherever MentorPi odometry started. Raw odometry is never used as a map position.
* **Alignment**: the operator physically places the robot at a known map pose (x, y, heading) and
  presses *Confirm initial pose*. Then `map_T_odom = map_pose ∘ odom_pose⁻¹`.
* **Confidence**: σ = 2 cm + 5 cm/m travelled + 1 cm/rad turned (conservative placeholder; measure it
  in Test D). Navigation stops at σ > 15 cm, on stale odometry (> 0.5 s), on odometry jumps (> 15 cm per
  sample), or on non-finite values. The operator then re-places/re-confirms the pose.

**Whether the ERFNet map is metrically calibrated is NOT verified.** The arrays carry no origin,
rotation or scale. Before Test E you must mark the map origin corner and +x direction on the floor and
check at least two known features (e.g. a wall end) against the grid with a tape measure.

## 5. Path following

`ugv_nav/controller.py` is a rotate-then-drive tracker with lookahead and PD heading control:
projection onto the A* polyline (monotonic, windowed), carrot point 0.15 m ahead,
turn in place above 35° heading error, forward speed scaled by heading error, cross-track error and
distance to goal. It fails safe on > 10 cm cross-track error or no progress for 8 s. `vy` is always 0;
mecanum lateral motion is deliberately left for later.

## 6. Obstacles and replanning (edge only)

* The edge ignores LiDAR returns within σ + 2 cells of **mapped** walls (A* clearance already handles
  them) and reacts to **unmapped** returns inside its speed-dependent stop zone.
* On such a stop, unmapped returns within 1 m are written into an in-memory copy of the occupancy grid
  and A* is rerun **on the edge** from the current pose. The robot stays stopped and disarmed;
  the new route needs fresh operator authorization.
* The bridge's own stop zone sees all returns (no map knowledge), as an independent last line.

### Auto-continue after replan (optional, default OFF)

Turn on with the **Auto-continue after replan** checkbox in the console, or `auto_continue_after_replan = true`
in `config/navigation.toml`. Then, when an unmapped obstacle blocks the route:

1. the robot stops (edge + bridge),
2. the obstacle is added to the edge map and A* finds the new best route from the current position to the same goal,
3. after the robot has stood still for `auto_continue_delay_s` (1 s), the navigator re-arms the UGV and drives the
   new route **without pressing AUTHORIZE**, repeating for further obstacles,
4. up to `max_auto_replans` (5) times per operator authorization; after that it waits for the operator.

It does **not** auto-continue when: no route exists (robot stays stopped), nothing new could be mapped (only the
UGV's own stop zone fired, so the same route would just stop again), the operator pressed STOP (also cancels a
pending continue), or for Wi-Fi loss, localization loss, path-following failure or any other fault.

## 7. Installation

### Edge laptop (Windows, your existing `.venv`)
```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe nav_main.py genkey --out config\mentorpi.key
.\.venv\Scripts\python.exe nav_main.py gui
```
Add `config/mentorpi.key` to `.gitignore` (never commit it).

### Raspberry Pi (no installation into system paths, no services)
1. Copy the `ugv_bridge/` folder to the Pi, e.g. `scp -r ugv_bridge <user>@<PI_IP>:~/edge_bridge/`.
2. Copy the key: `scp config/mentorpi.key <user>@<PI_IP>:~/edge_bridge/ugv_bridge/config/` then on the Pi
   `chmod 600 ~/edge_bridge/ugv_bridge/config/mentorpi.key`.
3. Run discovery (read-only): `bash ~/edge_bridge/ugv_bridge/tools/discover_mentorpi.sh | tee discovery.txt`.
   If ROS 2 runs inside a HiWonder Docker container, run it inside the container too.
4. `cp bridge_config.example.json bridge_config.json`, fill in **measured** `half_length_m`,
   `half_width_m`, the **verified** topic names, and the LiDAR mount.
5. Communication-only first: `cd ~/edge_bridge && python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend dryrun`
6. Only for Test C onward, and only with ROS sourced in the same environment as HiWonder's nodes:
   `--backend ros2`. If ROS runs in Docker, the bridge must run inside that container (or one sharing its
   network/DDS domain). This is decided from the discovery output.

## 8. Staged test procedure (do not skip stages)

| Stage | Command / action | Pass criteria |
|---|---|---|
| A software | `python -m unittest discover -s tests -v` | all pass |
| B comms (no motors) | Pi: `run_bridge --backend dryrun`; laptop: `nav_main.py check-connection --host <PI_IP>` | link OK, armed False, RTT/telemetry ages reported; then `--backend ros2` with no motion commands: real odom/imu/scan visible |
| C wheels off ground | robot on a stand; `nav_main.py motor-test --host <PI_IP>` (asks for typed confirmation, each phase needs Enter) | wheels turn in commanded direction; STOP halts; watchdog zeroes output ≤ 0.35 s and disarms ≤ 1.1 s with **no** STOP sent |
| D manual floor | clear level floor, small supervised moves, compare odometry vs tape | measure scale/heading error → set `drift_per_m`, choose `heading_source` |
| E single waypoint | GUI Physical mode, goal ≤ 0.5 m in verified-clear space | arrives within 5 cm (estimate) and check true error by tape |
| F multi-waypoint | short A* path with one turn | no cross-track faults; true error recorded |
| G arena | ENV map, verified-clear route, recalibrate when the drift guard stops | arrival, no contact |
| H safety | pull Wi-Fi / kill laptop app / cover LiDAR / place box on route / odometry reset | robot stops each time; never resumes without authorization |

## 9. Known limitations

* Wheel odometry alone cannot hold < 15 cm over a full 4–9 m arena route (it stops after ~2.6 m
  with the placeholder drift rate). Full-arena autonomy needs operator re-localization stops,
  or a LiDAR scan-matching localizer against the map (future work: the scan points and map
  are already on the edge).
* Map/arena registration, LiDAR mounting and topic names are unverified until the robot is inspected.
* The edge filter treats returns near mapped walls as known; a small object placed against a wall is
  only caught by the bridge's (short) zone.
* `ugv_planner/batch.py` from your RAR could not be extracted here (archive decoder error);
  your own copy is untouched by this work.

## 10. Survivor search mission (added later)
`ugv_nav/mission.py` (`SearchMission`) runs on the edge in the navigator's control thread:
TRANSIT (A* to the operator goal) → SCAN (turn on the spot in `scan_step_deg` steps, hold still
`scan_dwell_s` at each, `SpinTracker` in `navigator.py`) → a confirmed YOLO **green** object is
registered as a survivor (merged if within `same_survivor_m`) → APPROACH (A* to a free spot
`approach_gap_m` in front of it, preferring spots with extra room) → FACE it → status *reached* →
SCAN again. With nothing new, EXPLORE: A* to the reachable viewpoint with the most floor the camera
has not covered (ray-cast field of view, `view_range_m`, stops at walls). When the reachable floor
is covered, a new pass starts. The mission only ends with **Stop mission**.

Safety: the mission starts only from an operator authorization (AUTHORIZE with search ticked, or
*Search from here*), and every motion goes through the same navigator checks (localization, LiDAR
zones, UGV watchdog, speed limits). Operator STOP, link loss, localization loss and unknown faults
**pause** it; only the operator can resume. Obstacle stops and failed routes are retried / skipped
automatically, but `max_failures` failures in a row pause it. Automatic actions can never clear a
latched STOP. Steering is unchanged from the original: the robot drives and steers together (`align_first_deg` =
35, the same as `rotate_threshold_deg`).

Limitation: localization is odometry only, so the drift estimate grows with every metre and turn;
the mission pauses at `max_sigma_m` and needs a pose re-confirmation (no scan matching yet).


## 11. Version 2: web console, faster planner, smoothing, camera tilt, LiDAR matching

### Console (`ugv_console/`)
```
browser page (web/)  ◀── GET /api/events   Server-Sent Events: "fast" sample ~20 Hz + changed sections
                     ◀── GET /api/frame    newest camera JPEG + the YOLO boxes of that frame (long poll)
                     ──▶ POST /api/cmd     {"cmd": name, "args": {...}}  ->  ConsoleApp.command()
```
* `app.py` `ConsoleApp`: all console behaviour as validated commands (`cmd_*` methods are the whitelist),
  no GUI toolkit. `views.py`: JSON-ready state sections. `options.py`: one table of every setting with type,
  range and default; everything from the page or a settings file is coerced through it.
* `server.py`: standard-library HTTP server bound to 127.0.0.1. Every `/api` request needs the per-run token;
  `Host` must be this server (DNS rebinding), a foreign `Origin` is refused, there are no CORS headers, bodies
  are size-limited JSON. Static files come only from `web/`.
* Authorization is two commands: `authorize_prepare` (checks, summary text, one-time code valid 60 s, bound to
  the current route) and `authorize_confirm`. `stop` takes no lock.
* Presence: the event stream connection is the console's "I am here". When no page is connected for
  `deadman_s` (1.5 s) while the robot moves or a mission is active, the server sends STOP. Without `--stay`
  the program exits 25 s after the last page closed.
* The page renders at display rate and interpolates the pose between samples (render time = server time
  minus about 1.8 sample intervals), so the drawn robot is smooth and never ahead of the last report.

### Planner (`fast_planner.py`, `smoothing.py`)
`RoutePlanner('fast')` runs the same search as `ugv_planner.astar.plan` (same neighbour order, costs,
heuristic, tie-breaking and floating-point expressions) on flat lists and without the per-edge swept-disk
check. That check cannot fail inside the search: moves stay inside the segment (orthogonal) or the unit square
(diagonal, both orthogonal neighbours required) spanned by unblocked cell centres, and on that set the distance
to any lattice-aligned occupied square is smallest at one of those centres, which is already further than the
planning radius. The finished route is verified with the exact checker (fallback: original planner). Inflation
mask, penalties, checker and distance fields are cached by map content + footprint.

`smooth_route` densifies the route (6 cm), then greedily replaces sub-paths by straight segments when
(1) the segment is clear with the planning radius (conservative test on the exact distance field, exact
`CollisionChecker` when in doubt) and (2) the planner objective, length x (1 + weight x penalty) integrated
every centimetre with bilinear penalty lookup, is not larger than along the replaced part. Start pose and goal
are never moved; the result is re-validated with exact checks.

### Vision (`vision.py`)
* `pitch_from_box`: with image angles a_b (bottom edge, near floor edge of the block) and a_t (top edge, far
  top edge) below the optical axis, forward distances are `d_near = h_cam / tan(p + a_b)` and
  `d_far = (h_cam - H) / tan(p + a_t) = d_near + depth`; solved for the tilt p by bisection.
  `TiltEstimator` keeps the last 80 samples (0.25-1.6 m), median + MAD, confident with >= 12 samples and
  spread <= 1.2 deg. `VisionWorker(auto_tilt=True)` applies it (drops tracks made with a tilt more than
  1.5 deg off) and reports through `on_camera_change`.
* `ObstacleTracker`: weight = confidence / range^2 (capped sum), class by weighted vote, association
  independent of class (two boxes in one frame stay two objects), navigator notified when an estimate moved
  more than 3 cm.

### LiDAR <-> map (`scan_match.py`)
Likelihood field `exp(-d^2 / 2 sigma^2)` (sigma 5 cm) from the exact distance field of the operator map; the map
boundary counts as a wall. `match` scores a coarse grid (3 cm, 2 deg) around the estimate, refines (1 cm,
0.5 deg) and is `ok` only if the score is at least 0.40, poses within 0.03 of the best score are not spread
over more than 9 cm / 6 deg, and the best is not on the window edge. `Navigator.relocalize_with_lidar`
(operator, robot still, window 30 cm / 20 deg) and `lidar_correction` (beta, about 1 Hz, window 9 cm / 6 deg,
half steps above 4 cm) call `PoseEstimator.correct`, which restarts the drift estimate like a confirmed pose
but never clears an odometry fault.

### Navigator changes
`set_map(grid, keep_lidar=True)` keeps LiDAR-found cells as their own layer (`lidar_mask`,
`clear_lidar_obstacles()`); `fast_state()` and `snapshot(blocking=False)` never wait for planning;
`loop_stats()` reports control-loop rate and worst gap; the stall check uses a never-reset odometer.
