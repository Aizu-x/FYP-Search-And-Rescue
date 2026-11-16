# What is new in version 2

Version 2 lives in its own folder. The old project folder and its zip were not changed.
Unchanged copies inside this folder: `ugv_planner/` (your A*), `ugv_bridge/` (robot side),
`third_party/wro_yolo/` (YOLO), `data/`, `example_outputs/`, `main.py`.

**Everything below was tested in simulation and with automated tests on a development computer.
Nothing in version 2 has been tested on the real MentorPi yet.**

## 1. New console
* A local web page instead of the Tk window: `ugv_console/web/` (HTML/CSS/JavaScript, no internet and
  no extra Python packages needed), served by `ugv_console/server.py` to this computer only.
* The map is drawn 60 times a second. The robot's position arrives about 20 times a second and is
  interpolated in between (shown about 0.1 s behind), so motion is smooth and never ahead of what the
  robot reported. Measured in a test browser while the simulated robot drove: 16.7 ms per frame (median).
* Drawing the map no longer happens inside the Python program, so the control loop that sends driving
  commands is not slowed down by the display. In the simulator it held 10.0 Hz with at most 13 ms of work
  per cycle (shown live in the *System* card).
* New markers: true-size robot (body, wheels, front light, heading), state ring (driving / stopped / arrived),
  drift ring, camera view cone, safety stop zone, start outline with heading arrow (drag to turn), goal flag,
  animated route, fading driven trail, LiDAR points coloured "on a mapped wall" / "not on the map", true-size
  blocks, survivor pins with status, searched-floor shading, hover tool-tips with exact coordinates.
* Tools on the map: move/zoom, start (drag = heading), goal, paint, erase, test block, measure distance.
* Dark and light theme. Guided steps (Connect, Place robot, Goal, Go) and a hint under the AUTHORIZE button
  that says what is still missing.

### Safety of the new console
* Motion still starts only after AUTHORIZE. Authorization is now two requests (question with a one-time code,
  then the confirmation); a changed route or an old code is refused.
* STOP (button or Esc) never waits for planning or any other operation.
* If the last console page closes while the robot is moving, the robot is stopped (within about 2 s).
  The robot's own watchdog still stops it if the laptop program dies.
* The page is reachable only from this computer (127.0.0.1), needs a random key that changes at every start,
  and refuses requests from other web pages or with a wrong host name. Every command is validated again in Python.

## 2. A* improvements
| Change | Effect (measured) | Switch |
|---|---|---|
| **Fast planner** (`ugv_nav/fast_planner.py`): the same A* search without a per-step geometry check that can never fail there, plus caching of the inflation and clearance maps. The finished route is still verified with the exact check. | Identical routes to `ugv_planner` on every tested query (3 maps x 8 settings x random start/goal, pose connector, random maps). Example: 755 ms -> 13 ms. | `engine = "original"` in `config/navigation.toml` |
| **Route smoothing** (`ugv_nav/smoothing.py`): replaces zig-zag parts by straight lines only if the robot's swept disk stays clear with the full planning radius AND the planner's own cost (length x clearance preference) does not get worse. | ENV1 example: 25 -> 11 corner points, total turning 1035 deg -> 225 deg, corners sharper than 35 deg (where the robot stops and turns on the spot) 23 -> 2, length 4.20 -> 4.00 m, smallest wall distance unchanged. | Robot tab: *Smooth route* |
| A "shorter route?" check after an obstacle disappears now runs beside the control loop instead of blocking it. | No command gap while the laptop plans. | - |
| The remaining-route check runs only when the map changed (not every cycle). | Less work per control cycle. | - |

The planner mode (old "offline" mode) shows the same report as before (GT clearance, turns, detour ...). With
*Smooth route* off it shows exactly the original `ugv_planner` route.

## 3. YOLO / camera improvements
| Change | Effect (measured in simulation) | Switch |
|---|---|---|
| **Camera tilt from block size**: the known 10 cm block height gives the tilt from a single box, independent of the configured tilt. Sightings are collected; when they agree the tilt is applied and saved. While the first sightings disagree with the configured tilt, blocks are held back for a moment instead of being pinned in the wrong place. | Tilt within about 0.4 deg from one box with 1.5 px jitter. With the tilt set 10 deg wrong: pin more than 10 cm off before, within 4 cm after, no manual calibration. | Camera tab: *Auto tilt from block size* (or the button *Use measured tilt now*) |
| **Distance-weighted position**: close sightings count more than far ones (a small tilt error is a large distance error far away). | A pin 8 cm off from far sightings moves to within 2 cm after three close ones. | - |
| **Class voting**: one object seen sometimes as red and sometimes as green stays one obstacle with the majority class (no false "survivor" next to a red block). | - | - |
| **Shape filter**: boxes far too wide or flat for a block (for example the long arena barrier) and, once the tilt is known, boxes whose size contradicts their distance are shown but not mapped. Real blocks in your teammate's example photos have width/height 0.65-0.71; the filter accepts 0.2-2.4. | - | Camera tab: *Ignore boxes that cannot be a block* |
| The live view shows the robot's own JPEG frames (not re-encoded) with the boxes of exactly that frame: class, confidence, distance and why a box was not used. | - | - |

Manual calibration (Add sample, Fit) is still there; it also fits the forward offset and yaw.

## 4. Position (LiDAR <-> map)
| Change | Effect (measured in simulation) | Switch |
|---|---|---|
| **LiDAR fit** indicator: share of LiDAR points that lie on mapped walls at the believed position. | Shows a wrong position before it causes a stop. | always on |
| **Re-localize (LiDAR)**: fits the scan to the map in a small window around the believed position and corrects it. Refused if the fit is poor, ambiguous (for example a featureless corridor) or at the edge of the search window. | Start error about 8 cm / 6 deg -> about 1 cm / 0.3 deg (median), 21 ms per fit. | Drive tab button, banner button after a drift stop |
| **Continuous correction (beta)**: the same fit about once a second with small steps. | A drive of about 3 m with 8 % odometry error and the default drift limit: with correction the robot arrives (position error under 6 cm on arrival, under 3 cm after standing 2 s); without it the drift limit stops the drive. | Robot tab, default OFF |

Caution for the real robot: this relies on the map matching the real walls. Your edited maps should match; the
*LiDAR fit* bar tells you how well.

## 5. Other changes
* Obstacles found by the LiDAR are a separate purple layer (Map tab: *Add to map* / *Clear*). Before, they
  were mixed into your map as unsaved edits.
* The view rotation is not remembered between sessions (it starts normal: x to the right).
* Unsaved map edits are kept as a backup file in `data/edited/` when the program exits.
* `nav_main.py import-old <old folder>` copies your key, edited maps and settings (read-only on the old folder).

## 6. Not changed
* Driving behaviour of the path follower (it drives and steers together as before), speed limits, LiDAR stop
  zones, drift limit, stall check ("NOT really moving"), survivor search logic.
* The robot side: `ugv_bridge/` is byte-identical, same protocol, same key.

## 7. Not tested
* Anything on the real MentorPi (motors, real LiDAR, real camera frames through the new console).
* Opening the console window on Windows (the code looks for Edge/Chrome/Brave and falls back to the default
  browser; the page itself was tested in Chromium).
* The real YOLO model inside the new console (it was run on the example photos only; the console was tested
  with the simulated detector).

## Update: orientation, goal, "secured" mission, stricter YOLO, Git

| Change | Detail | Checked by |
|---|---|---|
| ENV2 / ENV3 orientation | Shown as the real arenas stand: ENV2 turned 90° clockwise, ENV3 90° counter-clockwise. Only the in-memory map is turned; `data/*.npy` and saved edited maps keep their format. | `OrientationTests`, screenshots compared with the operator's pictures |
| No preset goal | Opening the console or changing the environment leaves the goal empty; nothing is planned until you set one. | `PlannerModeTests`, browser run |
| Search mission | First green object seen is approached (also during the drive to the goal), robot stops at *How close* (default 25 cm), faces it, stands 3 s, status **secured**; another known green is next, otherwise **mission complete**. | `test_first_green_is_approached_held_then_secured_and_the_mission_ends`, browser run |
| Door / circle seen as a block | Rejected: round or square boxes, boxes far taller or smaller than a 10 cm block at that floor distance, boxes without their colour. Minimum confidence 60 %, 4 sightings. | `NotABlockTests` |
| Two greens for one object | Far sightings scatter more, so they are matched in a wider radius; same-colour pins closer than 18 cm are joined; the mission treats greens within 30 cm as one survivor. | `OneBlockOnePinTests` |
| Git updates | Your values live in `config/navigation.local.toml` (ignored by Git) so `git pull` never clashes. | `test_import_old` |

All of this is tested in simulation and in a browser only. The new YOLO filters have **not** been tried on the real
camera: if a real block is ever ignored, the camera view shows the reason on the box, and each filter has its own
switch in the Camera tab.
