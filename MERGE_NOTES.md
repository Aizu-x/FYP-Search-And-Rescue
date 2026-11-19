# Merge notes: FYP_SRM

18 November 2026. This folder is the UGV / UAV project (A\* navigation, YOLOv5, operator console, Tello section)
with the team's **combined system** (DAG scheduler + ORB-SLAM3 handoff) merged into it.

The rule for the merge: **the UGV / UAV project is the core.** Where the two disagreed, the combined system was
changed to fit, and the core's program was left as it was. (Three of the core's test files were changed
afterwards, on request: see "Changed later".)

## What came from where

| Part | From |
|---|---|
| `ugv_planner/`, `ugv_nav/`, `ugv_console/`, `ugv_bridge/`, `uav/`, `tests/`, `docs/`, `config/`, `data/`, `third_party/`, `nav_main.py`, `main.py`, `START_CONSOLE.bat`, `UPDATE.bat`, `requirements*.txt` | The core: the project folder as it was on 18 November 2026 (GitHub `main`, commit `cb9fe29`, plus this computer's own settings, key and edited maps) |
| `DAG_scheduler/`, `scripts/`, `assets/`, `verification/`, `COMBINED_INTEGRATION.md`, `EDGE_SETUP.md`, `THIRD_PARTY_NOTICES.md`, `RUN_COMBINED.cmd`, `run_combined.sh`, `requirements-combined.txt`, `.gitattributes` | `FYP_Combined_System_ready.rar`, folder `FYP_Combined_System` |
| `ORB_handoff/` (ORB-SLAM3 source, settings, saved results, two Tello recordings: 1,388 files, about 700 MB) | `FYP_Combined_System_ready.rar`, folder `ORB_assets` |
| `FYP_SRM.exe`, `FYP_SRM.ini`, `launcher/`, this file | New, made for this folder |

## What the comparison showed

- The combined system was built on an **older version** of the core: commit `e9ed19b`. The core has moved on
  since then (mission video recording, the UAV section with manual and planned flight).
- The two have 110 files in common. 93 have the same content. 15 more are, in the archive, exactly the old
  `e9ed19b` versions: 13 are simply newer in the core, `config/navigation.toml` carries this computer's own
  settings, and one icon differs only in its line endings. So nothing in those 15 was the combined system's own
  work. Only **two shared files were really changed by the combined system**: `README.md` and `.gitignore`.
- The combined system reaches the core through `DAG_scheduler/ugv_adapter.py` and `combined_adapter.py`. They
  call `ugv_planner` (A\*), `ugv_nav` (path checks, map loading, the detector) and
  `third_party/wro_yolo` (the model). **In the repository those files are the same, byte for byte, at the old
  and the new commit** (the only change under those folders is the added `ugv_nav/recorder.py`), so no code had
  to be adapted: the combined system's tests pass on the new core as they are.
- The combined system's code was read before it was run. It opens no network connection, does not import the
  console, the bridge or the robot link, and cannot send a movement command.

## What was changed so that it fits

| File | Change | Why |
|---|---|---|
| `README.md` | The core's README is kept. The combined system's introduction became a section in it, plus rows in the layout, status, documentation and credits tables. | The archive's README put the combined system first and described the core as an older "original project" |
| `.gitignore` | Both lists joined. | The core's list had `data/uav_plans/` and `recordings/`, the archive's had `combined_runs/`, `ORB_handoff/` and others |
| `RUN_COMBINED.cmd` | It now takes Python from `FYP_SRM.ini`, then from `.venv` in this folder. | It only accepted `.venv` inside the folder. On this computer the environment is somewhere else, so it refused to run |
| `RUN_COMBINED.cmd`, `DAG_scheduler/RUN_DEMO.cmd`, `DAG_scheduler/RUN_ORB_ARCHIVES.cmd` | Windows line endings (CRLF). No other change in the last two. | They were stored with Unix line endings, which can break `goto` in a `.cmd` file. The archive's own `.gitattributes` asks for CRLF |
| `EDGE_SETUP.md`, `COMBINED_INTEGRATION.md`, `THIRD_PARTY_NOTICES.md`, `verification/COMBINED_RESULTS.md` | A short "FYP_SRM" note in each. The original text is untouched. | They describe the old core version and a setup that is already done here |
| `ORB_handoff/` | Placed in the project folder. | That is where the runner looks first. In the archive it was in a separate `ORB_assets` folder, where the runner does not find it |

**Not changed:** every program file of the core, all Python code in `DAG_scheduler/` and `scripts/`, and
`requirements-combined.txt`. That file pins NumPy 2.5.5 and ONNX Runtime 1.32.0; this computer has 2.5.4 and
1.31.1. Both are inside the ranges the core asks for and every check below passes with them, so nothing was
installed or upgraded.

### Changed later, on request

Three files in `tests/` (`nav_helpers.py`, `test_uav_link.py`, `test_uav_video.py`), the same evening. Two UAV
tests check that the console rejects a packet that pretends to come from the drone but comes from another
address. To do that they send a packet from a second address of this computer (127.0.0.2). When a computer
does not deliver such a packet, there is nothing to reject and the tests used to report "failed", although
nothing was wrong with the console.

- Now each of the two tests first checks whether this computer delivers a packet sent from 127.0.0.2.
- If it does, the test runs exactly as before.
- If it does not, the test reports **"skipped"** and says why. The first half of the link test (a packet from
  the right address but the wrong port) is still checked in that case.

No program file was touched by this change: the console behaves as before.

## Added after the merge: the team simulator

19 November 2026, on request. The simulator can now run **2 to 4 simulated robots** as a team, and this is where
the DAG and Kahn's algorithm of the combined system take effect inside the console: the scheduler's Kahn
function decides which robot may drive when. The guide is [docs/TEAM.md](docs/TEAM.md).

- **New files:** `ugv_nav/team_plan.py`, `ugv_nav/team_sim.py`, `ugv_console/team_console.py`,
  `ugv_console/web/js/team.js`, `docs/TEAM.md`, `docs/images/console_team.png`, three test files
  (`tests/test_team_*.py`, 82 tests).
- **Files of the core that were changed for it:** `ugv_console/app.py`, `views.py`, `options.py`, `server.py`,
  and on the page `index.html`, `style.css`, `js/ui.js`, `js/main.js`, `js/api.js`, `js/camera.js`. The changes
  are additions: a mixed-in class, two settings, one more camera address, the team view. README and the guides
  were brought up to date.
- **What stays as it was:** with **1 robot** (the default) the simulator, the real-robot mode and the UAV
  section work as before; the single-robot code path was not rewritten. The A\* planner, the navigator, the
  bridge and the scheduler's own code are unchanged. The team uses the scheduler's `topological_order` and
  `Task` as they are.
- **Simulator only:** a team is built from simulated robots and nothing else. There is no way to connect it to
  a real robot.
- The files of the core that now differ from GitHub `main` are the ten listed above, the three test files of
  "Changed later", `README.md`, `.gitignore`, `docs/USER_GUIDE.md` and `docs/TESTING.md`.
- **Tested** on 19 November 2026 (Windows 11, Python 3.14.6, simulated robots only): the complete suite, now
  424 tests run, **421 passed, 2 skipped, 1 failed** (the 2 skipped ones are the two UAV tests described under
  "What was tested").
  The failed one is `test_field_is_measured_by_putting_the_robot_into_two_corners` (single robot: the simulated LiDAR draws the room while the robot drives in real time). It failed in three of the last four full runs, each time with the same message (the scan fits the map only 38 to 39 %), and passed every time it was run on its own or with the rest of its test file (5 of 5). The code it tests was not changed for the team simulator. Why it fails inside a full run is not known yet.
  The team page was also driven in a headless browser with its own controls: three robots
  placed with the mouse, goals, a block, plan, authorization, the drive, STOP with Esc, and back to one robot,
  without a page error. The team simulator is a simulator feature by design.

### Second round, the same day (after the first real use)

A team of three in ENV3 did not finish: one robot hit a wall, and the robot waiting for it never started. What
was found and changed:

- **The real robot's scale calibration was applied to the simulated team robots.** With *X scale* 0.9 a
  simulated robot believed it was 20 cm away from where it really was, and drove into a wall. Team robots now
  have exact odometry and the calibration is not applied to them (`ugv_nav/team_sim.py`).
- **The team drove in the data set's "true" arena, not in the map on the page.** With an edited map the two
  differ, so a robot met a wall the page did not show. The team's arena is now the operator map as drawn
  (`ugv_console/team_console.py`).
- **One stopped robot ended the mission for those waiting for it.** Now a robot that stops for something it can
  get over plans again and drives on (up to 3 times). One that stops for good is an obstacle where it stands:
  the waiting robots go if it is out of their way, take another way around it if there is one, and only
  otherwise stay ("cannot start", with the reason on the page). A robot without a goal stays where it is and
  the others drive around it.
- **The page:** the mode *Robot* is called **UGV**, and the top bar reads UAV, UGV, Simulator, Planner. In the
  team view the tabs stay: *Drive* is the team panel, *Robot* keeps the footprint, planner and field size;
  *Map*, *Camera* and *Search* belong to the single robot. The arena can be chosen in the team panel, a goal
  can be removed, and each robot's row shows why it stopped.
- **A STOP could be written down as a failure.** STOP halts the robots without waiting for the team's
  supervisor; a supervisor step that was under way at that moment took the stopped robots for failed ones
  (found by the full test run, once). STOP now announces itself first, and the mission always ends as
  *stopped*.
- **Not changed:** the single-robot simulator. It still applies the scale calibration and drives in the data
  set's arena, as before.
- Also changed for this: `docs/ROBOT_SETUP.md` and `docs/UAV.md` (the mode's name).

### Third round, the same day (two things asked for)

- **The simulated camera saw through walls.** It now checks the arena's walls: a block behind a wall is neither
  detected nor drawn, and the walls are in the camera picture. This is in `ugv_nav/vision_sim.py`, a file of
  the core, so it applies to the **single-robot simulator as well** (asked for as "in the simulator"). For the
  camera the simulator's walls are 20 cm high (`WALL_HEIGHT_M`), higher than the lens; the real height of the
  arena walls is not written down anywhere in the project, so this is an assumption to check.
- **Team robots only reported survivors.** With *Drive up to survivors found* (on by default) a robot that has
  seen a survivor drives up to it after its other tasks, faces it and stands there 3 seconds: secured. One
  robot per survivor; it waits until its way is clear of what the others still have to drive. One more task
  per robot in the graph (`r<i>_rescue`). The distances and times are those of the single robot's search
  mission (`MissionConfig`), which itself is unchanged.
- **Found on the way while testing this with the operator's map:**
  a survivor in a narrow passage closed a robot's route and the robot just stopped: it now goes to that
  survivor instead, and the mission ends as complete;
  a robot could plan the same route again and again from one place without end (the planner took a gap for
  wide enough, the robot's safety stop did not): it now gives up after 4 such routes;
  a robot waiting to go to a survivor waited for a team-mate that was never going to move.
- **Looking around is slower** (about 25 s instead of 8 s per robot): the camera only maps what it sees while
  the robot turns slowly, and finding survivors is what the look-around is for.
- Files changed: `ugv_nav/vision_sim.py` (core), `ugv_nav/team_sim.py`, `ugv_nav/team_plan.py`,
  `ugv_console/team_console.py`, `ugv_console/options.py` (one more setting), the page (`index.html`,
  `js/team.js`, `style.css`), `tests/test_nav_vision.py` (core, 5 tests added), the three team test files,
  `docs/TEAM.md`, `docs/USER_GUIDE.md`, `docs/TESTING.md`, `README.md`.

## What was tested

All on this laptop (Windows 11, Python 3.14.6), with the simulated robot and the simulated drone. The real
robot and the real Tello were tested separately by the project author.

The tests were run on a clean copy of the merged files with the stock settings. This folder itself carries this
computer's own settings (`config/navigation.toml`, `config/navigation.local.toml`), with which 7 of the core's
tests were found earlier to fail by design. The combined system's 62 tests and one full combined check were
also run inside this folder itself, with those settings, and passed.

| Check | Result |
|---|---|
| The combined system's own tests (`python scripts/verify_combined.py`) | **62 passed**, with no change to its code |
| Combined check, Video 1 and Video 2 (`DAG_scheduler/run_combined.py`) | Both `COMPONENT_INTEGRATION_SUCCEEDED`, with the same figures the teammate recorded: 17 poses / 1,755 points and 21 poses / 1,943 points; 126 waypoints, 4.19735 m; 4 detections in 3 images |
| `RUN_COMBINED.cmd` | Runs with the Python from `FYP_SRM.ini`; explains what to do when no Python is found |
| `DAG_scheduler/RUN_DEMO.cmd`, `RUN_ORB_ARCHIVES.cmd` | Both run as they are |
| `run_mission.py --mode real --check` | Lists the connections that are still missing and stops, as designed |
| The core's own tests (337, the two old Tk window tests left out as always) | Final run: **335 passed, 2 skipped, 0 failed** (below) |
| `FYP_SRM.exe` | Starts the console hidden and the page is served; ends quietly 25 s after the page is closed; the three error boxes (Python has no packages, no Python found, exe moved out of the folder); the console keeps running when the launcher is closed and the next start takes it over; a second start opens no second console and no second window |
| The picture of `FYP_SRM.exe` | `BUILD.cmd` builds it from `launcher\icon.png` (also run inside this folder, and the resulting `.exe` tested here); pictures that are not square, small, a JPG or not a picture at all; a missing picture; no rebuild while the console is running |

The core's tests were run in full twice.

- **Final run** (after the change to the two UAV tests, with nothing else running on the laptop): 337 tests,
  **335 passed, 2 skipped, 0 failed**. The two skipped ones are `test_datagrams_from_elsewhere_are_dropped` and
  `test_video_from_another_address_is_dropped`. On the evening of 18 November, Windows on this laptop did not
  deliver packets sent from 127.0.0.2, so the forged packet these tests need could not be made. A ten-line
  script without any project code shows the same; why Windows does this is not known. Both tests had passed at
  17:38 the same day, so on a computer that delivers such packets they run in full.
- **First run** (before that change, with other work going on on the laptop): 334 passed, 3 failed. Two were
  the tests above, then still reported as "failed"; they failed in the same way on the unmerged project (commit
  `cb9fe29`). The third, `test_field_is_measured_by_putting_the_robot_into_two_corners`, is a simulator test with
  a time limit: it passed twice when run alone and passed in the final run.

A separate check showed that the changed tests still do their job where a second address can be used: with a
deliverable packet put in its place, both run on to their "was it rejected?" line.

Not checked: whether a second start of `FYP_SRM.exe` really puts the console window in front of other windows.
The test could confirm that no second console and no second window appear; bringing a window to the front is
something Windows only allows for a program the user has just started by hand. The "Starting..." window was
checked by drawing it off-screen, with the picture from `icon.png` and with a replaced picture.

## Scope of this package

- The checks in this file ran in software. The real robot and the real drone were tested separately by the
  project author.
- A fresh ORB-SLAM3 run needs Linux / WSL and a compiled `mono_tum`; the combined check imports the saved ORB
  results.
- ORB-to-navigation-map conversion is not part of the package: A\* plans on the supplied occupancy maps.
- The scheduler is **not connected to the robot**. In `DAG_scheduler/team_adapters.py` the tasks `run_erfnet`,
  `fuse_map`, `prepare_planning_map`, `select_target`, `get_ugv_pose`, `navigate` and `stop_navigation` are
  placeholders, and `run_mission.py --mode real --check` lists them as missing, as it is meant to.
- The console has no button for the combined check. It is started with `RUN_COMBINED.cmd`.
- The UAV section and the ORB-SLAM3 part are not linked: the combined check uses the two saved Tello
  recordings, not a flight flown from the console.
- The team simulator (added later, see below) uses the scheduler's Kahn algorithm for simulated robots only.
  The scheduler's process-based runner and its placeholders for the real robot are still unconnected.

## The .exe

**`FYP_SRM.exe`** opens the console without a terminal: double-click it. It does what `python nav_main.py gui`
does, hidden.

- **Which Python:** the path in **`FYP_SRM.ini`** (`python=...`). If that file is missing or the path is wrong,
  it tries `.venv` in this folder, then `python` on the PATH. Change the line when the environment moves.
- **While starting** it shows a small "Starting..." window, then the console window appears.
- **If it fails** it shows the console's last messages in an error box. The full messages of the last run are in
  `launcher\last_run.log`.
- **Double-click it again** while the console is running: it brings the console window to the front (or opens
  the page again if the window was closed). It does not start a second console.
- **If the launcher itself is closed** (Task Manager), the console keeps running; the next double-click finds it.
- **Closing:** close the console window. The program ends by itself about 25 seconds later, as it always did.
- **Keep it in this folder.** To start it from the Desktop or the taskbar, make a shortcut to it.
- It is a small starter, not a packed copy of the program: Python and the project files are still needed.
  It never talks to the robot or the drone itself.
- **Source:** `launcher\FYP_SRM.cs` (about 700 lines of C#). Rebuild with `launcher\BUILD.cmd`, which uses the
  compiler that is part of Windows. `launcher\run_console.py` is the small Python file it starts.
- `START_CONSOLE.bat` still works as before for a terminal start.

### Its picture

The picture of `FYP_SRM.exe` is one normal image file: **`launcher\icon.png`**. Right now it is the console's
robot icon (512 x 512 pixels). To use a proper design later:

1. Save the design as a **square PNG**, 256 x 256 pixels or larger (512 x 512 is good), named `icon.png`.
   A see-through background is fine.
2. Put it in the `launcher` folder, **replacing** the `icon.png` that is there.
3. Close the console, then double-click **`launcher\BUILD.cmd`**. It builds the picture into `FYP_SRM.exe`.

- The "Starting..." window reads `icon.png` itself, so it shows the new picture straight after step 2.
- The icon Windows shows on the file changes with step 3, because Windows keeps a program's icon inside the
  `.exe`. `BUILD.cmd` makes all the sizes Windows needs (16 to 256 pixels) from the one picture
  (`launcher\MakeIcon.cs` does that) and needs nothing installed.
- A picture that is not square is centred with see-through edges. A JPG also works if it is named `icon.png`.
  If the file is missing or cannot be read, `BUILD.cmd` says so and leaves the old `.exe` as it is.
- The console window itself, and its taskbar button while it is open, use a different file,
  `ugv_console\web\icon.svg`, which belongs to the console and was not changed.
