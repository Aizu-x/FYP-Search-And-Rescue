# START HERE (version 2: new console)

This folder is a **separate, new version** of the A* + YOLO project. Your old project folder is not needed by
it and was not changed. If anything here does not suit you, keep using the old folder exactly as before.

**Nothing changes on the robot.** The bridge program on the MentorPi and the shared key stay as they are.

## 1. One-time setup (PowerShell terminal in VS Code, inside this folder)

Simplest: reuse the Python you already set up in the old folder (no install, no internet):
```powershell
& "C:\path\to\OLD project folder\.venv\Scripts\python.exe" nav_main.py gui
```

Or make a fresh environment here (needs internet once, so do it on normal Wi-Fi, not the robot hotspot):
```powershell
python -m venv .venv
```
```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r requirements-yolo.txt
```

## 2. Bring your own values from the old folder (key, robot size, IP, edited maps)
```powershell
.\.venv\Scripts\python.exe nav_main.py import-old "C:\path\to\OLD project folder"
```
It only reads the old folder. It copies `config/mentorpi.key`, your edited maps and your old console settings,
and writes your values from the old `config/navigation.toml` into `config/navigation.local.toml`.

## 3. Start the console
```powershell
.\.venv\Scripts\python.exe nav_main.py gui
```
(or double-click `START_CONSOLE.bat` in this folder: it does the same)
The console opens in its own window (Edge or Chrome). Keep the terminal open. If no window opens, click the
link the terminal prints. **Esc = STOP** at any time. Closing the window while the robot moves also stops it.

## The console at a glance
* **Top:** mode (Planner / Simulator / Robot), status chips (link, state, drift, mission), **STOP**.
* **Left tabs:** **Drive** (connect, start, goal, authorize, speed), **Map** (paint, save, layers, export),
  **Robot** (footprint, scale calibration, LiDAR correction), **Camera**, **Search**.
* **Middle:** the map. Wheel = zoom, drag = move, buttons on the right = fit, rotate, follow the robot.
  x runs left to right from 0 to 3 m, y bottom to top. Rotating only turns the picture and is not remembered.
  ENV2 and ENV3 are shown the way the real arenas stand (ENV2 turned 90° clockwise, ENV3 90° counter-clockwise
  compared with the data files; the files themselves are unchanged).
* **Right:** live camera with YOLO boxes, survivor list, activity log, system health.

## Try it without the robot (Simulator)
1. Top bar: **Simulator**. Drive tab: **Start simulator**.
2. Tool **Start** (left of the map): click where the robot stands and drag to point where it faces.
   Press **Confirm pose**.
3. Tool **Goal**: click the map. The orange line is the A* route. (There is no goal until you set one.)
4. Press **AUTHORIZE**, read the question, confirm. Esc stops.
5. Camera tab: **Camera + YOLO on**, then **Place / remove blocks** and click the map to drop green or red
   test blocks. Tick **Search for survivors at the goal** (Drive tab) to try the search mission.

## The real robot (same steps as before)
1. Robot on, laptop on the robot's Wi-Fi, bridge started on the robot (see `UGV_SETUP.md`, unchanged).
2. Top bar: **Robot**. Drive tab: IP `192.168.149.1`, **Connect**.
3. Place the robot, set **Start** + heading, **Confirm pose**. Set the **Goal**. **AUTHORIZE**.

## What is different from the old console (each can be switched back)
| New | Where to switch it |
|---|---|
| Routes are straightened (fewer stop-and-turn corners). Never closer to walls than A*'s own rules allow. | Robot tab: *Smooth route* |
| A* answers about 50x faster (same routes, checked by tests). | `config/navigation.toml`: `engine = "original"` |
| Camera tilt is measured from the known 10 cm block height, so pins are accurate without manual calibration. | Camera tab: *Auto tilt from block size* |
| Boxes that cannot be a block (for example a long barrier) are not put on the map. | Camera tab: *Ignore boxes that cannot be a block* |
| **Re-localize (LiDAR)**: one click corrects the position from the walls when the drift limit stops you. | Drive tab button |
| Continuous LiDAR position correction (beta, off by default). | Robot tab |
| Obstacles found by the LiDAR are a separate purple layer; you choose to keep or clear them. | Map tab |
| Driving behaviour (drive and steer together), speeds, safety zones, the robot bridge: **unchanged**. | - |

Details and test evidence: `WHAT_IS_NEW.md`.

## Maps, calibration, search
* **Edit map:** Map tab, **Paint wall** / **Erase**, drag on the map, **Save map**. Saved maps load
  automatically; the original `data/*.npy` files are never changed.
* **Scale calibration:** Robot tab (same procedure as before: Confirm pose, Start X run, drive, enter the real
  end position, Finish run). **Reset to 1.000** removes old values.
* **Search for survivors:** Camera on, set a goal, tick *Search for survivors at the goal*, **AUTHORIZE + SEARCH**
  (or **Search from here** without a goal). The robot goes to the **first green object it sees** (also on the
  way to the goal), stops at the distance you chose, faces it and stands still for 3 s: the survivor is
  **secured**. If it knows another green object it goes there next; otherwise the **mission is complete** and
  ends by itself. Nothing green yet: it keeps looking around and exploring. Search tab: *How close to the green
  object* (cm), *Stand there before "secured"* (s), and a switch if you do not want the mission to end by itself.
  STOP, link loss and position loss pause it: fix the cause, then **RESUME MISSION**. **Export** saves the list.
* **Fewer false detections** (Camera tab, all on by default): a box must be block-shaped (a round or square thing
  is not a block), must have the size of a 10 cm block at its floor distance (a door is far too tall), must
  really contain its colour, and needs 60 % confidence in 4 frames. One block = one pin: pins of the same colour
  closer than 18 cm are joined.

## Getting updates with Git (instead of copying zip files)
One time, in a folder of your choice (Git for Windows must be installed):
```powershell
git clone https://github.com/YOUR-NAME/UGV-AStar-YOLOV5.git
```
Then every time there is an update, inside that folder:
```powershell
git pull
```
(or double-click `UPDATE.bat`). Your own files are never uploaded and never overwritten by an update: the key
`config/mentorpi.key`, your values `config/navigation.local.toml`, your console settings and `data/edited/`.

## If something goes wrong
* **"The shared key file ... is missing":** run the `import-old` command above, or copy `config\mentorpi.key`
  from the old folder into this folder's `config`.
* **"YOLO is not available":** install `requirements-yolo.txt` (see setup), or use the old folder's Python.
* **Drift limit stops the robot:** press **Re-localize (LiDAR)**, then plan / resume. If the *LiDAR fit* bar is
  low, the robot is not where the console thinks: set Start and **Confirm pose** by hand.
* **"the robot is NOT really moving":** the robot received commands but the LiDAR view did not change. Check the
  battery and motor power on the robot.
* **Window closed by accident:** open the link from the terminal again (within 25 s), or start the console again.

## Folder map
| Folder / file | Runs on | Purpose |
|---|---|---|
| `ugv_planner/` | laptop | your A* planner, maps, original GUI (unchanged copy) |
| `ugv_nav/` | laptop | localization, path following, safety, YOLO mapping, search mission, fast planner, smoothing |
| `ugv_console/` | laptop | the new console: `app.py` (logic), `server.py` (this computer only), `web/` (the page) |
| `ugv_bridge/` | robot | unchanged copy of the bridge already on your MentorPi |
| `third_party/wro_yolo/` | laptop | your teammate's YOLO package (unchanged) |
| `config/navigation.toml` | laptop | robot size, speeds, limits (shipped values) |
| `config/navigation.local.toml` | laptop | YOUR values; win over the shipped file, untouched by updates |
| `tests/` | laptop | `python -m unittest discover -s tests` |

The original planner program still runs as before: `python main.py`.
