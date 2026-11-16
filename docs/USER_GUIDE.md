# User guide

## Start the console
```bash
.venv\Scripts\python.exe nav_main.py gui
```
On Windows you can double-click `START_CONSOLE.bat` instead. The console opens in its own browser window.
Keep the terminal open. If no window opens, click the link the terminal prints.

**Esc = STOP** at any time. Closing the window while the robot moves also stops it.

## The console at a glance
| Area | What is there |
|---|---|
| **Top bar** | Mode (Planner / Simulator / Robot), status chips (link, state, drift, mission), **STOP** |
| **Left tabs** | **Drive** (connect, start, goal, authorize, speed) · **Map** (paint, save, layers) · **Robot** (size, calibration) · **Camera** · **Search** |
| **Middle** | The map. Wheel = zoom, drag = move. x runs left to right 0-3 m, y bottom to top |
| **Right** | Live camera with YOLO boxes, survivor list, activity log, system health |

There is no goal until you set one: choose the **Goal** tool and click the map.

## Three modes
| Mode | Use it for |
|---|---|
| **Planner** | Plan and play back A\* routes on the map. No robot. |
| **Simulator** | The whole system with a simulated robot, LiDAR and camera. Safe place to try everything. |
| **Robot** | The real MentorPi. |

## Simulator, step by step
1. Top bar **Simulator**, Drive tab **Start simulator**.
2. Tool **Start**: click where the robot stands, drag to set the heading, **Confirm pose**.
3. Tool **Goal**: click the map. The orange line is the A\* route.
4. **AUTHORIZE**, read the question, confirm.
5. Camera tab: **Camera + YOLO** on, then **Place / remove blocks** to drop green or red test blocks.

## Real robot
1. Robot on, laptop on the robot's Wi-Fi, bridge started on the robot ([Robot setup](ROBOT_SETUP.md)).
2. Top bar **Robot**, Drive tab: enter the robot's IP, **Connect**.
3. Place the robot, set **Start** + heading, **Confirm pose**.
4. Set the **Goal**, then **AUTHORIZE**.

## Search mission
Camera on, tick **Search for survivors at the goal**, then **AUTHORIZE + SEARCH**
(or **Search from here** to start without a goal).

1. The robot drives to the goal and looks around in steps.
2. It goes to the **first green object it sees**, also when it sees one on the way.
3. It stops at the chosen distance, faces the object and stands still: the survivor is **secured**.
4. Another green object known: it goes there next. None: **mission complete**.
5. Nothing green yet: it explores floor the camera has not seen.

| Setting (Search tab) | Default |
|---|---|
| How close to the green object | 25 cm |
| Stand there before "secured" | 3 s |
| End the mission when secured and no other green is seen | on |

STOP, link loss and position loss **pause** the mission. Fix the cause, then **RESUME MISSION**.
**Export** in the Survivors card saves the list as CSV.

## Camera and YOLO
A detection becomes a pin on the map only if it can really be a block (all on by default, Camera tab):

| Check | Rejects |
|---|---|
| Block-shaped box | round or square things, long barriers |
| Size fits a 10 cm block at that distance | doors, posters, far-away coloured things |
| Box contains its colour | wrong-coloured detections |
| 60 % confidence, seen in 4 frames | flicker |

One block is one pin: same-colour pins closer than 18 cm are joined. The camera view shows on each box
why it was not used. The camera tilt is measured automatically from the known block height.

## Maps
* **Edit:** Map tab, **Paint wall** / **Erase**, drag on the map, **Save map**. Edits are stored in
  `data/edited/`; the original map files are never changed.
* **Environments:** ENV1, ENV2, ENV3 are shown the way the real arenas stand.
* **LiDAR finds** are a separate purple layer that you can keep or clear.

## Position and calibration
* **Drift limit:** the robot stops when its estimated position error passes 15 cm. Press
  **Re-localize (LiDAR)** to correct it from the walls, then continue.
* **Continuous LiDAR correction** (Robot tab, beta) does this automatically while driving.
* **Scale calibration** (Robot tab): Confirm pose, **Start X run**, drive, enter the real end position,
  **Finish run**. **Reset to 1.000** removes old values.

## Your own settings
| File | Purpose |
|---|---|
| `config/navigation.toml` | Shipped values: robot size, speeds, limits |
| `config/navigation.local.toml` | Your values. They win over the shipped file and are never touched by `git pull` |
| `config/mentorpi.key` | Shared secret with the robot. Never uploaded |

Coming from the older project folder? This copies your key, values and edited maps (it only reads the old folder):
```bash
.venv\Scripts\python.exe nav_main.py import-old "C:\path\to\old folder"
```

## If something goes wrong
| Message | What to do |
|---|---|
| "The shared key file ... is missing" | Copy `mentorpi.key` into `config/`, or run `import-old` |
| "YOLO is not available" | `pip install -r requirements-yolo.txt` (Python 3.11-3.13) |
| Drift limit stops the robot | **Re-localize (LiDAR)**. If *LiDAR fit* is low, set Start and **Confirm pose** by hand |
| "the robot is NOT really moving" | Commands arrive but the LiDAR view does not change: check battery and motor power |
| Window closed by accident | Open the link from the terminal again within 25 s, or restart |

## Other commands
```bash
python nav_main.py check-connection --host <robot IP>
```
```bash
python nav_main.py sim-demo --env ENV1
```
```bash
python -m unittest discover -s tests
```
`python main.py` still opens the original stand-alone A\* planner window.
