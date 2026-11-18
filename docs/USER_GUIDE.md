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
2. **Corner start**: click a corner (or use the **Start** tool: click where the robot stands, drag for the heading), then **Confirm pose**.
3. Tool **Goal**: click the map. The orange line is the A\* route.
4. **AUTHORIZE**, read the question, confirm.
5. Camera tab: **Camera + YOLO** on, then **Place / remove blocks** to drop green or red test blocks.

## Real robot
1. Robot on, laptop on the robot's Wi-Fi, bridge started on the robot ([Robot setup](ROBOT_SETUP.md)).
2. Top bar **Robot**, Drive tab: enter the robot's IP, **Connect**.
3. Tell the console where the robot is: see the next section.
4. Set the **Goal**, then **AUTHORIZE**.

## Where is the robot?
The picture is only right if the console knows where the robot really stands. Three ways, best first:

| Way | What you do | Needs |
|---|---|---|
| **Corner start** | Stand the robot square in a corner, **one block length (10 cm)** away from both walls. Click that corner (click again to turn the robot), then **Confirm pose**. | nothing: a block is your ruler |
| **Find robot** | Put the robot anywhere and press **Find robot**. After the first few sessions it is also found by itself when you connect or carry it. | one earlier corner start in this room |
| **Start pin** | Tool **Start**: click, drag for the heading, **Confirm pose**. | a good eye (least accurate) |

**Adjust** (the small arrows) moves the shown position by 1 cm or turns it by 2°. Typing coordinates works too.

### Why the LiDAR looks at the room
On the MentorPi the LiDAR sits higher than the arena walls, so it never sees the arena. It does see the room
around it (walls, furniture). The room does not move, so the console draws its outline (grey squares around
the map) and follows the robot's true movement in it:

* **Live LiDAR tracking** keeps the position right while driving, without the drift of the wheels. The tag in
  the *Robot position* card shows *tracking*.
* Carry the robot somewhere else: it is found again within a few seconds.
* The first time on a new robot the tag shows *watching*: the console first learns from the robot's own motion
  where the LiDAR is mounted (it needs about half a metre of straight driving and a few turns). Until that is
  verified, nothing is corrected and the position comes from odometry, as before.
* The LiDAR reaches 3 m. A place is only found where the room has been drawn, so let the robot drive through the
  arena once after a corner start. If the console is not sure, it marks the place and asks you to click it.

If your LiDAR does see the arena walls (another robot or higher walls): Robot tab, *The LiDAR sees: arena walls*.

## Real field size
The map files draw the floor **2.76 × 2.76 m** between the walls. If your real field is another size, the robot
ends up somewhere else than the picture shows (8 % wrong is 24 cm at the far wall).

1. Measure the inner width (X) and depth (Y) between the outer walls with a tape.
2. Robot tab, **Real field size**: type both numbers, **Apply**. The map is stretched to the real size; all
   coordinates are then real metres. **Use file size** goes back.

Without a tape: after one drive through the arena, stand the robot in two opposite corners like for a corner
start and click each corner under **Measure**. The distance between them comes from the LiDAR, not the wheels.

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

## Mission video
Every search mission is filmed with the robot's camera and the video is saved on this laptop when the mission
ends. Nothing is stored on the robot.

| | |
|---|---|
| **Starts** | with **AUTHORIZE + SEARCH** or **Search from here**, so the drive to the goal is in it too (a plain drive without *Search* is not filmed) |
| **Ends** | when the mission is complete or you press **Stop mission** (2 more seconds are filmed). **STOP** only pauses the mission, so the film keeps running. **Disconnect** saves it at once |
| **Where** | `Videos\UGV missions` in your user folder, for example `mission_2026-11-17_14-03-22_ENV3.mp4`. **Search tab > Mission video > Open video folder**; the folder can be changed there |
| **Picture** | Top: time since the mission started, what the robot is doing, survivors found / secured. Bottom: the latest mission event for a few seconds, date and time. The YOLO boxes: green / red, grey = ignored (with the reason). Switch *Draw detections and mission info* off for the plain camera picture |

Good to know:
* A red **REC** tag with the running time shows on the *Live camera* card while the video is recorded.
* The laptop films what the camera sends (about 8 pictures a second) and saves 10 frames a second, so the video is
  exactly as long as the mission. If no new picture arrives, the last one is repeated and the picture says how old it is.
* The video is made by a separate low-priority program that starts and ends with the mission, so filming cannot slow
  down the robot's control. (You may see a second `python.exe` in the Task Manager while a mission is recorded.)
* Camera + YOLO must be on (the search needs them anyway). Any player works (VLC, Windows Movies & TV). If this computer
  cannot write MP4 it writes AVI instead.
* It will not start with less than 1 GB free on the disk, it stops and keeps the video if less than 300 MB are left, and
  one video is at most 60 minutes. Why a video was not saved is written in the Activity log and under the switch.
* The videos can be large (about 5-15 MB a minute with a real camera) and may show people in the room: they stay on
  this laptop and are not part of the Git repository.

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
* **Drift limit:** without live tracking the robot stops when its estimated position error passes 15 cm.
  Press **Re-localize** or **Find robot**, or do a corner start again. With live tracking this does not build up.
* **Ignore drift alerts** (Drive tab, *Robot position* card): when it is on, "drift too high" no longer stops the
  robot, refuses to plan a route or pauses a mission. The estimate is still counted and shown (the Drift bar turns
  amber and says *ignored*, the map keeps the uncertainty ring). Everything else still stops the robot: **STOP**,
  the LiDAR stop zone, link loss, odometry that stops arriving or jumps, and a missing start pose. The console asks
  you to confirm before it switches on and remembers the choice. Use it only when you have checked that the shown
  position is right (the *Room fit* bar is the quick check), because nothing warns you any more if it is not.
* **Scale calibration** (Robot tab) is only needed without live tracking: Confirm pose, **Start X run**, drive,
  enter the real end position, **Finish run**. **Reset to 1.000** removes old values.
* **Forget room / Forget mounting** (Robot tab): start learning again, for example in another room.

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
| The mission was not filmed / no video was saved | Read the note under *Mission video* (Search tab) or the Activity log: camera off, disk almost full, folder not writable, or no camera pictures arrived |
| Drift limit stops the robot | **Find robot** or a corner start. Check that *Live LiDAR tracking* is on (Robot tab). Or switch on **Ignore drift alerts** (Drive tab, *Robot position*) |
| "Robot not found" | The LiDAR has not drawn that part of the room: corner start, then one drive through the arena |
| The robot is somewhere else than the picture shows | Corner start again; check **Real field size**; **Forget room** if furniture was moved a lot |
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
