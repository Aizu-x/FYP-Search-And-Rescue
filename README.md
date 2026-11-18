<div align="center">

# UGV A\* + YOLOv5 · Search & Rescue Console

**A ground robot that plans its own route with A\*, sees coloured objects with YOLOv5,<br>and drives to the survivor, controlled from one modern console on a laptop.<br>A Tello drone scouts first, flown from the same console.**

![Python](https://img.shields.io/badge/Python-3.11--3.13-3776AB?logo=python&logoColor=white)
![Planner](https://img.shields.io/badge/Planner-A*-f59e0b)
![Vision](https://img.shields.io/badge/Vision-YOLOv5n%20ONNX-22c55e)
![Robot](https://img.shields.io/badge/Robot-HiWonder%20MentorPi--M1-3b82f6)
![ROS 2](https://img.shields.io/badge/Robot%20side-ROS%202%20Humble-22314E?logo=ros&logoColor=white)
![UAV](https://img.shields.io/badge/UAV-Tello%20drone-ef4444)
![Tests](https://img.shields.io/badge/Tests-301%20passing-16a34a)

<img src="docs/images/demo.gif" alt="The robot drives to the goal, spots a green object, approaches it, holds 3 seconds and marks the survivor secured" width="860">

*Simulator run (3.5× speed): drive to the goal, look around, spot the green object, approach, hold 3 s, **survivor secured**, mission complete.*

</div>

---

## What it does

| | |
|---|---|
| 🚁 **UAV first (Tello drone)** | Before the robot drives, fly a Tello from the same console: live camera picture, **battery and altitude**, **W A S D** to move, **arrow keys** for height and turning, two-step take-off, **Esc = land**. Every flight is filmed and saved as an MP4. A simulated drone is built in for practice. |
| 🗺️ **A\* path planning** | Shortest safe route on the arena's occupancy map (3 cm cells, stretched to the real size of your field). The robot's real size is respected, routes prefer open space and are straightened without ever getting closer to walls. A plan takes about 15 ms. |
| 👁️ **YOLOv5 vision** | A YOLOv5n model finds **green** (survivor) and **red** (obstacle) blocks in the robot's camera. Each box is projected onto the floor and pinned on the map; red blocks are avoided by A\*. |
| 🚑 **Search mission** | The robot drives to your goal, looks around, goes to the **first green object it sees**, stops at the distance you chose, holds 3 s and marks it **secured**. No other green in sight: mission complete. |
| 🎥 **Mission video** | Every search mission is filmed with the robot's camera, from the start to the end, and saved on the laptop as an MP4 with the time, what the robot is doing, survivors found and the YOLO boxes on the picture (the overlay can be switched off). |
| 📍 **Knows where it is** | No pin placed by eye: stand the robot in a corner one block length from both walls and click that corner, or press **Find robot**. You can still nudge the position by hand. |
| 🛰️ **LiDAR tracking** | The LiDAR sits higher than the arena walls, so it sees the room instead. The console draws the room's outline and follows the robot's true movement in it: no wheel drift, and a robot that is carried elsewhere is found again. Tall unexpected objects stop the robot and are planned around. |
| 🖥️ **Modern console** | Smooth 60 fps map, live camera with detection boxes, survivor list, activity log, dark and light theme. Runs in your browser, on your laptop only. |
| 🛑 **Safety first** | Nothing moves without **AUTHORIZE**. **Esc = STOP**. The robot stops by itself if the link, the laptop or the console page goes away. |

## How it fits together

All thinking happens on the laptop. The robot only runs a small bridge that passes sensor data up and
checked velocity commands down.

```mermaid
flowchart LR
    subgraph Laptop["💻 Edge laptop: all decisions"]
        UI["Web console<br/>(ugv_console)"]
        NAV["Navigator<br/>A* · path following · mission"]
        YOLO["YOLOv5n<br/>green / red blocks"]
        UAV["UAV section<br/>keys → sticks · video · footage"]
        UI <--> NAV
        UI <--> UAV
        YOLO --> NAV
    end
    subgraph Robot["🤖 MentorPi-M1: executor"]
        BR["ugv_bridge<br/>watchdog · limits · LiDAR stop"]
        ROS["ROS 2 Humble<br/>motors · odometry · LiDAR · camera"]
        BR <--> ROS
    end
    NAV -- "velocity commands<br/>(signed, expire in 0.3 s)" --> BR
    BR -- "odometry · LiDAR · camera" --> NAV
    BR -- "camera frames" --> YOLO
    subgraph Drone["🚁 Tello drone: scout"]
        TELLO["Tello SDK<br/>camera · battery · height"]
    end
    UAV -- "take off · sticks · land<br/>(text over UDP)" --> TELLO
    TELLO -- "status · H.264 video" --> UAV
```

### The search mission

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Transit: AUTHORIZE
    Transit --> Scan: goal reached
    Scan --> Explore: nothing green
    Explore --> Scan: new viewpoint
    Transit --> Approach: green seen
    Scan --> Approach: green seen
    Explore --> Approach: green seen
    Approach --> Hold: at chosen distance
    Hold --> Secured: 3 s in position
    Secured --> Approach: another green known
    Secured --> [*]: mission complete
```

## Screenshots

| Planner: A\* route on ENV2 | Mission: approaching survivor S1 |
|---|---|
| <img src="docs/images/console_planner.png" alt="Planner mode with an A* route" width="430"> | <img src="docs/images/console_mission.png" alt="Search mission approaching a survivor" width="430"> |

| Live LiDAR tracking: the grey outline is the room | Mission complete: survivor secured |
|---|---|
| <img src="docs/images/console_tracking.png" alt="The room outline drawn by the LiDAR around the arena" width="430"> | <img src="docs/images/console_complete.png" alt="Mission complete, survivor secured" width="430"> |

**UAV section**: the drone's camera (here the built-in simulated drone above the arena), battery, altitude and the flight keys.

<img src="docs/images/console_uav.png" alt="UAV section: live drone camera, battery, altitude, flight keys" width="866">

## Quick start

You need **Python 3.11, 3.12 or 3.13** on Windows, macOS or Linux. No robot is needed to try it.

```bash
git clone https://github.com/Aizu-x/UGV_A-Star-YOLOV5.git
```
```bash
cd UGV_A-Star-YOLOV5
```
```bash
python -m venv .venv
```
```bash
.venv\Scripts\python.exe -m pip install -r requirements.txt -r requirements-yolo.txt
```
```bash
.venv\Scripts\python.exe nav_main.py gui
```

On macOS / Linux use `.venv/bin/python` instead of `.venv\Scripts\python.exe`.
On Windows you can also double-click **`START_CONSOLE.bat`**. To get the newest version later: `git pull`
(or double-click **`UPDATE.bat`**).

### Try it in 60 seconds (no robot)

1. Top bar: **Simulator**, then **Start simulator** (Drive tab).
2. **Corner start**: click a corner in the *Robot position* card, then **Confirm pose**.
3. **Camera** tab: switch **Camera + YOLO** on, then **Place / remove blocks** and drop a green block.
4. Tool **Goal**: click the map. Tick **Search for survivors at the goal**.
5. **AUTHORIZE + SEARCH**, confirm, and watch. **Esc** stops at any time.

### Try the drone in 60 seconds (no drone)

1. Top bar: **UAV**. Switch **Simulated drone** on, then **Start simulated drone**.
2. **TAKE OFF**, confirm.
3. Hold **W A S D** to move, **↑ ↓** for height, **← →** to turn. Let go: it hovers.
4. **Esc** lands. The flight video is in `Videos\UAV footage`.

Full instructions: **[User guide](docs/USER_GUIDE.md)** · Drone: **[UAV guide](docs/UAV.md)** · Real robot: **[Robot setup](docs/ROBOT_SETUP.md)**

## Project layout

| Folder | Runs on | What it is |
|---|---|---|
| `ugv_planner/` | laptop | The A\* planner, occupancy grid, collision and clearance checks |
| `ugv_nav/` | laptop | Localization, room-based LiDAR tracking, path following, safety, YOLO mapping, search mission |
| `ugv_console/` | laptop | The console: Python backend and the web page (`web/`) |
| `uav/` | laptop | The UAV section: link to the Tello, video receiver and recorder, simulated drone |
| `ugv_bridge/` | robot | The only program on the MentorPi: authenticated link, watchdog, ROS 2 topics |
| `third_party/wro_yolo/` | laptop | YOLOv5n model (ONNX) for green / red blocks |
| `data/` | laptop | Three arena maps (ENV1-3): occupancy grid + ground truth |
| `config/` | laptop | `navigation.toml`: robot size, speeds, limits |
| `tests/` | laptop | 301 automated tests |
| `docs/` | | Guides and technical notes |

## Safety design

- **Operator authorization**: every autonomous drive needs a two-step confirmation in the console.
- **Two independent layers**: the laptop checks the route, position confidence and LiDAR; the robot bridge
  checks speed limits and its own LiDAR stop zone, and is the only thing that writes to the motors.
- **Watchdog**: commands expire after 0.3 s. Wi-Fi loss, a closed laptop or a closed console page stops the robot.
- **Authenticated link**: every message is signed (HMAC-SHA256) with a shared key that never leaves your devices.
- **Position confidence**: if the estimated drift exceeds 15 cm, the robot stops and asks to be re-localized.
  You can switch this one alert off in the console (*Ignore drift alerts*, with a confirmation); STOP, the LiDAR
  stop zone, link loss and broken odometry keep stopping the robot either way.
- **No guessing**: the LiDAR may place the robot by itself only on a strong, unambiguous fit; otherwise you choose.
  Its corrections start only after it has verified, from the robot's own motion, how the LiDAR is mounted.
- **The drone**: take-off needs a confirmation, **Esc lands**, letting go of the keys means hover, and the drone is
  brought down when the page, the console program or the radio link goes away. One machine moves at a time.

## Project status

| Area | Status |
|---|---|
| A\* planning, smoothing, mission logic, vision filters, console | ✅ 301 automated tests + browser end-to-end runs |
| Full loop in the built-in simulator (real bridge code, simulated robot, LiDAR and camera) | ✅ working |
| Room-based LiDAR tracking, Find robot, real field size | ✅ in a simulated room · 🔧 not yet tried in a real one |
| Mission video | ✅ recorded and played back from simulated missions · 🔧 not yet tried with the real robot's camera stream |
| UAV section: flying with the keys, live picture, battery / altitude, footage | ✅ with the built-in simulated drone (same protocol, real H.264 video) · 🔧 **not yet flown with a real Tello** |
| Link to the real MentorPi (authentication, odometry, IMU, LiDAR received) | ✅ verified |
| Autonomous driving and YOLO on the real robot | 🔧 in progress, **not yet verified** |

Details: **[Testing](docs/TESTING.md)** · How each part works: **[How it works](docs/HOW_IT_WORKS.md)**

## Documentation

| Guide | For |
|---|---|
| [User guide](docs/USER_GUIDE.md) | Using the console: planner, simulator, real robot, search mission, settings |
| [UAV guide](docs/UAV.md) | The drone section: keys, readings, footage, first real flight, troubleshooting |
| [Robot setup](docs/ROBOT_SETUP.md) | Putting the bridge on the MentorPi and connecting, step by step |
| [How it works](docs/HOW_IT_WORKS.md) | Architecture, A\*, vision, mission, protocol, safety rules |
| [Testing](docs/TESTING.md) | What was tested, how, and what is still open |

## Credits

Final-year project by **Abbas Faisal Modhaffar**: A\* navigation, operator console and robot integration.
The YOLOv5n red / green block detector in `third_party/wro_yolo/` was trained by a project teammate.
Robot platform: HiWonder MentorPi-M1 (Raspberry Pi 5, ROS 2 Humble). Drone: Tello (Ryze / DJI), Tello SDK 1.3.
