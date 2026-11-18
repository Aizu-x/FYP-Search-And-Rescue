# UAV section: the Tello drone

The first button in the top bar. Fly a Tello from the laptop, watch its camera, see battery and altitude,
and keep the footage. It comes before the robot work: scout with the drone, then switch to the UGV modes.

<img src="images/console_uav.png" alt="The UAV section: live drone camera, battery, altitude, flight keys" width="860">

> **Status.** Everything here was built and tested with the **built-in simulated drone**. It has **not yet
> been flown with a real Tello**. Read [First real flight](#first-real-flight) before you try.

## Try it without a drone (2 minutes)

1. Top bar: **UAV**.
2. Switch **Simulated drone** on, press **Start simulated drone**. A picture of a drawn room appears.
3. Press **TAKE OFF** and confirm.
4. Hold **W A S D** and the **arrow keys**. Let go: it hovers.
5. **Esc** (or **LAND**) lands it. The flight video is saved by itself.

## Keys

| Key | The drone |
|---|---|
| **W** / **S** | flies forward / back |
| **A** / **D** | flies left / right |
| **↑** / **↓** | goes up / down |
| **←** / **→** | turns left / right |
| **Esc** | lands |

- The drone moves only while a key is held. Several keys together work (for example W + →).
- **Stick strength** sets how hard a key pushes: *Gentle 20*, *Normal 35*, *Fast 60*. Start gentle.
- The keys on the picture can be pressed with the mouse too.
- Keys do nothing while you type in a text field or a dialog is open.

## What you see

| Reading | Meaning |
|---|---|
| **Battery** | Percent. Yellow below 25 %, red below 20 % (no take-off below 20 %). |
| **Altitude** | Height above the take-off point. |
| **Distance to ground** | The range sensor under the drone. |
| **Flight time**, **Heading**, **Temperature** | As the drone reports them. |
| **Video** | Pictures per second and size of the camera stream. |
| **Sticks** | What is being sent right now: left-right, forward, up, turn (-100 to 100). |

## Fly the real Tello

1. Charge it, put the propeller guards on, choose a room with free space.
2. Switch the Tello on. On the laptop, join its Wi-Fi (the name starts with **TELLO-**).
3. Console: **UAV**, switch **Simulated drone** off, press **Connect**.
4. The first time, Windows asks about Python and the firewall: tick **Private and Public**, then *Allow*.
5. Check: battery shown, altitude 0.0 m, live picture.
6. **TAKE OFF**, read the question, confirm.
7. Fly. **Esc** or **LAND** brings it down.

The drone's Wi-Fi is not the robot's network. Switch the laptop back to the robot's network before you use
the **Robot** mode.

## Footage

- **Record every flight** (on by default): filming starts at take-off and the video is saved 2 seconds after
  the landing.
- **Record** films by hand, also on the ground. **Photo** saves one picture.
- Everything goes to `Videos\UAV footage` (change it under *Folder*; **Open footage folder** shows it).
- Files: `flight_<date>_<time>.mp4` and `photo_<date>_<time>.jpg`. Full camera size, nothing drawn on it.
- A video that is still being written is called `..._recording.mp4`. If the console closes or crashes it is
  finished and saved anyway.

## Safety rules built in

- **Take-off needs two steps**: the button, then a confirmation. Nothing starts the motors by itself.
- **LAND never waits**, works in every state and is repeated until the drone confirms. Esc lands too.
- **Let go = hover.** The laptop sends the stick values 20 times a second and sends zeros when no key is held.
- **Page not reporting its keys for 0.5 s**: hover. **Page gone for 5 s**: land.
- **Nothing heard from the drone for 2.5 s**: sticks to zero. **For 5 s**: land is sent.
- **Console program closed or crashed while flying**: land is sent (by the console, or by its video helper).
- **One machine at a time**: no take-off while the robot drives, no robot AUTHORIZE while the drone flies,
  and the UAV section cannot be left while the drone is in the air.
- **Cut motors (emergency)** asks first: the drone falls. Only for a drone that is caught or flying away.
- The Tello itself lands when it gets no command for 15 seconds (its own safety feature).
- Only messages from the drone's own address are accepted.

## First real flight

Not confirmed yet, because no real Tello was connected during development:

| To check | How |
|---|---|
| **Key directions** | Take off, set *Gentle 20*, tap each key once. The directions follow the convention of the common Tello libraries; the official document does not state them. If one is reversed, land and report it: it is a one-line change in `uav/tello.py` (`sticks_from_keys`). |
| **Losing Wi-Fi while a key is held** | Unknown whether the drone keeps its last movement until its 15 s safety landing. So: stay close, fly gently, keep the path clear. |
| **Live picture** | Should appear a few seconds after Connect. How far it lags behind reality with the real camera is not measured yet (one picture, about 0.07 s, with the simulated one). If it does not appear see below. |
| **Readings** | Battery and altitude should match the drone (altitude 0 on the ground, rising at take-off). |
| **Landing confirmation** | The console waits for the drone's "ok". If the page still says *Flying* after the drone is down, press LAND once. |

## If something does not work

| Problem | Fix |
|---|---|
| *"The drone did not answer"* | The laptop is not on the Tello's Wi-Fi, or the Tello is off. The message says which address the laptop is using. Windows sometimes jumps back to your normal Wi-Fi because the Tello's has no internet: join **TELLO-…** again. A brand-new Tello may first have to be started once with the official Tello app. |
| Connected, but no picture and/or no battery | Windows Firewall blocks Python: Windows Security > Firewall > *Allow an app* > Python > tick Private **and** Public. Reconnect. |
| *"Port 11111 / 8890 is used by another program"* | Close other Tello programs (and a second console). |
| Keys do nothing | Is the state *Flying*? Click the picture once. Are you typing in a text field? |
| No take-off | The hint under the button says why: battery below 20 %, no battery reading, robot moving. |
| Picture freezes | Too far from the drone, or Wi-Fi interference. The console restarts the video by itself twice. |

## How it works (short)

```
 page (browser)  --keys held-->  console program  --"rc a b c d", "takeoff", "land" (UDP 8889)-->  Tello
                 <--pictures---  video worker     <--H.264 video (UDP 11111)------------------------
                 <--readings---  console program  <--status text (UDP 8890)-------------------------
```

- `uav/tello.py`: the link (text commands of the official Tello SDK 1.3, status, safety rules).
- `uav/video.py`: a separate process that receives, decodes, previews and records the video.
- `uav/sim.py`: the simulated drone. It speaks the same protocol and sends real H.264 video, so the same
  code runs with it as with the real one. How it *flies* is made up.
- `ugv_console/uav_console.py` and `web/js/uav.js`: the commands and the page.
