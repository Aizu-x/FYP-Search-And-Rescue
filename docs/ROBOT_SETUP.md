# Robot setup (HiWonder MentorPi-M1)

The robot only runs the small `ugv_bridge` program. All planning stays on the laptop. Nothing here
installs services, changes HiWonder software or touches firmware.

Replace `<PI_IP>` with the robot's IP address and `<USER>` with its login user.

## Part 1: no motion

**1. Copy the bridge to the robot** (laptop, inside the project folder)
```bash
ssh <USER>@<PI_IP> "mkdir -p ~/edge_bridge"
```
```bash
scp -r ugv_bridge <USER>@<PI_IP>:~/edge_bridge/
```
If ROS 2 runs inside HiWonder's Docker container, copy into the folder that is shared with the container.

**2. Create the shared key and copy it**
```bash
python nav_main.py genkey --out config/mentorpi.key
```
```bash
scp config/mentorpi.key <USER>@<PI_IP>:~/edge_bridge/ugv_bridge/config/
```
On the robot:
```bash
chmod 600 ~/edge_bridge/ugv_bridge/config/mentorpi.key
```
Laptop and robot must hold the **same** file. Never share or commit it.

**3. Find the robot's ROS topics** (robot, read-only)
```bash
bash ~/edge_bridge/ugv_bridge/tools/discover_mentorpi.sh 2>&1 | tee discovery.txt
```

**4. Create the bridge config** (robot)
```bash
cd ~/edge_bridge/ugv_bridge/config && cp bridge_config.example.json bridge_config.json
```
Fill in the measured robot size and the topic names from step 3.

**5. Communication test without motors**

Robot:
```bash
cd ~/edge_bridge && python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend dryrun --verbose
```
Laptop:
```bash
python nav_main.py check-connection --host <PI_IP>
```
Expected: `Link OK`, a round trip of a few ms, `UGV armed: False`.
If it fails: same Wi-Fi? TCP port 8765 open? Same key file on both sides?

**6. Real sensors, still no motion**

Start the bridge with `--backend ros2` in the same environment as HiWonder's ROS nodes (inside their
Docker container if they use one) and run `check-connection` again. Odometry, IMU and LiDAR now show
real values.

## Part 2: motion (one stage at a time)

| Stage | What you do | Pass when |
|---|---|---|
| Wheels off the ground | Robot on a stand. `python nav_main.py motor-test --host <PI_IP>` | Wheels turn the right way, STOP halts them, they stop by themselves within 0.35 s when commands end |
| Small floor moves | Clear, level floor. Compare the distance driven with a tape measure | Use the result for the scale calibration |
| One waypoint | Console in **Robot** mode, goal at most 0.5 m away | Arrives within 5 cm |
| Arena route | A full A\* route on a verified-clear map | Arrives without contact |
| Safety | Pull the Wi-Fi, close the console, block the route, press STOP | The robot stops every time and never resumes on its own |

Keep a hand near the robot's power switch during every motion stage.

## Daily start
Robot:
```bash
python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend ros2 --verbose
```
Laptop: start the console, **Robot** mode, **Connect**.

## Things to avoid
- Do not run HiWonder's phone app or joystick control at the same time as the bridge.
- Do not add the bridge to startup services. Start it by hand.
- Do not change HiWonder's own configuration or firmware.

## Reference: the MentorPi used in this project
| Item | Value |
|---|---|
| Computer | Raspberry Pi 5, ROS 2 Humble inside Docker container `MentorPi` |
| Motor command topic | `/controller/cmd_vel` (Twist) |
| Sensors | `/odom`, `/imu`, `/scan_raw` (MS200 LiDAR), `/ascamera/camera_publisher/rgb0/image` |
| Body | 22 × 18 cm, planning radius 0.143 m + 0.02 m margin |
