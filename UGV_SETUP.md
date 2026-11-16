# MentorPi (UGV) setup: step by step

> **Version 2:** nothing changes on the robot. If the bridge from the old project is already on your
> MentorPi, keep it as it is and skip this file; the `ugv_bridge/` folder here is the same program.
> Where a step says "operator console", that is now `nav_main.py gui` (the web console).

The robot only runs the small `ugv_bridge` program. All planning stays on the laptop.
Nothing here installs services, changes HiWonder software, or touches firmware.
**No step in Part 1 moves the motors.**

Replace `<PI_IP>` with the robot's Wi-Fi IP address and `<USER>` with its login user
(the account you already used for your connectivity test).

## Part 1: no motion

### 1. Check you can log in to the Pi (laptop, PowerShell)
```powershell
ssh <USER>@<PI_IP>
```
Type `exit` to leave.

### 2. Copy the bridge folder to the Pi (laptop, PowerShell, inside the `ugv_astar` folder)
```powershell
ssh <USER>@<PI_IP> "mkdir -p ~/edge_bridge"
scp -r ugv_bridge <USER>@<PI_IP>:~/edge_bridge/
```

### 3. Create the shared secret key and copy it (laptop)
```powershell
.\.venv\Scripts\python.exe nav_main.py genkey --out config\mentorpi.key
scp config\mentorpi.key <USER>@<PI_IP>:~/edge_bridge/ugv_bridge/config/
```
Then lock the key down on the Pi (the bridge refuses a key other users can read):
```bash
chmod 600 ~/edge_bridge/ugv_bridge/config/mentorpi.key
```
Never share or commit this file. Laptop and Pi must hold the **same** file.

### 4. Discover the robot's software (Pi, read-only)
```bash
cd ~/edge_bridge
bash ugv_bridge/tools/discover_mentorpi.sh 2>&1 | tee discovery.txt
```
Copy `discovery.txt` back to the laptop and send it for review:
```powershell
scp <USER>@<PI_IP>:~/edge_bridge/discovery.txt .
```
It tells us the ROS 2 version, whether HiWonder runs ROS inside Docker, and the real topic names for
motor commands, odometry, IMU and LiDAR. **Do not use the `ros2` backend until these are confirmed.**

### 5. Create the bridge config (Pi)
```bash
cd ~/edge_bridge/ugv_bridge/config
cp bridge_config.example.json bridge_config.json
nano bridge_config.json
```
The robot size (0.11 / 0.09 m half length/width) is already filled in. The `topics` section gets
updated after step 4.

### 6. Communication test with the dry-run backend (never touches motors)
Pi:
```bash
cd ~/edge_bridge
python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend dryrun --verbose
```
Laptop (edit `config/navigation.toml`: `ugv_host = "<PI_IP>"`), then:
```powershell
.\.venv\Scripts\python.exe nav_main.py check-connection --host <PI_IP>
```
Expected: `Link OK`, an RTT of a few to tens of ms, and `UGV armed: False`.
If it fails: check that the Pi firewall allows TCP port 8765, that both devices are on the same Wi-Fi, and that the key files match.
Stop the bridge with Ctrl+C.

### 7. Real sensors, still no motion (after topics are confirmed)
Run the bridge with `--backend ros2` in the same environment as HiWonder's ROS nodes (inside their
Docker container if step 4 shows Docker). Run `check-connection` again: odometry, IMU and LiDAR should
now show real values. Push the robot by hand a few cm and run it again; odometry should change.

## Part 2: motion (each stage needs explicit approval first)

| Stage | What you do |
|---|---|
| C: wheels off ground | Robot on a box/stand, wheels free. `nav_main.py motor-test --host <PI_IP>`, type `WHEELS OFF GROUND`, confirm each phase with Enter. |
| D: small floor moves | Clear, level floor. Compare the distance driven with a tape measure. |
| E: one waypoint | Console in **Physical MentorPi** mode, goal ≤ 0.5 m away in clear space. |
| F, G, H | Short A* route, arena route, then safety tests (pull Wi-Fi, block the route, press STOP). |

Keep a hand near the robot's power switch during every motion stage. Laptop STOP = red button or Esc.
The bridge also stops the motors by itself within 0.3 s if the laptop goes silent.

## Things to avoid
- Don't run HiWonder's phone app or joystick control at the same time as the bridge (two controllers).
- Don't add the bridge to startup services. Start it by hand when testing.
- Don't change HiWonder's own config or firmware.


## Update for camera + YOLO (bridge v2)
The bridge can now stream the camera to the laptop (only when the console asks for it).
1. Laptop (PowerShell, in `ugv_astar`): copy the updated bridge into the shared folder.
   Your `bridge_config.json` and `mentorpi.key` on the robot are not overwritten.
   ```powershell
   scp -r ugv_bridge pi@192.168.149.1:~/docker/tmp/edge_bridge/
   ```
2. Robot (purple terminal): stop the running bridge (Ctrl+C) and start it again:
   ```bash
   cd ~/shared/edge_bridge
   python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend ros2 --verbose
   ```
   It should print `camera: /ascamera/camera_publisher/rgb0/image (sensor_msgs/msg/Image)`.
   (HiWonder's `image_compressed` topic is black-and-white, so the bridge uses the raw colour topic and
   compresses it itself. This needs OpenCV inside the container: check with `python3 -c "import cv2"`.)
The camera topic defaults to the MentorPi topic. If needed, set `"camera"` and `"camera_info"` under
`topics` in `bridge_config.json`.
