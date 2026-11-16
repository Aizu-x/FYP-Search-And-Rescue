#!/usr/bin/env bash
# READ-ONLY discovery of the MentorPi software environment.
# Publishes nothing, starts/stops nothing, installs nothing. Safe to run with the robot powered.
# Usage (on the Pi):  bash discover_mentorpi.sh 2>&1 | tee mentorpi_discovery.txt
# If ROS 2 lives inside a HiWonder Docker container, run it again inside that container:
#   docker exec -it <container> bash   then   bash /path/to/discover_mentorpi.sh
# Note: no 'set -u' -- ROS setup.bash files reference unset variables.
section() { printf '\n===== %s =====\n' "$1"; }

section "OS / hardware"
cat /etc/os-release 2>/dev/null; uname -a; cat /proc/device-tree/model 2>/dev/null; echo
python3 --version 2>&1

section "Network addresses"
hostname -I 2>/dev/null; ip -brief addr 2>/dev/null

section "Docker (read-only listing)"
if command -v docker >/dev/null 2>&1; then
  docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Networks}}' 2>&1
  for c in $(docker ps --format '{{.Names}}' 2>/dev/null); do
    echo "--- $c network mode:"; docker inspect -f '{{.HostConfig.NetworkMode}}' "$c" 2>&1
  done
else
  echo "docker not installed"
fi

section "Startup services mentioning ros/hiwonder/robot (listing only)"
systemctl list-units --type=service --no-pager 2>/dev/null | grep -Ei 'ros|hiwonder|robot|start|app' || echo "none matched"
ls /etc/systemd/system 2>/dev/null | grep -Ei 'ros|hiwonder|robot|start' || true
crontab -l 2>/dev/null | grep -v '^#' || true

section "ROS 2 environment"
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW=${RMW_IMPLEMENTATION:-<unset>}"
ls /opt/ros 2>/dev/null
echo "--- ROS-related lines in shell startup files (HiWonder sets domain/workspace here):"
grep -H -i -E "ros|domain|rmw|source|export" /root/.bashrc /root/.zshrc /home/*/.bashrc /home/*/.zshrc 2>/dev/null | grep -v "^\s*#" | head -40
if ! command -v ros2 >/dev/null 2>&1; then
  for d in /opt/ros/*/setup.bash; do [ -f "$d" ] && source "$d" && echo "sourced $d" && break; done
fi
for ws in ~/ros2_ws/install/setup.bash /home/*/ros2_ws/install/setup.bash ~/*/install/setup.bash /home/*/*/install/setup.bash; do
  if [ -f "$ws" ]; then echo "workspace found: $ws"; source "$ws" 2>/dev/null && echo "sourced $ws"; fi
done
# Use HiWonder's ROS_DOMAIN_ID if it is set in a startup file and not already set here.
if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  dom=$(grep -h -E "^\s*export\s+ROS_DOMAIN_ID=" /home/*/.bashrc /home/*/.zshrc /root/.bashrc /root/.zshrc 2>/dev/null | tail -1 | sed -E 's/.*=//; s/["'"'"' ]//g')
  if [ -n "$dom" ]; then export ROS_DOMAIN_ID="$dom"; echo "using ROS_DOMAIN_ID=$dom from startup files"; fi
fi
echo "effective: ROS_DISTRO=${ROS_DISTRO:-<unset>} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW=${RMW_IMPLEMENTATION:-<unset>}"
if command -v ros2 >/dev/null 2>&1; then
  python3 -c "import rclpy; print('rclpy import OK')" 2>&1
  section "ROS 2 nodes"
  timeout 10 ros2 node list 2>&1
  section "ROS 2 topics with types"
  timeout 10 ros2 topic list -t 2>&1
  section "Candidate topic details (publishers/subscribers/QoS)"
  for t in /controller/cmd_vel /cmd_vel /odom /odom_raw /odometry/filtered /imu /imu/data /imu_corrected /scan /scan_raw; do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$t"; then
      echo "--- $t"; timeout 8 ros2 topic info -v "$t" 2>&1 | head -40
    fi
  done
  section "Message rates (5 s each, receive only)"
  for t in /odom /imu /scan; do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$t"; then
      echo "--- $t"; timeout 6 ros2 topic hz "$t" 2>&1 | tail -3
    fi
  done
  section "One odom sample (frame ids)"
  timeout 5 ros2 topic echo --once /odom 2>&1 | head -25
  section "TF frames (receive only)"
  timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_frame 2>&1 | head -12 || true
else
  echo "ros2 CLI not found on host (may be inside Docker; see above)"
fi
echo; echo "Discovery complete. Nothing was published or changed."
