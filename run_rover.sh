#!/bin/bash
set -e

echo "========================================="
echo "  ROVER SOFTWARE STACK STARTUP SCRIPT"
echo "========================================="

# -----------------------------
# 1. ENVIRONMENT
# -----------------------------
echo "[1/6] Setting ROS + DDS environment..."

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "✓ ROS_DISTRO: $ROS_DISTRO"
echo "✓ DDS: $RMW_IMPLEMENTATION"
echo

# -----------------------------
# 2. CLEAN OLD PROCESSES
# -----------------------------
echo "[2/6] Killing old rover processes..."

pkill -9 -f odometry_node || true
pkill -9 -f ekf_node || true
pkill -9 -f rplidar || true
pkill -9 -f slam_toolbox || true

sleep 1
echo "✓ Old processes cleared"
echo

# -----------------------------
# 3. START ODOMETRY NODE
# -----------------------------
echo "[3/6] Starting ODOMETRY node..."

ros2 run rover_navigation odometry_node &
ODOM_PID=$!

sleep 2

# Check if /odom topic exists (publisher creates it immediately)
if ! ros2 topic list | grep -q "^/odom$"; then
  echo "✗ ERROR: /odom topic not found"
  exit 1
fi

# Check if ESP is publishing (odometry_node needs ESP data)
if ! ros2 topic list | grep -qE "^/esp/odom$|^esp/odom$"; then
  echo "⚠ WARNING: /esp/odom topic not found"
  echo "  odometry_node requires ESP to publish odometry"
  echo "  /odom topic exists but may not have messages until ESP publishes"
  echo "  Continuing anyway..."
else
  echo "✓ ESP odometry topic found"
fi

echo "✓ Odometry running (PID: $ODOM_PID)"
echo

# -----------------------------
# 4. START EKF
# -----------------------------
echo "[4/6] Starting EKF (robot_localization)..."

ros2 run robot_localization ekf_node \
  --ros-args --params-file ~/ros2_ws/config/ekf.yaml &
EKF_PID=$!

sleep 3

if ! ros2 node list | grep -q ekf_filter_node; then
  echo "✗ ERROR: EKF node not visible"
  exit 1
fi

echo "✓ EKF running (PID: $EKF_PID)"
echo

# -----------------------------
# 5. START LIDAR
# -----------------------------
echo "[5/6] Starting RPLIDAR..."

# Check if hardware is connected
if [ ! -e "/dev/ttyUSB1" ]; then
  echo "⚠ WARNING: /dev/ttyUSB1 not found - hardware may not be connected"
  echo "  Continuing anyway (topic may not appear)..."
fi

ros2 launch rplidar_ros rplidar.launch.py \
  serial_port:=/dev/ttyUSB1 \
  frame_id:=laser \
  scan_mode:=Standard &
LIDAR_PID=$!

# Wait longer for RPLIDAR initialization (hardware can take 5-10 seconds)
echo "  Waiting for RPLIDAR to initialize..."
for i in {1..10}; do
  sleep 1
  if ros2 topic list 2>/dev/null | grep -qE "^/scan$|^scan$"; then
    echo "✓ /scan topic found after ${i}s"
    break
  fi
  echo "  ... still waiting (${i}/10s)"
done

if ! ros2 topic list 2>/dev/null | grep -qE "^/scan$|^scan$"; then
  echo "✗ WARNING: /scan topic not found after 10 seconds"
  echo "  RPLIDAR process is running (PID: $LIDAR_PID)"
  echo "  Possible causes:"
  echo "    - Hardware not connected to /dev/ttyUSB1"
  echo "    - Hardware initialization failed"
  echo "    - Check: ros2 topic list | grep scan"
  echo "    - Check: dmesg | grep ttyUSB1"
  echo "  Continuing anyway..."
else
echo "✓ LiDAR running (PID: $LIDAR_PID)"
fi
echo

# -----------------------------
# 6. OPTIONAL: SLAM TOOLBOX
# -----------------------------
# Uncomment ONLY after odom → base_link TF is confirmed
#
# echo "[6/6] Starting SLAM Toolbox..."
# ros2 launch slam_toolbox online_async_launch.py \
#   slam_params_file:=~/ros2_ws/config/slam_params.yaml &
#
# sleep 3

echo "========================================="
echo "  ALL CORE NODES STARTED SUCCESSFULLY"
echo "========================================="

echo
echo "Useful checks:"
echo "  ros2 topic echo /odom --once"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo "  ros2 topic echo /scan --once"
echo
echo "To stop everything:"
echo "  pkill -9 -f odometry_node"
echo "  pkill -9 -f ekf_node"
echo "  pkill -9 -f rplidar"
