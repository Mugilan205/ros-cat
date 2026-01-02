#!/bin/bash

set -euo pipefail

fail() { echo "[start_mapping] Error: $1" >&2; exit 1; }
trap 'fail "line $LINENO"' ERR

safe_source() {
  set +u
  source "$1"
  set -u
}

command -v ros2 >/dev/null || fail "ros2 not found. Source ROS before running."
command -v xacro >/dev/null || fail "xacro not found. Install or source the right workspace."
command -v gnome-terminal >/dev/null || fail "gnome-terminal not available."

echo "========== ROS 2 SLAM SAFE START =========="

safe_source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws

echo "🔧 Building workspace..."
colcon build --symlink-install

safe_source ~/ros2_ws/install/setup.bash

start_term() {
  local title="$1"; shift
  gnome-terminal --title="${title}" -- bash -c \
  "set +e; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; $*; exec bash"
}

LIDAR_PORT="/dev/ttyUSB0"
[ -e "${LIDAR_PORT}" ] || fail "LiDAR port ${LIDAR_PORT} not found."

ESP_PORT="/dev/ttyUSB1"
[ -e "${ESP_PORT}" ] || fail "ESP odometry port ${ESP_PORT} not found."

echo "🚀 Starting LiDAR..."
start_term "LiDAR" \
"ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=${LIDAR_PORT} -p serial_baudrate:=115200"

sleep 4

echo "🤖 Starting robot_state_publisher..."
start_term "robot_state_publisher" \
"robot_description=\"\$(xacro ~/ros2_ws/src/simple_rover_description/urdf/simple_rover.urdf.xacro)\"; \
 ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\${robot_description}\""

sleep 4

echo "📡 Starting ESP Serial Odometry..."
start_term "esp_serial_odometry" \
"ros2 run rover_navigation esp_serial_odometry_node --ros-args -p port:=${ESP_PORT} -p baud:=115200"

sleep 4

# ==========================================================
# ✅ ADDITION 1 — TEMPORARY map → odom TF (SAFE + NON-BREAKING)
# ==========================================================
echo "🛠️ Ensuring map → odom TF exists (temporary if SLAM/AMCL not active)..."
start_term "map_to_odom_tf" \
"ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"

sleep 2

# ==========================================================
# ✅ ADDITION 2 — WAIT FOR CORRECT TF CHAIN
# odom → base_footprint → base_link → laser_frame
# ==========================================================
echo "⏳ Waiting for TF chain (odom → base_footprint → laser_frame)..."

set +e
python3 - <<'PY'
import sys, time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

FRAMES = [
    ('odom', 'base_footprint'),
    ('base_footprint', 'base_link'),
    ('base_link', 'laser_frame')
]

TIMEOUT = 25.0

rclpy.init()
node = rclpy.create_node('tf_waiter')
buf = Buffer()
listener = TransformListener(buf, node, spin_thread=False)

start = time.time()
ok = False

while time.time() - start < TIMEOUT:
    rclpy.spin_once(node, timeout_sec=0.1)
    ready = True
    for target, source in FRAMES:
        if not buf.can_transform(target, source, Time()):
            ready = False
            break
    if ready:
        ok = True
        break

node.destroy_node()
rclpy.shutdown()

if not ok:
    sys.exit(1)
PY

rc=$?
set -e
[ $rc -eq 0 ] || fail "TF chain not ready (odom → base_footprint → laser_frame)"

sleep 2

echo "🗺️ Starting SLAM Toolbox..."
start_term "slam_toolbox" \
"ros2 launch slam_toolbox online_async_launch.py \
 slam_params_file:=/home/caterpillar/ros2_ws/src/rover_navigation/config/slam_params.yaml \
 odom_frame:=odom base_frame:=base_footprint scan_topic:=/scan use_sim_time:=false"

echo "✅ All nodes launched safely"
sleep 5

echo "📍 Starting Localization (AMCL + Map Server)..."
start_term "nav2_localization" "\
ros2 launch nav2_bringup localization_launch.py \
map:=/home/caterpillar/maps/rover_map.yaml \
use_sim_time:=false"

sleep 5

echo "🧭 Starting Nav2 Navigation Stack..."
start_term "nav2_navigation" "\
ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false"