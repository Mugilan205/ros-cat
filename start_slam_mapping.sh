#!/bin/bash
# Start SLAM Mapping Stack in Correct Order
# This fixes the TF synchronization issues

set -e

echo "=========================================="
echo "Starting SLAM Mapping Stack"
echo "=========================================="

# Source ROS2
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash 2>/dev/null || true

echo ""
echo "Step 1: Starting robot_state_publisher (for base_link → laser TF)..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting robot_state_publisher...'
ros2 run robot_state_publisher robot_state_publisher /home/caterpillar/ros2_ws/urdf/your_robot.urdf
exec bash
" &

sleep 2

echo "Step 2: Starting RPLidar..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting RPLidar...'
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
exec bash
" &

sleep 3

echo "Step 3: Starting Odometry (for odom → base_link TF)..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting Odometry Node...'
ros2 run rover_navigation odometry_node
exec bash
" &

sleep 2

echo "Step 4: Starting EKF (sensor fusion)..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting EKF...'
ros2 run robot_localization ekf_node --ros-args -p config_file:=/home/caterpillar/ros2_ws/config/ekf.yaml
exec bash
" &

sleep 3

echo "Step 5: Starting SLAM Toolbox..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting SLAM Toolbox...'
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p scan_topic:=/scan -p map_frame:=map -p odom_frame:=odom -p base_frame:=base_link -p config_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml
exec bash
" &

sleep 3

echo "Step 6: Starting RViz2 (optional)..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
echo 'Starting RViz2...'
rviz2
exec bash
" &

echo ""
echo "=========================================="
echo "All components started!"
echo "=========================================="
echo ""
echo "Verify TF chain:"
echo "  ros2 run tf2_ros tf2_echo base_link laser"
echo ""
echo "Check topics:"
echo "  ros2 topic list | grep -E 'scan|map|odom'"
echo ""
echo "Monitor SLAM:"
echo "  ros2 topic echo /map --once"
echo ""







