#!/bin/bash
# Load ROS2
source /opt/ros/jazzy/setup.bash

echo "Starting RPLIDAR..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser; exec bash"

sleep 3  # Wait for /scan to start

echo "Starting laser TF broadcaster..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; python3 ~/ros2_ws/bridge/laser_tf.py; exec bash"

sleep 1  # Wait for TF

echo "Starting fake odometry..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; python3 ~/ros2_ws/bridge/fake_odom.py; exec bash"

sleep 2  # Wait for odom → base_link TF

echo "Starting SLAM Toolbox..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 launch slam_toolbox online_sync_launch.py; exec bash"

sleep 2

echo "Starting RViz2..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; rviz2; exec bash"

echo "All systems started successfully!"

