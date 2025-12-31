#!/bin/bash

echo "🚀 Starting Caterpillar Autonomous Rover..."

# 1️⃣ Source ROS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2️⃣ Set domain (important for DDS stability)
export ROS_DOMAIN_ID=0

# 3️⃣ USB permissions (safe)
sudo chmod 666 /dev/ttyUSB0 2>/dev/null
sudo chmod 666 /dev/ttyUSB1 2>/dev/null

# 4️⃣ Launch everything
ros2 launch rover_navigation autonomy.launch.py
