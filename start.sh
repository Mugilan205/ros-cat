#!/bin/bash

echo "🚀 Starting Caterpillar Autonomous Rover..."

# 1️⃣ Source ROS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2️⃣ Set domain (important for DDS stability)
export ROS_DOMAIN_ID=0

# 3️⃣ USB permissions (persistent paths)
sudo chmod 666 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 2>/dev/null
sudo chmod 666 /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0 2>/dev/null

# 4️⃣ Launch everything
ros2 launch rover_navigation autonomy.launch.py
