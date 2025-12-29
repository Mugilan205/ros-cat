#!/bin/bash
# Quick Test Script for A* + LWB Navigation Stack
# Run this after building the package

set -e  # Exit on error

echo "=========================================="
echo "A* + LWB Navigation Stack Quick Test"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || echo -e "${YELLOW}Warning: Could not source workspace${NC}"

echo -e "${GREEN}Step 1: Checking hardware connections...${NC}"

# Check LiDAR
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ LiDAR detected at /dev/ttyUSB0${NC}"
else
    echo -e "${RED}✗ LiDAR not found at /dev/ttyUSB0${NC}"
fi

# Check pigpio
if systemctl is-active --quiet pigpiod; then
    echo -e "${GREEN}✓ pigpio daemon is running${NC}"
else
    echo -e "${YELLOW}⚠ pigpio daemon not running${NC}"
    echo "   Start with: sudo systemctl start pigpiod"
fi

# Check odometry port
if [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}✓ Odometry port detected at /dev/ttyACM0${NC}"
else
    echo -e "${YELLOW}⚠ Odometry port not found at /dev/ttyACM0${NC}"
fi

echo ""
echo -e "${GREEN}Step 2: Checking package build...${NC}"

# Check if package is built
if ros2 pkg executables rover_navigation | grep -q "a_star_planner"; then
    echo -e "${GREEN}✓ Navigation package is built${NC}"
else
    echo -e "${RED}✗ Navigation package not found${NC}"
    echo "   Build with: cd ~/ros2_ws && colcon build --packages-select rover_navigation"
    exit 1
fi

# List available executables
echo ""
echo "Available navigation executables:"
ros2 pkg executables rover_navigation | grep -E "a_star|lwb|planner|dashboard" | sed 's/^/  ✓ /'

echo ""
echo -e "${GREEN}Step 3: Checking ROS2 topics (requires nodes running)...${NC}"

# Check if topics exist (if nodes are running)
TOPICS_TO_CHECK=("/scan" "/odom" "/map" "/cmd_vel")

for topic in "${TOPICS_TO_CHECK[@]}"; do
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        echo -e "${GREEN}✓ Topic $topic exists${NC}"
    else
        echo -e "${YELLOW}⚠ Topic $topic not found (node may not be running)${NC}"
    fi
done

echo ""
echo -e "${GREEN}Step 4: Quick dependency check...${NC}"

# Check Python dependencies
python3 -c "import numpy; import scipy; import tf2_ros" 2>/dev/null && \
    echo -e "${GREEN}✓ Python dependencies OK${NC}" || \
    echo -e "${YELLOW}⚠ Some Python dependencies missing${NC}"

# Check ROS2 packages
REQUIRED_PKGS=("slam_toolbox" "robot_localization" "rplidar_ros")
for pkg in "${REQUIRED_PKGS[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo -e "${GREEN}✓ ROS2 package $pkg installed${NC}"
    else
        echo -e "${YELLOW}⚠ ROS2 package $pkg not found${NC}"
    fi
done

echo ""
echo "=========================================="
echo -e "${GREEN}Quick Check Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Start SLAM and mapping:"
echo "   ros2 launch slam_toolbox online_async_launch.py"
echo ""
echo "2. Launch navigation stack:"
echo "   ros2 launch rover_navigation a_star_lwb_navigation.launch.py"
echo ""
echo "3. Send a test goal:"
echo "   ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{\"header\": {\"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": 1.0, \"y\": 0.5}, \"orientation\": {\"w\": 1.0}}}'"
echo ""
echo "For detailed testing guide, see: TESTING_GUIDE.md"
echo ""












