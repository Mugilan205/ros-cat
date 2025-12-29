#!/bin/bash
# FastDDS SHM Disable Script for ROS 2 Jazzy EKF Node Discovery Fix
# This script disables FastDDS shared-memory transport and forces UDP discovery

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== FastDDS SHM Disable Fix for EKF Node Discovery ===${NC}"
echo ""

# Step 1: Verify XML profile exists
if [ ! -f "/tmp/fastrtps_disable_shm.xml" ]; then
    echo "ERROR: /tmp/fastrtps_disable_shm.xml not found!"
    exit 1
fi

echo -e "${GREEN}Step 1: XML profile verified${NC}"

# Step 2: Export environment variables
echo -e "${YELLOW}Step 2: Exporting environment variables...${NC}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml
export ROS_DOMAIN_ID=0

echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""

# Step 3: Source ROS 2 Jazzy
echo -e "${YELLOW}Step 3: Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -f "/home/caterpillar/ros2_ws/install/setup.bash" ]; then
    source /home/caterpillar/ros2_ws/install/setup.bash
fi
echo -e "${GREEN}ROS 2 environment sourced${NC}"
echo ""

# Step 4: Verify ekf_node exists
echo -e "${YELLOW}Step 4: Verifying ekf_node executable...${NC}"
if command -v ros2 &> /dev/null; then
    if ros2 pkg executables robot_localization | grep -q ekf_node; then
        echo -e "${GREEN}ekf_node found${NC}"
    else
        echo "ERROR: ekf_node not found in robot_localization package"
        exit 1
    fi
else
    echo "ERROR: ros2 command not found"
    exit 1
fi
echo ""

# Step 5: Check if ekf_node is already running and kill it
echo -e "${YELLOW}Step 5: Checking for existing ekf_node processes...${NC}"
EKF_PIDS=$(pgrep -f "ekf_node" || true)
if [ ! -z "$EKF_PIDS" ]; then
    echo "Found running ekf_node processes: $EKF_PIDS"
    echo "Killing existing processes..."
    kill -9 $EKF_PIDS 2>/dev/null || true
    sleep 2
fi
echo -e "${GREEN}Clean state verified${NC}"
echo ""

# Step 6: Start ekf_node with correct parameters
EKF_CONFIG="/home/caterpillar/ros2_ws/config/ekf.yaml"
if [ ! -f "$EKF_CONFIG" ]; then
    echo "ERROR: EKF config file not found at $EKF_CONFIG"
    exit 1
fi

echo -e "${YELLOW}Step 6: Starting ekf_node with UDP-only transport...${NC}"
echo "  Config file: $EKF_CONFIG"
echo "  Command: ros2 run robot_localization ekf_node --ros-args --params-file $EKF_CONFIG"
echo ""
echo -e "${GREEN}Starting ekf_node in background...${NC}"
echo ""

# Start ekf_node in background
ros2 run robot_localization ekf_node --ros-args --params-file "$EKF_CONFIG" &
EKF_PID=$!
echo "ekf_node started with PID: $EKF_PID"

# Wait a moment for the node to initialize
sleep 3

# Step 7: Verification
echo ""
echo -e "${YELLOW}=== VERIFICATION CHECKLIST ===${NC}"
echo ""

# Check 1: Node visibility
echo -e "${YELLOW}Check 1: Node visibility (ros2 node list)${NC}"
if ros2 node list | grep -q "/ekf_filter_node"; then
    echo -e "${GREEN}✓ PASS: /ekf_filter_node is visible in ros2 node list${NC}"
    ros2 node list | grep ekf_filter_node
else
    echo -e "${YELLOW}✗ FAIL: /ekf_filter_node NOT visible yet (may need more time)${NC}"
    ros2 node list
fi
echo ""

# Check 2: Topic visibility
echo -e "${YELLOW}Check 2: Topic visibility (ros2 topic list)${NC}"
if ros2 topic list | grep -q "/odometry/filtered"; then
    echo -e "${GREEN}✓ PASS: /odometry/filtered topic is visible${NC}"
    ros2 topic list | grep odometry
else
    echo -e "${YELLOW}✗ FAIL: /odometry/filtered topic NOT visible yet${NC}"
    echo "Available odometry topics:"
    ros2 topic list | grep -i odom || echo "  (none found)"
fi
echo ""

# Check 3: Process status
echo -e "${YELLOW}Check 3: Process status${NC}"
if ps -p $EKF_PID > /dev/null; then
    echo -e "${GREEN}✓ PASS: ekf_node process is running (PID: $EKF_PID)${NC}"
else
    echo -e "${YELLOW}✗ FAIL: ekf_node process is not running${NC}"
fi
echo ""

echo -e "${YELLOW}=== MANUAL VERIFICATION COMMANDS ===${NC}"
echo ""
echo "To verify TF transform:"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo ""
echo "To check node info:"
echo "  ros2 node info /ekf_filter_node"
echo ""
echo "To monitor filtered odometry:"
echo "  ros2 topic echo /odometry/filtered"
echo ""
echo "To stop ekf_node:"
echo "  kill $EKF_PID"
echo ""
echo -e "${GREEN}Setup complete!${NC}"

