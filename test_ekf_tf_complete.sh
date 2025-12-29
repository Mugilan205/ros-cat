#!/bin/bash
# Complete EKF TF Diagnostic Test Script
# Tests all conditions for EKF TF publishing

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== EKF TF Complete Diagnostic Test ===${NC}"
echo ""

# Export DDS settings
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml
export ROS_DOMAIN_ID=0

# Source ROS 2
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /home/caterpillar/ros2_ws/install/setup.bash 2>/dev/null || true

# Test 1: Verify EKF Node State
echo -e "${YELLOW}Test 1: EKF Node State${NC}"
if ros2 node list | grep -q "/ekf_filter_node"; then
    echo -e "${GREEN}✓ PASS: /ekf_filter_node is visible${NC}"
else
    echo -e "${RED}✗ FAIL: /ekf_filter_node not visible${NC}"
    exit 1
fi

PUBLISH_TF=$(ros2 param get /ekf_filter_node publish_tf 2>&1 | grep -oP 'Boolean value is: \K\w+')
if [ "$PUBLISH_TF" = "True" ]; then
    echo -e "${GREEN}✓ PASS: publish_tf = True${NC}"
else
    echo -e "${RED}✗ FAIL: publish_tf = $PUBLISH_TF${NC}"
    exit 1
fi

if ros2 node info /ekf_filter_node 2>&1 | grep -q "/tf.*tf2_msgs"; then
    echo -e "${GREEN}✓ PASS: Node publishes to /tf topic${NC}"
else
    echo -e "${RED}✗ FAIL: Node does not publish to /tf topic${NC}"
    exit 1
fi
echo ""

# Test 2: Verify Odometry Input
echo -e "${YELLOW}Test 2: Odometry Input (/odom topic)${NC}"
ODOM_PUB_COUNT=$(ros2 topic info /odom 2>&1 | grep "Publisher count" | grep -oP '\d+')
if [ "$ODOM_PUB_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✓ PASS: /odom has $ODOM_PUB_COUNT publisher(s)${NC}"
else
    echo -e "${RED}✗ FAIL: /odom has 0 publishers (odometry node not running or not discovered)${NC}"
    echo -e "${YELLOW}  → Start odometry node: ros2 run rover_navigation odometry_node${NC}"
    exit 1
fi

ODOM_MSG=$(timeout 2 ros2 topic echo /odom --once 2>&1 | head -20)
if echo "$ODOM_MSG" | grep -q "header:"; then
    FRAME_ID=$(echo "$ODOM_MSG" | grep -A 1 "frame_id:" | tail -1 | tr -d ' "')
    CHILD_FRAME=$(echo "$ODOM_MSG" | grep -A 1 "child_frame_id:" | tail -1 | tr -d ' "')
    
    if [ "$FRAME_ID" = "odom" ]; then
        echo -e "${GREEN}✓ PASS: header.frame_id = odom${NC}"
    else
        echo -e "${RED}✗ FAIL: header.frame_id = $FRAME_ID (expected: odom)${NC}"
        exit 1
    fi
    
    if [ "$CHILD_FRAME" = "base_link" ]; then
        echo -e "${GREEN}✓ PASS: child_frame_id = base_link${NC}"
    else
        echo -e "${RED}✗ FAIL: child_frame_id = $CHILD_FRAME (expected: base_link)${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ FAIL: Cannot read /odom messages${NC}"
    exit 1
fi
echo ""

# Test 3: Verify EKF Filter Initialization
echo -e "${YELLOW}Test 3: EKF Filter Initialization${NC}"
sleep 2  # Wait for EKF to process messages

FILTERED_MSG=$(timeout 3 ros2 topic echo /odometry/filtered --once 2>&1 | head -20)
if echo "$FILTERED_MSG" | grep -q "header:"; then
    echo -e "${GREEN}✓ PASS: /odometry/filtered is publishing (filter initialized)${NC}"
else
    echo -e "${RED}✗ FAIL: /odometry/filtered not publishing (filter not initialized)${NC}"
    echo -e "${YELLOW}  → EKF may need more time or odometry messages are invalid${NC}"
    exit 1
fi

# Check diagnostics
DIAGNOSTICS=$(timeout 3 ros2 topic echo /diagnostics --once 2>&1 | grep -A 10 "ekf_filter_node")
EVENTS=$(echo "$DIAGNOSTICS" | grep "Events since startup" | grep -oP '\d+')
if [ ! -z "$EVENTS" ] && [ "$EVENTS" -gt 0 ]; then
    echo -e "${GREEN}✓ PASS: EKF has received $EVENTS events${NC}"
else
    echo -e "${YELLOW}⚠ WARNING: EKF events count unclear (may be normal if just started)${NC}"
fi
echo ""

# Test 4: Verify TF Publication
echo -e "${YELLOW}Test 4: TF Publication${NC}"
TF_MSG=$(timeout 3 ros2 topic echo /tf --once 2>&1 | head -30)
if echo "$TF_MSG" | grep -q "transforms:"; then
    echo -e "${GREEN}✓ PASS: /tf topic is publishing messages${NC}"
    
    # Check if odom->base_link transform is in the message
    if echo "$TF_MSG" | grep -q "odom\|base_link"; then
        echo -e "${GREEN}✓ PASS: Transform contains odom/base_link frames${NC}"
    fi
else
    echo -e "${RED}✗ FAIL: /tf topic not publishing messages${NC}"
    echo -e "${YELLOW}  → EKF may not be initialized yet, or TF publishing failed${NC}"
    exit 1
fi
echo ""

# Test 5: Verify TF Tree
echo -e "${YELLOW}Test 5: TF Tree (odom → base_link)${NC}"
TF_ECHO_OUTPUT=$(timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10)
if echo "$TF_ECHO_OUTPUT" | grep -q "At time\|Translation\|Rotation"; then
    echo -e "${GREEN}✓ PASS: odom → base_link transform exists and is queryable${NC}"
    echo "$TF_ECHO_OUTPUT" | head -5
else
    if echo "$TF_ECHO_OUTPUT" | grep -q "frame does not exist"; then
        echo -e "${RED}✗ FAIL: odom frame does not exist in TF tree${NC}"
        echo -e "${YELLOW}  → Even though /tf is publishing, transform may not be available yet${NC}"
        exit 1
    else
        echo -e "${RED}✗ FAIL: Cannot query transform (timeout or error)${NC}"
        echo "$TF_ECHO_OUTPUT"
        exit 1
    fi
fi
echo ""

# Final Summary
echo -e "${GREEN}=== ALL TESTS PASSED ===${NC}"
echo ""
echo "Summary:"
echo "  - EKF node is running and configured correctly"
echo "  - Odometry input is available and valid"
echo "  - EKF filter is initialized"
echo "  - TF transforms are being published"
echo "  - odom → base_link transform is available"
echo ""
echo -e "${GREEN}EKF TF system is working correctly!${NC}"

