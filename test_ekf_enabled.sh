#!/bin/bash
# Test script to verify ekf_node behavior with enabled: true

set -e

echo "=========================================="
echo "Testing EKF Node with enabled: true"
echo "=========================================="
echo ""

# Source environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Step 1: Checking current nodes..."
ros2 node list
echo ""

echo "Step 2: Starting ekf_node in background..."
ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/config/ekf.yaml > /tmp/ekf_test.log 2>&1 &
EKF_PID=$!
echo "EKF PID: $EKF_PID"
echo ""

echo "Step 3: Waiting 2 seconds..."
sleep 2
echo ""

echo "Step 4: Checking if node is running..."
if ps -p $EKF_PID > /dev/null; then
    echo "✓ Process is still running (PID: $EKF_PID)"
else
    echo "✗ Process has exited"
    echo ""
    echo "Last 20 lines of output:"
    tail -20 /tmp/ekf_test.log 2>/dev/null || echo "No log file"
fi
echo ""

echo "Step 5: Checking node list..."
ros2 node list
echo ""

echo "Step 6: Checking if /ekf_filter_node exists..."
if ros2 node list | grep -q ekf_filter_node; then
    echo "✓ /ekf_filter_node found in node list"
    echo ""
    echo "Step 7: Checking enabled parameter..."
    ros2 param get /ekf_filter_node enabled 2>&1 || echo "Could not get parameter"
else
    echo "✗ /ekf_filter_node NOT found in node list"
fi
echo ""

echo "Step 8: Cleaning up..."
kill $EKF_PID 2>/dev/null || true
wait $EKF_PID 2>/dev/null || true
echo "Done."











