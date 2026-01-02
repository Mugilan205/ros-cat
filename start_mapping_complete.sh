#!/bin/bash

# ================================================================
# START MAPPING + NAVIGATION COMPLETE SCRIPT
# Launches: Hardware + SLAM + Nav2 + RViz
# ================================================================

set -e

WORKSPACE_DIR="/home/caterpillar/ros2_ws"
LOG_DIR="$WORKSPACE_DIR/.launch_logs"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "🚀 STARTING COMPLETE MAPPING & NAVIGATION"
echo "=========================================="

# Source ROS2 setup
cd "$WORKSPACE_DIR"
source install/setup.bash

echo ""
echo "[1/3] Launching Hardware Bringup (RPLIDAR + Motors)..."
rm -f "$LOG_DIR/bringup.log"
gnome-terminal --tab --title="Bringup" -- bash -c "
  cd '$WORKSPACE_DIR'
  source install/setup.bash
  echo '📡 Hardware Bringup Starting...'
  ros2 launch your_robot_bringup bringup.launch.py 2>&1 | tee '$LOG_DIR/bringup.log'
" &
BRINGUP_PID=$!

sleep 3

echo "[2/3] Launching Nav2 + SLAM Stack..."
rm -f "$LOG_DIR/navigation.log"
gnome-terminal --tab --title="Navigation" -- bash -c "
  cd '$WORKSPACE_DIR'
  source install/setup.bash
  echo '🗺️  Navigation + SLAM Starting...'
  ros2 launch rover_navigation full_navigation.launch.py 2>&1 | tee '$LOG_DIR/navigation.log'
" &
NAV_PID=$!

sleep 5

echo "[3/3] Launching RViz..."
gnome-terminal --tab --title="RViz" -- bash -c "
  cd '$WORKSPACE_DIR'
  source install/setup.bash
  echo '📊 RViz Starting...'
  ros2 run rviz2 rviz2 -d config/nav2_safe.rviz
" &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "✅ SYSTEM RUNNING"
echo "=========================================="
echo ""
echo "📍 Process IDs:"
echo "   Bringup:    $BRINGUP_PID"
echo "   Navigation: $NAV_PID"
echo "   RViz:       $RVIZ_PID"
echo ""
echo "📋 Log Files:"
echo "   Bringup:    $LOG_DIR/bringup.log"
echo "   Navigation: $LOG_DIR/navigation.log"
echo ""
echo "🎯 To Test Navigation:"
echo "   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "   \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\""
echo ""
echo "🔍 To Monitor:"
echo "   ros2 topic echo /cmd_vel            # Command velocity"
echo "   ros2 topic echo /amcl_pose          # Pose estimate"
echo "   ros2 run tf2_ros tf2_echo map base_footprint  # TF chain"
echo ""
echo "⏹️  To stop: Close terminal windows or press Ctrl+C"
echo "=========================================="

# Wait for all processes
wait
