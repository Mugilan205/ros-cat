#!/bin/bash
# Example Commands for Rover Navigation System
# Copy and paste these commands to test the system

# ============================================================================
# 1. BUILD AND SETUP
# ============================================================================

# Build the package
cd ~/ros2_ws
colcon build --packages-select rover_navigation
source install/setup.bash

# Verify installation
ros2 pkg list | grep rover_navigation


# ============================================================================
# 2. LAUNCH OPTIONS
# ============================================================================

# Option A: Full navigation stack (recommended)
# Terminal 1:
ros2 launch rover_navigation full_navigation.launch.py

# Option B: Individual nodes
# Terminal 1: Obstacle Detection
ros2 run rover_navigation obstacle_detection

# Terminal 2: RRT Planner
ros2 run rover_navigation rrt_planner

# Terminal 3: Path Executor
ros2 run rover_navigation path_executor


# ============================================================================
# 3. SEND GOALS
# ============================================================================

# Send goal to (5.0, 0.0)
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 5.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'

# Send goal to (10.0, 5.0)
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'

# Send goal with negative coordinates (backward)
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: -5.0, y: -5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'


# ============================================================================
# 4. MONITORING TOPICS
# ============================================================================

# Monitor planning status (SUCCESS/FAILED/REPLANNING)
ros2 topic echo /plan_status

# Echo detected obstacles
ros2 topic echo /obstacles

# Echo planned path waypoints
ros2 topic echo /path

# Echo velocity commands sent to motor
ros2 topic echo /cmd_vel

# Echo current odometry
ros2 topic echo /odom

# Echo incoming LiDAR scans
ros2 topic echo /scan


# ============================================================================
# 5. TOPIC INFORMATION
# ============================================================================

# List all active topics
ros2 topic list

# List rover_navigation specific topics
ros2 topic list | grep -E "(scan|obstacles|path|cmd_vel|plan_status)"

# Get topic frequency
ros2 topic hz /scan
ros2 topic hz /path
ros2 topic hz /cmd_vel

# Get topic info
ros2 topic info /path
ros2 topic info /cmd_vel


# ============================================================================
# 6. NODE INFORMATION
# ============================================================================

# List all running nodes
ros2 node list

# List navigation nodes
ros2 node list | grep -E "obstacle|planner|executor"

# Get detailed node info
ros2 node info /obstacle_detection
ros2 node info /rrt_planner
ros2 node info /path_executor

# See node graph
ros2 run rqt_graph rqt_graph


# ============================================================================
# 7. VISUALIZATION IN RVIZ
# ============================================================================

# Launch RViz
ros2 run rviz2 rviz2

# In RViz, add displays:
# 1. LaserScan topic: /scan (white points on robot)
# 2. Path topic: /path (green line showing planned path)
# 3. OccupancyGrid: /occupancy_grid (2D grid with obstacles)
# 4. PoseStamped: /odom (robot pose/frame)
# 5. PoseStamped: /goal (goal marker if added)


# ============================================================================
# 8. TESTING WITH TEST SCENARIOS
# ============================================================================

# Make test script executable
chmod +x ~/ros2_ws/src/rover_navigation/test_navigation.py

# Scenario 1: Simple goal without obstacles
ros2 run rover_navigation test_navigation --scenario 1

# Scenario 2: Goal with static obstacle to avoid
ros2 run rover_navigation test_navigation --scenario 2

# Scenario 3: Dynamic obstacles triggering replanning
ros2 run rover_navigation test_navigation --scenario 3 --verbose


# ============================================================================
# 9. DEBUGGING WITH LOGGING
# ============================================================================

# Enable debug logging for obstacle detection
ros2 run rover_navigation obstacle_detection \
  --ros-args --log-level rover_navigation:=DEBUG

# Enable debug logging for RRT planner
ros2 run rover_navigation rrt_planner \
  --ros-args --log-level rover_navigation:=DEBUG

# See all debug messages
ros2 run rover_navigation path_executor \
  --ros-args --log-level rover_navigation:=DEBUG


# ============================================================================
# 10. PARAMETER MODIFICATION (at runtime)
# ============================================================================

# Set parameter value
ros2 param set /obstacle_detection obstacle_distance_threshold 2.0

# Get parameter value
ros2 param get /obstacle_detection obstacle_distance_threshold

# List all parameters for a node
ros2 param list /rrt_planner

# Dump all parameters
ros2 param dump /rrt_planner > /tmp/rrt_params.yaml

# Load parameters from file
ros2 param load /rrt_planner /tmp/rrt_params.yaml


# ============================================================================
# 11. LAUNCH WITH CUSTOM PARAMETERS
# ============================================================================

# Launch with custom obstacle threshold
ros2 launch rover_navigation obstacle_detection.launch.py \
  obstacle_threshold:=2.0 max_range:=15.0

# Launch planner with custom RRT parameters
ros2 launch rover_navigation rrt_planner.launch.py \
  step_size:=0.3 max_iterations:=10000

# Launch full stack with custom parameters
ros2 launch rover_navigation full_navigation.launch.py \
  linear_speed:=0.8 angular_speed:=1.5


# ============================================================================
# 12. ADVANCED DEBUGGING
# ============================================================================

# Record a bag file (ROS 2 recording)
ros2 bag record -o rover_nav /scan /odom /path /cmd_vel /plan_status

# Play back a bag file
ros2 bag play rover_nav

# Inspect bag file
ros2 bag info rover_nav

# Monitor CPU/Memory usage
ros2 run rqt_monitor rqt_monitor

# View message definitions
ros2 msg show sensor_msgs/LaserScan
ros2 msg show nav_msgs/Path
ros2 msg show geometry_msgs/Twist

# Show action/service definitions
ros2 action list
ros2 service list


# ============================================================================
# 13. REAL ROVER INTEGRATION
# ============================================================================

# Start LiDAR driver (rplidar)
ros2 launch rplidar_ros rplidar_a1_launch.py

# Start odometry publisher (your robot specific)
ros2 run your_robot_bringup odometry_node

# Start motor controller (your BTS driver)
ros2 run bts_motor_controller motor_control_node

# Now launch navigation
ros2 launch rover_navigation full_navigation.launch.py

# Send goals to rover
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 10.0, y: 0.0, z: 0.0}}
}'


# ============================================================================
# 14. TROUBLESHOOTING COMMANDS
# ============================================================================

# Check if LiDAR is publishing
timeout 5 ros2 topic hz /scan

# Check if odometry is publishing
timeout 5 ros2 topic hz /odom

# Check if obstacles are being detected
timeout 5 ros2 topic echo /obstacles | head -20

# Check if path is generated
timeout 5 ros2 topic echo /path

# Check if velocity commands are sent
timeout 5 ros2 topic echo /cmd_vel

# Check ROS 2 domain ID (useful for multiple robots)
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0  # Change if needed


# ============================================================================
# 15. PERFORMANCE ANALYSIS
# ============================================================================

# Monitor latency (time between scan and cmd_vel)
# Install: sudo apt install ros-jazzy-latency-tools
# Then run: ros2 run latency_tools timings /scan /cmd_vel

# Monitor CPU per node
top
ps aux | grep ros2

# Profile with cProfile (Python)
python3 -m cProfile -s cumtime /opt/ros/jazzy/bin/ros2 run rover_navigation obstacle_detection

# Generate computation graph
ros2 run rqt_graph rqt_graph


# ============================================================================
# 16. CLEAN BUILD
# ============================================================================

# Clean build directory
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select rover_navigation

# Rebuild from scratch
colcon build --packages-select rover_navigation --symlink-install


# ============================================================================
# 17. INSTALLATION VERIFICATION
# ============================================================================

# Verify package is built
ls -la ~/ros2_ws/install/rover_navigation/

# Verify executables are available
which obstacle_detection
which rrt_planner
which path_executor

# Test imports
python3 -c "from rover_navigation.obstacle_detection import ObstacleDetectionNode"
python3 -c "from rover_navigation.rrt_planner import RRTPlannerNode"
python3 -c "from rover_navigation.path_executor import PathExecutorNode"


# ============================================================================
# 18. HELPFUL ALIASES (add to ~/.bashrc)
# ============================================================================

# alias ros_nav_build="cd ~/ros2_ws && colcon build --packages-select rover_navigation"
# alias ros_nav_launch="ros2 launch rover_navigation full_navigation.launch.py"
# alias ros_nav_goal="ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{header: {frame_id: \"odom\"}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}'"
# alias ros_nav_status="ros2 topic echo /plan_status"
# alias ros_nav_test="ros2 run rover_navigation test_navigation"


# ============================================================================
# 19. QUICK START SEQUENCE
# ============================================================================

# Run this sequence to start everything:
cd ~/ros2_ws
source install/setup.bash

# Terminal 1
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2 (after seeing nodes started)
sleep 2
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 0.0, z: 0.0}}
}'

# Terminal 3 (optional monitoring)
ros2 topic echo /plan_status


# ============================================================================
# 20. RVIZ VISUALIZATION QUICK SETUP
# ============================================================================

# Terminal 1: Launch navigation
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2: Launch RViz with default config
ros2 run rviz2 rviz2

# In RViz:
# - Set Fixed Frame to "odom" (top left)
# - Add displays:
#   * LaserScan: /scan
#   * Path: /path
#   * OccupancyGrid: /occupancy_grid

# Terminal 3: Send test goal
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 10.0, y: 5.0, z: 0.0}}
}'

# Watch robot plan and execute path!
