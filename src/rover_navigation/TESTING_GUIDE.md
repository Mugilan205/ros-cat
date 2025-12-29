# Testing Guide: A* + LWB Navigation Stack

Complete guide for hardware and software setup to test the new navigation stack.

## 📋 Hardware Prerequisites Checklist

### ✅ Required Hardware

1. **Raspberry Pi** (running ROS2 Jazzy)
   - ✅ ROS2 Jazzy installed
   - ✅ Network connectivity (for SSH/remote access if needed)

2. **RPLidar A1/A2**
   - ✅ Connected to USB port → `/dev/ttyUSB0`
   - ✅ Baud rate: 115200
   - ✅ Power connected (5V)
   - ✅ Frame ID: `laser`
   - ✅ Test: `ls -l /dev/ttyUSB0` should show device

3. **BTS7960 Motor Controller**
   - ✅ GPIO12 → L_PWM
   - ✅ GPIO5 → L_DIR
   - ✅ GPIO13 → R_PWM
   - ✅ GPIO6 → R_DIR
   - ✅ Power connected to motors
   - ✅ pigpio daemon running: `sudo systemctl status pigpiod`

4. **Wheel Encoders / Odometry**
   - ✅ Encoders connected (via ESP8266 or Arduino)
   - ✅ Serial port: `/dev/ttyACM0` (or configured port)
   - ✅ Publishing to `/odom` topic

5. **TF Transform Chain**
   - ✅ `base_link` → `laser` transform (x=0.12m, z=0.25m)
   - ✅ `map` → `odom` → `base_link` transform chain
   - ✅ Robot state publisher running with URDF

### 🔧 Hardware Verification Commands

```bash
# 1. Check LiDAR connection
ls -l /dev/ttyUSB0
# Should show: crw-rw---- 1 root dialout 188, 0 ...

# 2. Check pigpio daemon
sudo systemctl status pigpiod
# Should show: active (running)

# 3. Check odometry serial port
ls -l /dev/ttyACM0
# Should show device if Arduino/ESP8266 connected

# 4. Check GPIO permissions
groups
# Should include 'gpio' or 'dialout' group
```

---

## 💻 Software Setup

### Step 1: Build the Package

```bash
cd ~/ros2_ws

# Build the navigation package
colcon build --packages-select rover_navigation

# Source the workspace
source install/setup.bash

# Verify executables are available
ros2 pkg executables rover_navigation
```

Expected output should include:
```
rover_navigation a_star_planner
rover_navigation lwb_planner
rover_navigation planner_manager
rover_navigation navigation_dashboard
```

### Step 2: Verify Dependencies

```bash
# Check if required ROS2 packages are installed
ros2 pkg list | grep -E "slam_toolbox|robot_localization|rplidar_ros|tf2"

# Install missing packages if needed
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-robot-localization

# Check Python dependencies
python3 -c "import numpy, scipy, tf2_ros" && echo "Dependencies OK"
```

### Step 3: Verify Existing Nodes

Make sure your existing infrastructure is ready:

```bash
# Check if motor controller package is built
ros2 pkg executables bts_motor_controller

# Check if odometry node exists
ros2 pkg executables rover_navigation | grep odometry
```

---

## 🚀 Step-by-Step Testing Procedure

### Phase 1: Basic System Check (Without Navigation)

#### Terminal 1: Start Hardware Drivers

```bash
# Start pigpio daemon (if not running)
sudo systemctl start pigpiod

# Launch RPLidar
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser

# Verify /scan topic
ros2 topic hz /scan
# Should show ~5-10 Hz depending on LiDAR model
```

#### Terminal 2: Start Odometry

```bash
source ~/ros2_ws/install/setup.bash

# Launch odometry node
ros2 run rover_navigation odometry_node

# Verify /odom topic
ros2 topic echo /odom --once
# Should show current pose
```

#### Terminal 3: Start Motor Controller

```bash
source ~/ros2_ws/install/setup.bash

# Launch motor controller (CAREFUL - robot will move!)
ros2 run bts_motor_controller motor_node

# Test with manual velocity command (in safe area!)
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
# Robot should move forward slowly - STOP after 1-2 seconds!
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

**✅ Verification Checklist:**
- [ ] `/scan` topic publishing at ~5-10 Hz
- [ ] `/odom` topic publishing pose data
- [ ] Motor controller responds to `/cmd_vel` commands
- [ ] Robot moves when commanded (test carefully!)

---

### Phase 2: SLAM and Mapping Setup

#### Terminal 1: Start SLAM Stack

```bash
# Option A: Use existing full_mapping launch file
ros2 launch your_robot_bringup full_mapping.launch.py

# OR Option B: Launch components individually

# 1. RPLidar
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser

# 2. Robot State Publisher (with URDF)
ros2 run robot_state_publisher robot_state_publisher \
  /path/to/your/robot.urdf

# 3. EKF (after 1.5s delay)
sleep 1.5
ros2 run robot_localization ekf_node --ros-args \
  -p config_file:=/path/to/ekf.yaml

# 4. SLAM Toolbox (after 3.0s delay)
sleep 3.0
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/path/to/slam_params.yaml

# 5. RViz2 (optional, for visualization)
sleep 2.0
rviz2
```

#### Terminal 2: Monitor Topics

```bash
# Check map topic
ros2 topic hz /map
# Should start publishing after SLAM initializes

# Check TF tree
ros2 run tf2_tools view_frames
# Verify: map → odom → base_link → laser chain exists

# Visualize map
ros2 run rviz2 rviz2
# Add: Map display → /map topic
```

**✅ Verification Checklist:**
- [ ] `/map` topic publishing (nav_msgs/OccupancyGrid)
- [ ] TF tree complete: `map → odom → base_link → laser`
- [ ] Map visible in RViz2
- [ ] Robot pose visible on map

---

### Phase 3: Test A* Global Planner Only

#### Terminal 1: Start Prerequisites (from Phase 2)

Keep SLAM and mapping running.

#### Terminal 2: Launch A* Global Planner

```bash
source ~/ros2_ws/install/setup.bash

# Launch A* planner
ros2 run rover_navigation a_star_planner

# Monitor status
ros2 topic echo /planner/debug/state
```

#### Terminal 3: Test Planning

```bash
# Send a goal (adjust coordinates based on your map)
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map", stamp: {sec: 0, nanosec: 0}},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

# Check if path is generated
ros2 topic echo /global_path --once

# Visualize in RViz2
# Add: Path display → /global_path (color: blue)
```

**✅ Verification Checklist:**
- [ ] Planner receives `/map` topic
- [ ] Planner state changes to "PLANNING" then "ACTIVE"
- [ ] `/global_path` publishes with waypoints
- [ ] Path visible in RViz2 (blue line)
- [ ] Path avoids obstacles on map

---

### Phase 4: Test Full Navigation Stack (A* + LWB)

#### Terminal 1: Complete Infrastructure

```bash
# Start everything needed:
# 1. RPLidar (already running)
# 2. SLAM (already running)
# 3. EKF (already running)
# 4. Robot State Publisher (already running)
```

#### Terminal 2: Launch Navigation Stack

```bash
source ~/ros2_ws/install/setup.bash

# Launch complete navigation stack
ros2 launch rover_navigation a_star_lwb_navigation.launch.py

# Monitor planner status
ros2 topic echo /planner/debug/state
```

#### Terminal 3: Monitor Navigation

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor global path
ros2 topic echo /global_path --once

# Monitor local trajectory
ros2 topic echo /local_trajectory --once

# Check all topics
ros2 topic list | grep -E "global_path|local_trajectory|cmd_vel|planner"
```

#### Terminal 4: Send Goal and Observe

```bash
# Send goal (SAFE DISTANCE FIRST!)
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map", stamp: {sec: 0, nanosec: 0}},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},  # Close goal for first test!
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

# Watch robot navigate (keep hands near emergency stop!)
# Monitor cmd_vel to see velocity commands
# Check if robot reaches goal
```

#### Terminal 5: Visualization (Optional but Recommended)

```bash
rviz2
```

**RViz2 Displays to Add:**
1. **Map**: `/map` (OccupancyGrid)
2. **Global Path**: `/global_path` (Path, color: blue, line width: 0.05)
3. **Local Trajectory**: `/local_trajectory` (Path, color: green, line width: 0.03)
4. **Laser Scan**: `/scan` (LaserScan, color: red)
5. **Robot Pose Marker**: `/planner/debug/robot_pose` (Marker)
6. **Obstacle Markers**: `/planner/debug/obstacles` (MarkerArray)
7. **TF**: Show transforms (map → odom → base_link → laser)

**✅ Verification Checklist:**
- [ ] All navigation nodes start without errors
- [ ] `/global_path` generates when goal is sent
- [ ] `/cmd_vel` publishes velocity commands
- [ ] `/local_trajectory` shows sampled trajectories
- [ ] Robot moves toward goal
- [ ] Robot avoids obstacles (test by placing obstacle in path)
- [ ] Robot reaches goal (state changes to "GOAL_REACHED")

---

## 🔍 Detailed Verification Tests

### Test 1: Path Planning Validation

```bash
# Send multiple goals and verify paths
for x in 1.0 2.0 3.0; do
  ros2 topic pub -1 /goal geometry_msgs/PoseStamped "{
    header: {frame_id: 'map'},
    pose: {position: {x: $x, y: 0.5, z: 0.0}, orientation: {w: 1.0}}
  }"
  sleep 2
  ros2 topic echo /global_path --once | head -20
done
```

### Test 2: Obstacle Avoidance

1. **Place obstacle in robot's path**
2. Send goal beyond obstacle
3. Verify:
   - Global path avoids obstacle
   - Local planner adjusts trajectory
   - Robot navigates around obstacle

### Test 3: Replanning

```bash
# Send goal
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}
}'

# While robot is moving, send new goal
sleep 5
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}
}'

# Verify replanning occurs
ros2 topic echo /planner/debug/state
```

### Test 4: Parameter Tuning

```bash
# Test with different velocities
ros2 launch rover_navigation a_star_lwb_navigation.launch.py \
  max_vel_x:=0.3 \
  max_vel_theta:=0.8

# Test with different inflation radius
ros2 run rover_navigation a_star_planner \
  --ros-args -p inflation_radius:=0.8
```

---

## 🐛 Troubleshooting

### Problem: "/map topic not found"

**Solution:**
```bash
# Check if SLAM is running
ros2 node list | grep slam

# Check map topic
ros2 topic list | grep map

# Restart SLAM if needed
ros2 launch slam_toolbox online_async_launch.py
```

### Problem: "Cannot get robot pose" / TF errors

**Solution:**
```bash
# Check TF tree
ros2 run tf2_ros tf2_echo map base_link

# Verify robot_state_publisher is running
ros2 node list | grep robot_state_publisher

# Check transform chain
ros2 run tf2_tools view_frames
evince frames.pdf  # View transform tree
```

### Problem: "No path found" from A* planner

**Possible causes:**
1. Goal is in occupied/invalid cell
2. Start position is invalid
3. Map not loaded correctly

**Solution:**
```bash
# Check goal coordinates
ros2 topic echo /goal --once

# Check current robot pose
ros2 topic echo /odom --once

# Verify map has valid cells
ros2 topic echo /map --once | grep -A 5 "data"

# Try goal closer to current position
```

### Problem: Robot doesn't move / No cmd_vel

**Solution:**
```bash
# Check if LWB planner is running
ros2 node list | grep lwb

# Check if global_path exists
ros2 topic echo /global_path --once

# Check LWB planner logs
ros2 topic echo /cmd_vel
# Should publish commands at 15 Hz

# Verify motor controller is running
ros2 node list | grep motor
```

### Problem: Robot moves but doesn't reach goal

**Possible causes:**
1. Goal tolerance too small
2. Obstacles blocking path
3. Odometry drift

**Solution:**
```bash
# Check goal tolerance
ros2 param get /planner_manager goal_tolerance

# Monitor robot position vs goal
ros2 topic echo /odom --once
ros2 topic echo /goal --once

# Increase tolerance if needed
ros2 param set /planner_manager goal_tolerance 0.5
```

### Problem: High CPU usage

**Solution:**
```bash
# Reduce sampling density
ros2 run rover_navigation lwb_planner \
  --ros-args \
  -p vx_samples:=10 \
  -p vtheta_samples:=20

# Reduce planning frequency
ros2 run rover_navigation a_star_planner \
  --ros-args -p plan_frequency:=0.5
```

---

## 📊 Performance Monitoring

### Monitor Topics

```bash
# Topic frequencies
ros2 topic hz /scan          # Should be ~5-10 Hz
ros2 topic hz /map           # Should be ~0.1-1 Hz (map updates)
ros2 topic hz /cmd_vel       # Should be ~15 Hz (LWB planner rate)
ros2 topic hz /global_path   # Should be ~1 Hz (A* planner rate)
ros2 topic hz /local_trajectory  # Should be ~15 Hz
```

### Monitor CPU/Memory

```bash
# Check node resource usage
top -p $(pgrep -f "a_star_planner|lwb_planner")

# Or use htop
htop -p $(pgrep -f "a_star_planner|lwb_planner")
```

### Monitor Robot State

```bash
# Watch all planner topics
ros2 topic echo /planner/debug/state
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

---

## ⚠️ Safety Considerations

1. **Emergency Stop**: Always be ready to stop the robot
   ```bash
   # Emergency stop command
   ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{}'
   ```

2. **Test in Safe Area**: Start with small goals in open space

3. **Monitor First Run**: Watch robot closely on first autonomous navigation

4. **Clear Obstacles**: Ensure test area is clear of fragile items

5. **Battery Check**: Ensure sufficient battery for test duration

6. **LiDAR Safety**: Don't look directly into LiDAR beam

---

## 📝 Testing Checklist Summary

### Pre-Test
- [ ] All hardware connected and verified
- [ ] pigpio daemon running
- [ ] Package built successfully
- [ ] Dependencies installed

### Phase 1: Basic Hardware
- [ ] LiDAR publishing `/scan`
- [ ] Odometry publishing `/odom`
- [ ] Motor controller responding to `/cmd_vel`

### Phase 2: SLAM
- [ ] `/map` topic publishing
- [ ] TF tree complete
- [ ] Map visible in RViz2

### Phase 3: A* Planner
- [ ] Planner receives map
- [ ] Path generated for valid goals
- [ ] Path visible in RViz2

### Phase 4: Full Navigation
- [ ] All nodes start successfully
- [ ] Robot moves toward goal
- [ ] Obstacle avoidance works
- [ ] Goal reached successfully

---

## 🎯 Next Steps After Testing

1. **Tune Parameters**: Adjust velocities, costs, inflation based on test results
2. **Map Building**: Build complete map of your environment
3. **Waypoint Navigation**: Test navigation through multiple waypoints
4. **Long Duration Tests**: Test navigation reliability over time
5. **Dynamic Obstacles**: Test with moving obstacles (if applicable)

---

## 📚 Additional Resources

- `NAVIGATION_STACK_GUIDE.md` - Detailed architecture and parameter guide
- `README.md` - General package documentation
- ROS2 Navigation Stack documentation
- SLAM Toolbox documentation

---

**Good luck with testing! 🚀**












