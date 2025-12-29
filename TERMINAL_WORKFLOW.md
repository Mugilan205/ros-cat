# Terminal Workflow: Quick Reference

## 🎯 Quick Start (Automated)

### One Command to Start Everything:
```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```
**This opens 6 terminals automatically in the correct order!**

---

## 📺 Manual Terminal Setup (Detailed)

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                            │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  RPLidar         Wheel Encoders      BTS7960 Motors          │
│  (USB)           (Serial)            (GPIO)                  │
│     │                 │                  │                    │
│     └─────────────────┴──────────────────┘                    │
│                          │                                    │
│                    Raspberry Pi                                │
│                          │                                    │
└──────────────────────────┼────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 SOFTWARE LAYER                      │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Terminal 2: RPLidar Driver                                  │
│  ┌─────────────────────────────────────┐                     │
│  │ /scan (LaserScan)                   │                     │
│  └──────────────┬──────────────────────┘                     │
│                 │                                             │
│  Terminal 1: robot_state_publisher                           │
│  ┌─────────────────────────────────────┐                     │
│  │ /tf (base_link → laser)             │                     │
│  └──────────────┬──────────────────────┘                     │
│                 │                                             │
│  Terminal 3: Odometry Node                                   │
│  ┌─────────────────────────────────────┐                     │
│  │ /odom (PoseStamped)                 │                     │
│  └──────────────┬──────────────────────┘                     │
│                 │                                             │
│  Terminal 4: EKF                                             │
│  ┌─────────────────────────────────────┐                     │
│  │ /tf (odom → base_link)              │                     │
│  └──────────────┬──────────────────────┘                     │
│                 │                                             │
│  Terminal 5: SLAM Toolbox                                    │
│  ┌─────────────────────────────────────┐                     │
│  │ /map (OccupancyGrid)                │                     │
│  │ /tf (map → odom)                    │                     │
│  └──────────────┬──────────────────────┘                     │
│                 │                                             │
│  Terminal 8: Navigation Stack (Optional)                     │
│  ┌─────────────────────────────────────┐                     │
│  │ /global_path (Path)                 │                     │
│  │ /cmd_vel (Twist) ───────────────┐   │                     │
│  └─────────────────────────────────┼───┘                     │
│                                    │                         │
│  Terminal 4: Motor Controller                                │
│  ┌─────────────────────────────────────┐                     │
│  │ ← /cmd_vel                          │                     │
│  │ → GPIO (PWM signals to motors)      │                     │
│  └─────────────────────────────────────┘                     │
│                                                               │
│  Terminal 6: RViz2 (Visualization)                           │
│  ┌─────────────────────────────────────┐                     │
│  │ Visualizes: /map, /scan, TF tree    │                     │
│  └─────────────────────────────────────┘                     │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 📋 Terminal-by-Terminal Guide

### **Terminal 1: Robot State Publisher**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```

**What It Does:**
- Reads URDF robot description
- Publishes static transform: `base_link` → `laser`
- Tells ROS where LiDAR is mounted on robot
- **MUST START FIRST** (other nodes need this transform)

**Expected Output:**
```
[INFO] [robot_state_publisher]: Starting robot state publisher...
[INFO] [robot_state_publisher]: Publishing transforms...
```

**What to Watch For:**
- ✅ No errors about URDF file not found
- ✅ Transform publishing successfully

**Verify:**
```bash
# In another terminal
ros2 run tf2_ros tf2_echo base_link laser
# Should show transform immediately
```

---

### **Terminal 2: RPLidar Driver**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

**What It Does:**
- Opens serial communication with RPLidar on `/dev/ttyUSB0`
- Reads laser scan data from hardware
- Publishes to `/scan` topic (sensor_msgs/LaserScan)
- Each scan contains 360° distance measurements
- **Primary sensor for SLAM and obstacle detection**

**Expected Output:**
```
[INFO] [rplidar_composition]: RPLidar driver starting...
[INFO] [rplidar_composition]: RPLidar connected on /dev/ttyUSB0
[INFO] [rplidar_composition]: Publishing scan data...
[INFO] [rplidar_composition]: Scan frequency: ~5-10 Hz
```

**What to Watch For:**
- ✅ "RPLidar connected" message
- ✅ No serial port errors
- ✅ Scan data publishing regularly

**Verify:**
```bash
# In another terminal
ros2 topic hz /scan
# Expected: ~5-10 Hz (depends on LiDAR model)

ros2 topic echo /scan --once
# Should show LaserScan message with ranges[] array
```

---

### **Terminal 3: Odometry Node**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rover_navigation odometry_node
```

**What It Does:**
- Subscribes to `/cmd_vel` (velocity commands)
- Integrates velocity over time to estimate position
- Publishes current pose to `/odom` topic
- Provides odometry frame (`odom`) for localization
- **Dead reckoning** (drift over time without wheel encoders)

**Expected Output:**
```
[INFO] [odometry_node]: Odometry Node Started
[INFO] [odometry_node]:   Wheel Radius: 0.05 m
[INFO] [odometry_node]:   Wheel Base: 0.15 m
[INFO] [odometry_node]:   Encoder Resolution: 20 ticks/rev
[INFO] [odometry_node]: Publishing /odom at 20 Hz
```

**What to Watch For:**
- ✅ Node starts without errors
- ✅ Odometry values updating (even if at 0,0,0 initially)

**Verify:**
```bash
# In another terminal
ros2 topic hz /odom
# Expected: ~20 Hz

ros2 topic echo /odom --once
# Should show PoseStamped message with position/orientation
```

---

### **Terminal 4: EKF (Extended Kalman Filter)**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_localization ekf_node \
    --ros-args -p config_file:=/home/caterpillar/ros2_ws/config/ekf.yaml
```

**What It Does:**
- Fuses multiple sensor sources (odometry, IMU if available)
- Provides better localization than raw odometry
- Publishes `odom` → `base_link` transform
- Reduces drift and improves accuracy
- Uses Extended Kalman Filter algorithm

**Expected Output:**
```
[INFO] [ekf_filter_node]: EKF starting...
[INFO] [ekf_filter_node]: Subscribing to /odom_raw
[INFO] [ekf_filter_node]: Publishing odom → base_link transform at 30 Hz
[INFO] [ekf_filter_node]: EKF filter initialized
```

**What to Watch For:**
- ✅ No errors about config file
- ✅ Successfully subscribing to odometry input
- ✅ Transform publishing

**Verify:**
```bash
# In another terminal
ros2 run tf2_ros tf2_echo odom base_link
# Should show transform (may take a few seconds)
```

---

### **Terminal 5: SLAM Toolbox**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    -p scan_topic:=/scan \
    -p map_frame:=map \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p config_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml
```

**What It Does:**
- **Maps** the environment using LiDAR scans
- **Localizes** robot position within the map
- Publishes `/map` topic (OccupancyGrid) - the map!
- Publishes `map` → `odom` transform
- **REQUIRES robot movement** to initialize and build map
- Takes 10-30 seconds to initialize

**Expected Output:**
```
[INFO] [slam_toolbox]: SLAM Toolbox starting...
[INFO] [slam_toolbox]: Subscribing to /scan
[INFO] [slam_toolbox]: Waiting for transforms...
[INFO] [slam_toolbox]: Transforms ready
[INFO] [slam_toolbox]: Initializing map...
[INFO] [slam_toolbox]: SLAM initialized (may take 10-30 seconds)
[INFO] [slam_toolbox]: Publishing map updates...
[INFO] [slam_toolbox]: Map resolution: 0.05 m/pixel
```

**What to Watch For:**
- ✅ No "queue full" errors (means transforms are ready)
- ✅ "SLAM initialized" message
- ✅ Map publishing starts after initialization
- ⚠️ **Robot MUST move** for SLAM to initialize properly

**Verify:**
```bash
# Wait 10-30 seconds, then:
ros2 topic hz /map
# Expected: ~0.1-1 Hz (slow updates)

ros2 topic echo /map --once
# Should show OccupancyGrid with map data
```

---

### **Terminal 6: RViz2 (Visualization)**

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

**What It Does:**
- Opens GUI visualization window
- Shows map, laser scans, robot pose, transforms
- **Essential for debugging and monitoring**

**Setup RViz2:**
1. **Set Fixed Frame**: `map` (top dropdown)
2. **Add → Map**:
   - Topic: `/map`
   - Color Scheme: `costmap`
3. **Add → LaserScan**:
   - Topic: `/scan`
   - Color: Red
   - Size: 0.05
4. **Add → TF**:
   - Show Names: Enabled
   - Show Axes: Enabled

**What You Should See:**
- **Map**: Gray/black/white grid showing obstacles and free space
- **Laser Scan**: Red dots showing current obstacles
- **Robot**: Coordinate frame showing robot position and orientation
- **TF Tree**: Lines showing transform relationships

---

### **Terminal 7: Motor Controller** ⚠️

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run bts_motor_controller motor_node
```

**What It Does:**
- Subscribes to `/cmd_vel` topic
- Converts linear.x and angular.z to motor commands
- **Differential Drive Control**:
  - `left_motor = linear.x - angular.z`
  - `right_motor = linear.x + angular.z`
- Sends PWM signals to BTS7960 via GPIO pins
- **ROBOT WILL MOVE** when receiving commands!

**Expected Output:**
```
[INFO] [motor_controller]: Motor Controller Started
[INFO] [motor_controller]: Waiting for /cmd_vel commands...
```

**What to Watch For:**
- ✅ "pigpio daemon" connected (not an error)
- ✅ No GPIO permission errors
- ⚠️ Robot will respond to velocity commands

**Test (CAREFUL!):**
```bash
# In another terminal - Small test movement
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.1}, angular: {z: 0.0}}'

# STOP immediately!
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

---

### **Terminal 8: Navigation Stack** (Optional)

**Command:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rover_navigation a_star_lwb_navigation.launch.py
```

**What It Does:**
- **A* Global Planner**: Plans optimal path from current position to goal
- **LWB Local Planner**: Generates smooth velocity commands
- **Planner Manager**: Coordinates planning and execution
- Subscribes to `/map` and `/odom`
- Publishes `/global_path` (planned path)
- Publishes `/cmd_vel` (velocity commands → motor controller)

**Expected Output:**
```
[INFO] [a_star_planner]: A* Global Planner started
[INFO] [a_star_planner]: Subscribing to /map
[INFO] [lwb_planner]: LWB Local Planner started
[INFO] [planner_manager]: Planner Manager started
[INFO] [navigation_dashboard]: Navigation Dashboard started
```

**What to Watch For:**
- ✅ All nodes start successfully
- ✅ No errors about missing topics
- ✅ Planner ready to receive goals

---

### **Terminal 9: Send Navigation Goal** (Optional)

**Command:**
```bash
# Send goal to navigation stack
ros2 topic pub -1 /goal geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

**What Happens:**
1. A* planner generates path to goal
2. LWB planner generates velocity commands
3. Robot moves toward goal
4. Robot avoids obstacles

**Monitor:**
```bash
# Check planner status
ros2 topic echo /planner/debug/state

# Check velocity commands
ros2 topic echo /cmd_vel

# Check path
ros2 topic echo /global_path --once
```

---

## 🔄 Startup Order (Critical!)

**Must start in this order:**
1. **Terminal 1**: robot_state_publisher (immediately)
2. **Terminal 2**: RPLidar (wait 2 seconds after Terminal 1)
3. **Terminal 3**: Odometry (wait 2 seconds after Terminal 2)
4. **Terminal 4**: EKF (wait 2 seconds after Terminal 3)
5. **Terminal 5**: SLAM Toolbox (wait 3 seconds after Terminal 4)
6. **Terminal 6**: RViz2 (optional, can start anytime)
7. **Terminal 4**: Motor Controller (can start anytime, but robot will move!)
8. **Terminal 8**: Navigation Stack (only after SLAM is working)

**Why this order matters:**
- SLAM needs TF transforms to be ready
- EKF needs odometry input
- robot_state_publisher must be first (others depend on it)

---

## ✅ Quick Verification Checklist

After starting all terminals:

```bash
# 1. Check all topics are publishing
ros2 topic list | grep -E "scan|map|odom|cmd_vel"

# 2. Check TF transforms exist
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom

# 3. Check topic frequencies
ros2 topic hz /scan    # Should be ~5-10 Hz
ros2 topic hz /map     # Should be ~0.1-1 Hz
ros2 topic hz /odom    # Should be ~20 Hz

# 4. View complete TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## 🆘 Emergency Stop

**If robot is moving unexpectedly:**
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'
```

This sends zero velocity command to stop the robot immediately.

---

## 📊 Data Flow Summary

```
Hardware Sensors
    │
    ├─ RPLidar → Terminal 2 → /scan
    │
    └─ Encoders → (optional) → /odom
         │
         │
    Terminal 1: robot_state_publisher
         │
         └─ /tf (base_link → laser)
              │
              │
    Terminal 3: Odometry Node
         │
         └─ /odom
              │
              │
    Terminal 4: EKF
         │
         └─ /tf (odom → base_link)
              │
              │
    Terminal 5: SLAM Toolbox
         │
         ├─ /map (OccupancyGrid)
         └─ /tf (map → odom)
              │
              │
    Terminal 8: Navigation Stack (optional)
         │
         ├─ /global_path
         └─ /cmd_vel
              │
              │
    Terminal 4: Motor Controller
         │
         └─ GPIO → Motors → Robot Movement
```

---

## 🎯 What Each Terminal Outputs

| Terminal | Key Output | Frequency | Purpose |
|----------|------------|-----------|---------|
| 1 | `/tf` (base_link→laser) | Continuous | Robot structure |
| 2 | `/scan` (LaserScan) | 5-10 Hz | Sensor data |
| 3 | `/odom` (PoseStamped) | 20 Hz | Robot position |
| 4 | `/tf` (odom→base_link) | 30 Hz | Localization |
| 5 | `/map` (OccupancyGrid) | 0.1-1 Hz | Environment map |
| 5 | `/tf` (map→odom) | Continuous | SLAM localization |
| 6 | Visual display | GUI | Debugging |
| 4 | GPIO PWM signals | Variable | Motor control |
| 8 | `/global_path` | 1 Hz | Planned path |
| 8 | `/cmd_vel` | 15 Hz | Velocity commands |

---

**See `COMPLETE_TESTING_GUIDE.md` for detailed testing procedures!**




