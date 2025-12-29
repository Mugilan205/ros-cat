# Complete Testing Guide: ROS2 Rover Navigation System

## 📋 System Overview

This is a **ROS2 Jazzy-based autonomous rover system** with the following capabilities:

1. **SLAM (Simultaneous Localization and Mapping)** - Builds maps while localizing
2. **Path Planning** - A* global planner + LWB (Local Weighted Band) local planner
3. **Obstacle Avoidance** - Real-time LiDAR-based obstacle detection
4. **Motor Control** - BTS7960 motor driver for differential drive
5. **Odometry** - Wheel encoder-based position tracking

---

## 🔧 Hardware Requirements

### 1. **Raspberry Pi** (Primary Computer)
- Running ROS2 Jazzy
- Network connectivity (for SSH/remote access)
- GPIO pins available for motor control

### 2. **RPLidar A1/A2** (Laser Scanner)
- **Connection**: USB port → `/dev/ttyUSB0`
- **Baud Rate**: 115200
- **Power**: 5V USB power
- **Frame ID**: `laser`
- **Purpose**: Provides 360° laser scan data for SLAM and obstacle detection

### 3. **BTS7960 Motor Controller** (Motor Driver)
- **GPIO12** → Left Motor PWM (`L_PWM`)
- **GPIO5** → Left Motor Direction (`L_DIR`)
- **GPIO13** → Right Motor PWM (`R_PWM`)
- **GPIO6** → Right Motor Direction (`R_DIR`)
- **Purpose**: Controls differential drive motors
- **Power**: External power supply to motors

### 4. **Wheel Encoders** (Optional but Recommended)
- **Connection**: Arduino/ESP8266 → `/dev/ttyACM0` (Serial, 115200 baud)
- **Purpose**: Provides wheel odometry for better position tracking
- **Alternative**: System can use `cmd_vel` integration for odometry (less accurate)

### 5. **Differential Drive Robot Chassis**
- Two wheels with motors
- Wheel radius: ~5cm (configurable)
- Wheel base: ~15cm distance between wheels (configurable)

---

## 💻 Software Prerequisites

### Required ROS2 Packages
```bash
# Install ROS2 Jazzy packages
sudo apt update
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-robot-state-publisher
```

### Python Dependencies
```bash
sudo apt install -y python3-numpy python3-scipy python3-serial
```

### System Services
```bash
# Start pigpio daemon for GPIO control
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
sudo systemctl status pigpiod  # Verify it's running
```

### Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 Testing Procedure: Step-by-Step

### Phase 1: Hardware Verification (No Robot Movement)

#### Terminal 1: Check LiDAR Connection
```bash
# Verify LiDAR device exists
ls -l /dev/ttyUSB0
# Expected: crw-rw---- 1 root dialout 188, 0 ...

# Check user permissions
groups | grep dialout
# If not in dialout group: sudo usermod -a -G dialout $USER (then logout/login)
```

**Expected Output:**
- Device file exists
- You have read/write permissions (either as root or in dialout group)

---

#### Terminal 2: Check Motor Controller (pigpio)
```bash
# Check pigpio daemon status
sudo systemctl status pigpiod

# Test GPIO access (optional - be careful!)
# This should not cause motor movement if no motors are connected
pigs t 12  # Test GPIO12
```

**Expected Output:**
```
● pigpiod.service - pigpio daemon
   Active: active (running) since ...
```

---

#### Terminal 3: Check Serial Port (Encoders - Optional)
```bash
# Check if encoder device exists (Arduino/ESP8266)
ls -l /dev/ttyACM0

# If device exists, test serial communication
# (Be careful - don't send commands that might cause issues)
```

**Expected Output:**
- If encoders connected: Device file exists
- If no encoders: Device not found (OK - odometry will use cmd_vel integration)

---

### Phase 2: Test Individual Components

#### Terminal 1: Test RPLidar Driver
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch RPLidar
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

**Expected Output:**
```
[INFO] [rplidar_composition]: RPLidar driver starting...
[INFO] [rplidar_composition]: RPLidar connected on /dev/ttyUSB0
[INFO] [rplidar_composition]: Publishing scan data...
```

**Verify in another terminal:**
```bash
# Check scan topic is publishing
ros2 topic hz /scan
# Expected: ~5-10 Hz (depends on LiDAR model)

# View scan data
ros2 topic echo /scan --once
# Expected: LaserScan message with ranges[] array
```

**What This Terminal Does:**
- Starts the RPLidar driver node
- Opens serial communication with LiDAR on `/dev/ttyUSB0`
- Publishes laser scan data to `/scan` topic at ~5-10 Hz
- Each scan contains 360° distance measurements
- **Keep this terminal running** - this is your primary sensor

---

#### Terminal 2: Test Robot State Publisher (TF)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start robot state publisher with URDF
ros2 run robot_state_publisher robot_state_publisher \
    /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```

**Expected Output:**
```
[INFO] [robot_state_publisher]: Starting robot state publisher...
[INFO] [robot_state_publisher]: Publishing transforms...
```

**Verify transform:**
```bash
# In another terminal
ros2 run tf2_ros tf2_echo base_link laser
# Expected: Transform from base_link to laser
# Translation: x: 0.12, y: 0.0, z: 0.25 (from URDF)
# Rotation: (0, 0, 0, 1) - quaternion
```

**What This Terminal Does:**
- Reads the URDF robot description file
- Publishes static transform: `base_link` → `laser`
- This transform tells ROS where the LiDAR is mounted on the robot
- Essential for SLAM to correctly interpret scan data
- **Keep this terminal running**

---

#### Terminal 3: Test Odometry Node
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start odometry node
ros2 run rover_navigation odometry_node
```

**Expected Output:**
```
[INFO] [odometry_node]: Odometry Node Started
[INFO] [odometry_node]:   Wheel Radius: 0.05 m
[INFO] [odometry_node]:   Wheel Base: 0.15 m
[INFO] [odometry_node]:   Encoder Resolution: 20 ticks/rev
```

**Verify odometry:**
```bash
# In another terminal
ros2 topic echo /odom --once
# Expected: PoseStamped message with position and orientation
# Initially at (0, 0, 0) if robot hasn't moved
```

**What This Terminal Does:**
- Subscribes to `/cmd_vel` (velocity commands)
- Integrates velocity over time to estimate robot position (x, y, theta)
- Publishes current pose to `/odom` topic at 20 Hz
- Provides odometry frame (`odom`) for localization
- **Note**: Without wheel encoders, this uses dead reckoning (drift over time)
- **Keep this terminal running**

---

#### Terminal 4: Test Motor Controller (⚠️ CAREFUL - Robot Will Move!)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start motor controller
ros2 run bts_motor_controller motor_node
```

**Expected Output:**
```
[INFO] [motor_controller]: Motor Controller Started
[INFO] [motor_controller]: Waiting for /cmd_vel commands...
```

**Test with SMALL movement (in safe area!):**
```bash
# In another terminal - Move forward slowly for 1 second
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.1}, angular: {z: 0.0}}'

# STOP IMMEDIATELY after testing!
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

**What This Terminal Does:**
- Subscribes to `/cmd_vel` topic (Twist messages)
- Converts linear.x and angular.z to left/right motor speeds
- **Differential Drive Math**: 
  - `left_motor = linear.x - angular.z`
  - `right_motor = linear.x + angular.z`
- Sends PWM signals to BTS7960 via GPIO pins
- Controls motor direction and speed
- **Keep this terminal running** (but robot should be stopped)

---

### Phase 3: SLAM Mapping Stack

This is the main mapping system. Use the automated script OR manual launch.

#### Option A: Automated Script (Recommended)

```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```

**What This Script Does:**
- Opens 6 separate terminals (gnome-terminal)
- Starts components in correct order with delays:
  1. **Terminal 1**: robot_state_publisher (immediately)
  2. **Terminal 2**: RPLidar (after 2 seconds)
  3. **Terminal 3**: Odometry node (after 3 seconds)
  4. **Terminal 4**: EKF (after 5 seconds)
  5. **Terminal 5**: SLAM Toolbox (after 8 seconds)
  6. **Terminal 6**: RViz2 visualization (after 11 seconds)

---

#### Option B: Manual Launch (For Control)

**Terminal 1: robot_state_publisher**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```
**Expected Output:**
```
[INFO] [robot_state_publisher]: Publishing transforms...
```
**What It Does:** Publishes `base_link` → `laser` transform from URDF

---

**Terminal 2: RPLidar**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```
**Expected Output:**
```
[INFO] [rplidar_composition]: RPLidar connected
[INFO] [rplidar_composition]: Publishing scan data at ~5-10 Hz
```
**What It Does:** Publishes `/scan` topic with laser measurements

---

**Terminal 3: Odometry**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rover_navigation odometry_node
```
**Expected Output:**
```
[INFO] [odometry_node]: Odometry Node Started
[INFO] [odometry_node]: Publishing /odom at 20 Hz
```
**What It Does:** Publishes `/odom` with robot pose (from cmd_vel integration)

---

**Terminal 4: EKF (Extended Kalman Filter)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_localization ekf_node \
    --ros-args -p config_file:=/home/caterpillar/ros2_ws/config/ekf.yaml
```
**Expected Output:**
```
[INFO] [ekf_filter_node]: EKF starting...
[INFO] [ekf_filter_node]: Subscribing to /odom_raw
[INFO] [ekf_filter_node]: Publishing odom → base_link transform
```
**What It Does:**
- Fuses odometry data (sensor fusion)
- Publishes `odom` → `base_link` transform
- Improves localization accuracy
- Reads config from `config/ekf.yaml`

---

**Terminal 5: SLAM Toolbox**
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
**Expected Output:**
```
[INFO] [slam_toolbox]: SLAM Toolbox starting...
[INFO] [slam_toolbox]: Subscribing to /scan
[INFO] [slam_toolbox]: Waiting for transforms...
[INFO] [slam_toolbox]: Initializing map...
[INFO] [slam_toolbox]: SLAM initialized (may take 10-30 seconds)
[INFO] [slam_toolbox]: Publishing map...
```
**What It Does:**
- **Maps** the environment using LiDAR scans (`/scan`)
- **Localizes** the robot in the map using odometry
- Publishes `/map` topic (OccupancyGrid) at ~0.1-1 Hz
- Publishes `map` → `odom` transform
- **Requires robot movement** to initialize and build map

**Verify SLAM:**
```bash
# Wait 10-30 seconds for SLAM to initialize
# Then check map topic
ros2 topic hz /map
# Expected: ~0.1-1 Hz (slow updates)

# View map data
ros2 topic echo /map --once
# Expected: OccupancyGrid message with map data
```

---

**Terminal 6: RViz2 (Visualization)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```
**Expected Output:**
- RViz2 window opens
- Initially shows empty visualization

**Configure RViz2:**
1. **Set Fixed Frame**: `map` (in left panel)
2. **Add Display → Map**:
   - Topic: `/map`
   - Color Scheme: `costmap`
3. **Add Display → LaserScan**:
   - Topic: `/scan`
   - Color: Red
   - Size: 0.05
4. **Add Display → TF**:
   - Show Names: Enabled
   - Show Axes: Enabled

**What You Should See:**
- Map being built in real-time as robot moves
- Laser scan points (red dots) showing obstacles
- Robot position (base_link frame)
- Transform tree visualization

---

### Phase 4: Verify Complete System

#### Terminal 7: Verification Terminal
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check all transforms exist
echo "=== Checking TF Transforms ==="
ros2 run tf2_ros tf2_echo base_link laser        # Should work
ros2 run tf2_ros tf2_echo odom base_link         # Should work (from EKF)
ros2 run tf2_ros tf2_echo map odom               # Should work (from SLAM)

# View complete TF tree
ros2 run tf2_tools view_frames
evince frames.pdf  # Opens PDF showing transform tree

# Check all topics
echo "=== Checking Topics ==="
ros2 topic list | grep -E "scan|map|odom|cmd_vel"

# Check topic frequencies
echo "=== Topic Frequencies ==="
ros2 topic hz /scan     # Should be ~5-10 Hz
ros2 topic hz /map      # Should be ~0.1-1 Hz
ros2 topic hz /odom     # Should be ~20 Hz
```

**Expected Output:**
- All transforms exist and are publishing
- All topics are active
- Frequencies match expected values

---

### Phase 5: Test Navigation Stack (Optional)

Once SLAM is working and map is built:

**Terminal 8: Start Navigation Stack**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch full navigation stack (A* + LWB)
ros2 launch rover_navigation a_star_lwb_navigation.launch.py
```

**Expected Output:**
```
[INFO] [a_star_planner]: A* Global Planner started
[INFO] [lwb_planner]: LWB Local Planner started
[INFO] [planner_manager]: Planner Manager started
[INFO] [navigation_dashboard]: Navigation Dashboard started
```

**What This Terminal Does:**
- **A* Global Planner**: Plans optimal path from current position to goal
- **LWB Local Planner**: Generates smooth velocity commands following the path
- **Planner Manager**: Coordinates planning and execution
- Subscribes to `/map` and `/odom`
- Publishes `/global_path` and `/cmd_vel`

---

**Terminal 9: Send Navigation Goal (⚠️ Robot Will Move!)**
```bash
# Send goal (adjust coordinates based on your map)
ros2 topic pub -1 /goal geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

**What Happens:**
1. A* planner generates path from current position to goal
2. LWB planner generates velocity commands following the path
3. `/cmd_vel` commands are sent to motor controller
4. Robot moves toward goal
5. Robot avoids obstacles using LiDAR data

**Monitor Navigation:**
```bash
# Check planner status
ros2 topic echo /planner/debug/state

# Check velocity commands
ros2 topic echo /cmd_vel

# Check global path
ros2 topic echo /global_path --once
```

---

## 📊 Expected Outputs Summary

### Terminal Outputs at a Glance

| Terminal | Component | Key Messages | Topic Published | Frequency |
|----------|-----------|--------------|-----------------|-----------|
| 1 | robot_state_publisher | "Publishing transforms..." | `/tf` (base_link→laser) | Continuous |
| 2 | RPLidar | "RPLidar connected" | `/scan` | 5-10 Hz |
| 3 | Odometry | "Odometry Node Started" | `/odom` | 20 Hz |
| 4 | EKF | "EKF starting..." | `/tf` (odom→base_link) | 30 Hz |
| 5 | SLAM Toolbox | "SLAM initialized" | `/map` | 0.1-1 Hz |
| 6 | RViz2 | GUI window | N/A | N/A |
| 7 | Verification | Command outputs | N/A | N/A |
| 8 | Navigation Stack | "A* Planner started" | `/global_path`, `/cmd_vel` | 1 Hz, 15 Hz |
| 9 | Goal Command | Command sent | N/A | On-demand |

---

## 🔍 Troubleshooting

### Problem: `/scan` topic not publishing
**Check:**
```bash
ls -l /dev/ttyUSB0  # Device exists?
sudo chmod 666 /dev/ttyUSB0  # Try fixing permissions
ros2 topic list | grep scan  # Topic exists?
```

### Problem: Motor controller errors
**Check:**
```bash
sudo systemctl status pigpiod  # pigpio daemon running?
groups | grep gpio  # User in gpio group?
```

### Problem: SLAM not initializing
**Check:**
- Robot needs to **move** for SLAM to initialize
- All TF transforms must exist (base_link→laser, odom→base_link)
- Check SLAM logs for errors
- Wait 10-30 seconds for initialization

### Problem: No `/map` topic
**Check:**
```bash
ros2 node list | grep slam  # SLAM node running?
ros2 topic list | grep map  # Topic exists?
# Wait 10-30 seconds after SLAM starts
# Robot must move slightly for SLAM to initialize
```

### Problem: TF transform errors
**Check:**
```bash
ros2 run tf2_tools view_frames  # View complete TF tree
ros2 topic echo /tf --once  # Check if transforms are publishing
# Verify robot_state_publisher is running
ros2 node list | grep robot_state_publisher
```

---

## ⚠️ Safety Considerations

1. **Emergency Stop**: Always have this ready:
   ```bash
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'
   ```

2. **Test Area**: Start in open, safe area with no obstacles

3. **First Tests**: Use small velocity commands (0.1 m/s max)

4. **Supervision**: Always supervise first autonomous navigation

5. **Battery**: Ensure sufficient battery for test duration

6. **LiDAR Safety**: Don't look directly into LiDAR beam

---

## ✅ Testing Checklist

### Pre-Test
- [ ] All hardware connected
- [ ] pigpio daemon running
- [ ] ROS2 workspace built
- [ ] Dependencies installed

### Phase 1: Hardware
- [ ] `/dev/ttyUSB0` exists (LiDAR)
- [ ] pigpio daemon active
- [ ] Serial port exists (if using encoders)

### Phase 2: Components
- [ ] `/scan` publishing (~5-10 Hz)
- [ ] `/odom` publishing (~20 Hz)
- [ ] Motor controller responds to `/cmd_vel`
- [ ] TF transforms exist

### Phase 3: SLAM
- [ ] `/map` topic publishing (~0.1-1 Hz)
- [ ] Complete TF chain: `map → odom → base_link → laser`
- [ ] Map visible in RViz2
- [ ] Robot pose visible on map

### Phase 4: Navigation (Optional)
- [ ] Navigation nodes start successfully
- [ ] `/global_path` generates for valid goals
- [ ] Robot moves toward goal
- [ ] Obstacle avoidance works
- [ ] Goal reached successfully

---

## 📚 Additional Resources

- `QUICK_START_MAPPING.md` - Quick reference for SLAM
- `src/rover_navigation/TESTING_GUIDE.md` - Detailed navigation testing
- `src/rover_navigation/README.md` - Package documentation
- `config/ekf.yaml` - EKF configuration
- `config/slam_params.yaml` - SLAM configuration

---

**Good luck with testing! 🚀**


