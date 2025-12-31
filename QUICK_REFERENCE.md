# Quick Reference: ROS2 Rover Navigation System

## 🚀 One-Liner Quick Start

```bash
cd ~/ros2_ws && ./start_slam_mapping.sh  # Starts everything automatically!
```

---

## 📁 Where Things Are

### Configuration
- **EKF Config**: `config/ekf.yaml`
- **SLAM Config**: `config/slam_params.yaml`
- **Robot Description**: `urdf/your_robot.urdf`

### Startup Scripts
- **Automated SLAM**: `start_slam_mapping.sh` ⭐
- **Launch File**: `launch/full_mapping.launch.py`

### Core Packages
- **Motor Control**: `src/bts_motor_controller/`
- **Navigation**: `src/rover_navigation/` ⭐
- **LiDAR Driver**: `src/rplidar_ros/`

### Documentation (NEW!)
- **Testing Guide**: `COMPLETE_TESTING_GUIDE.md` ⭐
- **Terminal Guide**: `TERMINAL_WORKFLOW.md` ⭐
- **Quick Summary**: `TESTING_SUMMARY.md` ⭐
- **File Tree**: `CODEBASE_FILE_TREE.md` ⭐

---

## 🎯 Key Commands

### Build & Setup
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Start SLAM (Automated)
```bash
./start_slam_mapping.sh
```

### Start Navigation
```bash
ros2 launch rover_navigation a_star_lwb_navigation.launch.py
```

### Start Individual Components
```bash
# LiDAR
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser

# Motor Controller
ros2 run bts_motor_controller motor_node

# Odometry
ros2 run rover_navigation odometry_node
```

---

## 📊 Key Topics

| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/scan` | LaserScan | RPLidar | Laser measurements |
| `/odom` | PoseStamped | Odometry Node | Robot position |
| `/map` | OccupancyGrid | SLAM Toolbox | Environment map |
| `/cmd_vel` | Twist | Navigation/LWB | Velocity commands |
| `/global_path` | Path | A* Planner | Planned path |
| `/tf` | Transform | Various | Robot structure |

---

## 🔧 Hardware Requirements

1. **Raspberry Pi** (ROS2 Jazzy)
2. **RPLidar A1/A2** → `/dev/ttyUSB0`
3. **BTS7960 Motor Driver** → GPIO 12, 5, 13, 6
4. **Differential Drive Robot**
5. **Wheel Encoders** (optional) → `/dev/ttyACM0`

---

## 📺 Terminal Setup (6 Terminals Minimum)

1. **robot_state_publisher** - Robot structure
2. **RPLidar** - Laser scanner
3. **Odometry** - Position tracking
4. **EKF** - Sensor fusion
5. **SLAM Toolbox** - Mapping
6. **RViz2** - Visualization
7. **Motor Controller** - Motor control (when moving)
8. **Navigation Stack** - Path planning (optional)

See `TERMINAL_WORKFLOW.md` for details!

---

## 🆘 Emergency Stop

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'
```

---

## 📚 Documentation Hierarchy

```
START HERE
  ↓
TESTING_SUMMARY.md (Quick overview)
  ↓
COMPLETE_TESTING_GUIDE.md (Full procedures)
  ↓
TERMINAL_WORKFLOW.md (Terminal details)
  ↓
CODEBASE_FILE_TREE.md (File structure)
  ↓
src/rover_navigation/TECHNICAL_GUIDE.md (Deep dive)
```

---

## ✅ Testing Checklist

- [ ] Hardware connected
- [ ] pigpio daemon running
- [ ] Workspace built
- [ ] `/scan` publishing
- [ ] `/odom` publishing
- [ ] `/map` publishing (after SLAM init)
- [ ] TF transforms exist
- [ ] Robot moves when commanded

---

## 🎯 System Capabilities

✅ **SLAM** - Builds maps from LiDAR  
✅ **Localization** - Tracks robot position  
✅ **Path Planning** - A* global + LWB local  
✅ **Obstacle Avoidance** - Real-time detection  
✅ **Motor Control** - GPIO-based differential drive  

---

**See `COMPLETE_TESTING_GUIDE.md` for full details!**











