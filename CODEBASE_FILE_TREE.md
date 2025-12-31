# Complete Codebase File Tree & Documentation Guide

This document provides a complete overview of the ROS2 Rover Navigation codebase structure, showing what exists, what's been created, and what you need to know.

---

## 🆕 Files Created by Documentation

The following files were created to help you understand and test the system:

- **`COMPLETE_TESTING_GUIDE.md`** - Comprehensive testing procedures
- **`TERMINAL_WORKFLOW.md`** - Detailed terminal-by-terminal explanation
- **`TESTING_SUMMARY.md`** - Quick reference summary
- **`CODEBASE_FILE_TREE.md`** - This file

---

## 📁 Complete Directory Structure

```
ros2_ws/                                    # ROS2 Workspace Root
│
├── 🆕 COMPLETE_TESTING_GUIDE.md            # Complete testing guide (NEW)
├── 🆕 TERMINAL_WORKFLOW.md                 # Terminal workflow details (NEW)
├── 🆕 TESTING_SUMMARY.md                   # Quick testing summary (NEW)
├── 🆕 CODEBASE_FILE_TREE.md                # This file (NEW)
│
├── README.md                               # Basic workspace README
├── QUICK_START_MAPPING.md                  # Quick SLAM mapping guide
├── FIX_SLAM_TF_ISSUE.md                    # SLAM TF issue documentation
├── START_MAPPING_FIXED.md                  # Fixed mapping startup guide
├── SOLUTION_SUMMARY.md                     # Solution summary for issues
│
├── ARDUINO_TEST_SKETCH.ino                 # Arduino test sketch for encoders
├── start_slam_mapping.sh                   # ⭐ Automated SLAM startup script
├── start_mapping.sh                        # Alternative mapping startup script
├── rplidar.log                             # RPLidar log file
├── test                                    # Test file
├── test.pub                                # Test publish file
│
├── config/                                 # Configuration Files Directory
│   ├── ekf.yaml                            # ⭐ EKF (Extended Kalman Filter) config
│   └── slam_params.yaml                    # ⭐ SLAM Toolbox parameters
│
├── launch/                                 # Launch Files Directory
│   ├── full_mapping.launch.py              # ⭐ Complete SLAM mapping launch file
│   └── __pycache__/                        # Python cache
│
├── urdf/                                   # Robot Description Directory
│   └── your_robot.urdf                     # ⭐ Robot URDF description (defines robot structure)
│
├── frames_*.gv                             # TF tree graph files (generated)
├── frames_*.pdf                            # TF tree PDFs (generated)
│
├── src/                                    # Source Code Directory
│   │
│   ├── bts_motor_controller/               # ⭐ BTS7960 Motor Controller Package
│   │   ├── package.xml                     # Package metadata
│   │   ├── setup.py                        # Python package setup
│   │   ├── setup.cfg                       # Setup configuration
│   │   ├── resource/
│   │   │   └── bts_motor_controller        # Resource file
│   │   ├── bts_motor_controller/           # Python package directory
│   │   │   ├── __init__.py
│   │   │   └── motor_node.py               # ⭐ Main motor controller node (GPIO control)
│   │   └── test/                           # Test files
│   │       ├── test_copyright.py
│   │       ├── test_flake8.py
│   │       └── test_pep257.py
│   │
│   ├── rover_navigation/                   # ⭐ Main Navigation Package
│   │   ├── package.xml                     # Package metadata
│   │   ├── setup.py                        # Python package setup
│   │   ├── setup.cfg                       # Setup configuration
│   │   ├── resource/
│   │   │   └── rover_navigation            # Resource file
│   │   │
│   │   ├── rover_navigation/               # Python package directory
│   │   │   ├── __init__.py
│   │   │   ├── __pycache__/                # Python cache
│   │   │   │
│   │   │   ├── odometry_node.py            # ⭐ Odometry tracking node
│   │   │   ├── obstacle_detection.py       # ⭐ LiDAR obstacle detection node
│   │   │   ├── rrt_planner.py              # ⭐ RRT path planning node
│   │   │   ├── path_executor.py            # ⭐ Path execution node (velocity control)
│   │   │   │
│   │   │   ├── global_planner_a_star.py    # ⭐ A* global path planner
│   │   │   ├── local_planner_lwb.py        # ⭐ LWB (Local Weighted Band) local planner
│   │   │   ├── planner_manager.py          # ⭐ Navigation planner manager
│   │   │   ├── costmap.py                  # Costmap implementation
│   │   │   ├── dashboard_node.py           # Navigation dashboard
│   │   │   ├── esp8266_bridge.py           # ESP8266 encoder bridge (optional)
│   │   │   │
│   │   ├── launch/                         # Launch Files
│   │   │   ├── obstacle_detection.launch.py      # Obstacle detection only
│   │   │   ├── rrt_planner.launch.py             # RRT planner only
│   │   │   ├── full_navigation.launch.py         # Complete RRT navigation stack
│   │   │   └── a_star_lwb_navigation.launch.py   # ⭐ A* + LWB navigation stack
│   │   │
│   │   ├── monitor_rover.py                # Rover monitoring script
│   │   ├── test_navigation.py              # Navigation testing script
│   │   │
│   │   ├── QUICK_MONITOR.sh                # Quick monitoring script
│   │   ├── QUICK_TEST.sh                   # Quick test script
│   │   ├── EXAMPLE_COMMANDS.sh             # Example command script
│   │   │
│   │   ├── 00_START_HERE.md                # Navigation package overview
│   │   ├── README.md                       # ⭐ Navigation package README
│   │   ├── QUICKSTART.md                   # Quick start guide
│   │   ├── IMPLEMENTATION_SUMMARY.md       # Implementation summary
│   │   ├── TECHNICAL_GUIDE.md              # ⭐ Technical deep dive
│   │   ├── TESTING_GUIDE.md                # Testing guide for navigation
│   │   ├── NAVIGATION_STACK_GUIDE.md       # ⭐ Navigation stack architecture
│   │   ├── MONITORING_GUIDE.md             # Monitoring guide
│   │   ├── HOW_TO_MONITOR.md               # How to monitor rover
│   │   └── SEE_ENCODER_VALUES.md           # Encoder values guide
│   │
│   ├── rplidar_ros/                        # RPLidar ROS2 Driver Package (External)
│   │   ├── package.xml                     # Package metadata
│   │   ├── CMakeLists.txt                  # CMake build configuration
│   │   ├── LICENSE                         # License file
│   │   ├── README.md                       # RPLidar driver README
│   │   ├── CHANGELOG.rst                   # Changelog
│   │   │
│   │   ├── include/                        # C++ Headers
│   │   │   ├── rplidar_node.hpp            # RPLidar node header
│   │   │   └── visibility.h                # Visibility control
│   │   │
│   │   ├── src/                            # C++ Source Files
│   │   │   ├── rplidar_node.cpp            # ⭐ Main RPLidar node implementation
│   │   │   └── standalone_rplidar.cpp      # Standalone test
│   │   │
│   │   ├── launch/                         # Launch Files
│   │   │   ├── rplidar.launch.py           # ⭐ Main RPLidar launch file
│   │   │   ├── rplidar_a3.launch.py        # RPLidar A3 model
│   │   │   ├── rplidar_s1.launch.py        # RPLidar S1 model
│   │   │   ├── rplidar_s1_tcp.launch.py    # RPLidar S1 TCP
│   │   │   ├── view_rplidar.launch.py      # View with RViz
│   │   │   ├── view_rplidar_a3.launch.py   # View A3 with RViz
│   │   │   ├── view_rplidar_s1.launch.py   # View S1 with RViz
│   │   │   ├── view_rplidar_s1_tcp.launch.py
│   │   │   └── test_rplidar_a3.launch.py   # A3 test launch
│   │   │
│   │   ├── scripts/                        # Utility Scripts
│   │   │   ├── create_udev_rules.sh        # Create udev rules for USB
│   │   │   ├── delete_udev_rules.sh        # Delete udev rules
│   │   │   └── rplidar.rules               # udev rules file
│   │   │
│   │   ├── rviz/                           # RViz Config
│   │   │   └── rplidar.rviz                # RViz configuration
│   │   │
│   │   ├── sdk/                            # RPLidar SDK
│   │   │   ├── include/                    # SDK headers
│   │   │   │   ├── rplidar.h               # Main SDK header
│   │   │   │   ├── rplidar_driver.h        # Driver interface
│   │   │   │   ├── rplidar_protocol.h      # Protocol definitions
│   │   │   │   ├── rplidar_cmd.h           # Command definitions
│   │   │   │   └── rptypes.h               # Type definitions
│   │   │   ├── src/                        # SDK source
│   │   │   │   ├── rplidar_driver.cpp      # Driver implementation
│   │   │   │   ├── rplidar_driver_serial.h # Serial driver
│   │   │   │   ├── rplidar_driver_TCP.h    # TCP driver
│   │   │   │   ├── rplidar_driver_impl.h   # Driver implementation
│   │   │   │   ├── sdkcommon.h             # Common SDK code
│   │   │   │   ├── hal/                    # Hardware abstraction layer
│   │   │   │   │   ├── thread.h, thread.cpp
│   │   │   │   │   ├── types.h
│   │   │   │   │   └── ...
│   │   │   │   └── arch/                   # Architecture-specific code
│   │   │   │       ├── linux/              # Linux implementation
│   │   │   │       ├── macOS/              # macOS implementation
│   │   │   │       └── win32/              # Windows implementation
│   │   │   └── README.txt                  # SDK README
│   │   │
│   │   ├── rplidar_A1.png                  # A1 model image
│   │   └── rplidar_A2.png                  # A2 model image
│   │
│   ├── your_robot_bringup/                 # Robot Bringup Package
│   │   ├── package.xml                     # Package metadata
│   │   ├── setup.py                        # Python package setup
│   │   ├── setup.cfg                       # Setup configuration
│   │   ├── resource/
│   │   │   └── your_robot_bringup          # Resource file
│   │   ├── nav2_params.yaml                # Nav2 parameters (if used)
│   │   ├── launch/
│   │   │   └── bringup.launch.py           # Basic bringup launch file
│   │   └── src/
│   │       ├── setup.py
│   │       └── your_robot_bringup/
│   │           ├── __init__.py
│   │           ├── __pycache__/
│   │           ├── wheel_odom.py           # Wheel odometry node
│   │           ├── wheel_odom.py.save      # Backup
│   │           └── rrt_autonomous.py       # RRT autonomous navigation
│   │
│   └── simple_rover_description/           # Simple Rover URDF Description
│       └── urdf/
│           └── simple_rover.urdf.xacro     # Xacro robot description
│
├── build/                                  # Build Directory (Generated)
│   ├── COLCON_IGNORE
│   ├── bts_motor_controller/               # Built motor controller package
│   ├── rover_navigation/                   # Built navigation package
│   ├── rplidar_ros/                        # Built RPLidar package
│   └── your_robot_bringup/                 # Built bringup package
│
├── install/                                # Install Directory (Generated)
│   ├── COLCON_IGNORE
│   ├── local_setup.bash                    # ⭐ Source this to use workspace
│   ├── local_setup.sh
│   ├── local_setup.zsh
│   ├── local_setup.ps1
│   ├── setup.bash                          # ⭐ Main setup script
│   ├── setup.sh
│   ├── setup.zsh
│   ├── setup.ps1
│   ├── bts_motor_controller/               # Installed motor controller
│   ├── rover_navigation/                   # Installed navigation package
│   ├── rplidar_ros/                        # Installed RPLidar package
│   └── your_robot_bringup/                 # Installed bringup package
│
└── log/                                    # Build Logs (Generated)
    ├── COLCON_IGNORE
    ├── latest                              # Latest build log
    ├── latest_build                        # Latest build directory
    └── build_YYYY-MM-DD_HH-MM-SS/         # Timestamped build logs
        ├── events.log
        ├── logger_all.log
        └── [package_name]/                 # Package-specific logs
```

---

## ⭐ Critical Files to Understand

### Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| `ekf.yaml` | Extended Kalman Filter configuration (sensor fusion) | `config/` |
| `slam_params.yaml` | SLAM Toolbox parameters (mapping settings) | `config/` |
| `your_robot.urdf` | Robot description (defines robot structure, LiDAR position) | `urdf/` |

### Launch Files

| File | Purpose | Location |
|------|---------|----------|
| `start_slam_mapping.sh` | ⭐ **Automated SLAM startup script** (opens all terminals) | Root |
| `full_mapping.launch.py` | Complete SLAM mapping launch file | `launch/` |
| `a_star_lwb_navigation.launch.py` | Complete navigation stack (A* + LWB) | `src/rover_navigation/launch/` |
| `rplidar.launch.py` | RPLidar driver launch file | `src/rplidar_ros/launch/` |

### Core Nodes (Python)

| File | Purpose | Package |
|------|---------|---------|
| `motor_node.py` | ⭐ BTS7960 motor controller (GPIO control) | `bts_motor_controller` |
| `odometry_node.py` | ⭐ Odometry tracking (position estimation) | `rover_navigation` |
| `obstacle_detection.py` | ⭐ LiDAR obstacle detection | `rover_navigation` |
| `global_planner_a_star.py` | ⭐ A* global path planner | `rover_navigation` |
| `local_planner_lwb.py` | ⭐ LWB local planner (velocity commands) | `rover_navigation` |
| `planner_manager.py` | ⭐ Navigation planner coordinator | `rover_navigation` |

### Documentation (Existing)

| File | Purpose | Location |
|------|---------|----------|
| `README.md` | Navigation package overview | `src/rover_navigation/` |
| `TECHNICAL_GUIDE.md` | Deep technical dive | `src/rover_navigation/` |
| `NAVIGATION_STACK_GUIDE.md` | Navigation architecture | `src/rover_navigation/` |
| `TESTING_GUIDE.md` | Navigation testing procedures | `src/rover_navigation/` |
| `QUICK_START_MAPPING.md` | Quick SLAM reference | Root |

### Documentation (New - Created for Testing)

| File | Purpose | Location |
|------|---------|----------|
| 🆕 `COMPLETE_TESTING_GUIDE.md` | ⭐ **Complete testing guide** (hardware + software) | Root |
| 🆕 `TERMINAL_WORKFLOW.md` | ⭐ **Terminal-by-terminal explanation** | Root |
| 🆕 `TESTING_SUMMARY.md` | ⭐ **Quick testing reference** | Root |
| 🆕 `CODEBASE_FILE_TREE.md` | ⭐ **This file - codebase overview** | Root |

---

## 📦 Package Overview

### 1. `bts_motor_controller` Package
**Purpose**: Controls BTS7960 motor driver via GPIO

**Key Files:**
- `motor_node.py` - Main motor controller node
  - Subscribes to `/cmd_vel` (Twist messages)
  - Converts to PWM signals via GPIO
  - Controls left/right motors for differential drive

**Usage:**
```bash
ros2 run bts_motor_controller motor_node
```

---

### 2. `rover_navigation` Package
**Purpose**: Complete navigation system with path planning and obstacle avoidance

**Key Nodes:**
- `odometry_node.py` - Tracks robot position
- `obstacle_detection.py` - Detects obstacles from LiDAR
- `global_planner_a_star.py` - Plans global path (A* algorithm)
- `local_planner_lwb.py` - Generates velocity commands (LWB algorithm)
- `planner_manager.py` - Coordinates planning and execution
- `rrt_planner.py` - Alternative RRT path planner
- `path_executor.py` - Executes planned paths

**Key Launch Files:**
- `a_star_lwb_navigation.launch.py` - Complete navigation stack
- `full_navigation.launch.py` - RRT-based navigation
- `obstacle_detection.launch.py` - Obstacle detection only

**Usage:**
```bash
ros2 launch rover_navigation a_star_lwb_navigation.launch.py
```

---

### 3. `rplidar_ros` Package
**Purpose**: RPLidar A1/A2/A3/S1 driver for ROS2

**Key Files:**
- `rplidar_node.cpp` - Main driver node (C++)
- `rplidar.launch.py` - Launch file for RPLidar

**Usage:**
```bash
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

---

### 4. `your_robot_bringup` Package
**Purpose**: Basic robot bringup utilities

**Key Files:**
- `wheel_odom.py` - Wheel odometry node
- `bringup.launch.py` - Basic bringup launch

---

## 🔄 Data Flow Overview

```
Hardware Layer
├── RPLidar (USB) → /scan (LaserScan)
├── Encoders (Serial) → /odom (optional)
└── Motors (GPIO) ← /cmd_vel (Twist)

ROS2 Nodes
├── rplidar_ros → /scan
├── robot_state_publisher → /tf (base_link→laser)
├── odometry_node → /odom
├── ekf_node → /tf (odom→base_link)
├── slam_toolbox → /map, /tf (map→odom)
├── a_star_planner → /global_path
├── lwb_planner → /cmd_vel
└── motor_controller ← /cmd_vel → GPIO
```

---

## 🚀 Quick Start Files

### For Testing SLAM:
1. **`start_slam_mapping.sh`** - Automated script (easiest)
2. **`launch/full_mapping.launch.py`** - Launch file
3. **`QUICK_START_MAPPING.md`** - Quick reference

### For Testing Navigation:
1. **`src/rover_navigation/launch/a_star_lwb_navigation.launch.py`** - Navigation launch
2. **`src/rover_navigation/TESTING_GUIDE.md`** - Navigation testing
3. **🆕 `COMPLETE_TESTING_GUIDE.md`** - Complete testing guide

### For Understanding:
1. **🆕 `TESTING_SUMMARY.md`** - Quick overview
2. **🆕 `TERMINAL_WORKFLOW.md`** - Terminal details
3. **`src/rover_navigation/README.md`** - Package documentation
4. **`src/rover_navigation/TECHNICAL_GUIDE.md`** - Technical details

---

## 📋 File Categories

### Configuration Files
- `config/ekf.yaml` - EKF sensor fusion config
- `config/slam_params.yaml` - SLAM parameters
- `urdf/your_robot.urdf` - Robot structure definition

### Launch Files
- `start_slam_mapping.sh` - Automated startup script
- `launch/full_mapping.launch.py` - SLAM launch
- `src/rover_navigation/launch/*.launch.py` - Navigation launches
- `src/rplidar_ros/launch/rplidar.launch.py` - LiDAR launch

### Core Implementation
- `src/bts_motor_controller/bts_motor_controller/motor_node.py` - Motor control
- `src/rover_navigation/rover_navigation/*.py` - Navigation nodes
- `src/rplidar_ros/src/rplidar_node.cpp` - LiDAR driver (C++)

### Documentation (Root)
- 🆕 `COMPLETE_TESTING_GUIDE.md` - Complete testing procedures
- 🆕 `TERMINAL_WORKFLOW.md` - Terminal explanations
- 🆕 `TESTING_SUMMARY.md` - Quick reference
- `QUICK_START_MAPPING.md` - Quick SLAM guide
- `SOLUTION_SUMMARY.md` - Issue solutions

### Documentation (rover_navigation)
- `README.md` - Package overview
- `TECHNICAL_GUIDE.md` - Technical details
- `TESTING_GUIDE.md` - Testing procedures
- `NAVIGATION_STACK_GUIDE.md` - Architecture guide
- `00_START_HERE.md` - Package quick start

### Generated/Build Files
- `build/` - Build artifacts (generated)
- `install/` - Installed packages (generated)
- `log/` - Build logs (generated)
- `frames_*.pdf`, `frames_*.gv` - TF tree visualizations (generated)

---

## 🎯 What You Need to Know

### Essential Files to Read:
1. **🆕 `TESTING_SUMMARY.md`** - Start here for overview
2. **🆕 `COMPLETE_TESTING_GUIDE.md`** - Follow for testing
3. **`config/ekf.yaml`** - Understand EKF configuration
4. **`config/slam_params.yaml`** - Understand SLAM settings
5. **`urdf/your_robot.urdf`** - Understand robot structure

### Essential Commands:
```bash
# Build workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

# Start SLAM (automated)
./start_slam_mapping.sh

# Start navigation
ros2 launch rover_navigation a_star_lwb_navigation.launch.py

# Start motor controller
ros2 run bts_motor_controller motor_node

# Start LiDAR
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

### Essential Topics:
- `/scan` - Laser scan data (from RPLidar)
- `/odom` - Robot odometry (from odometry_node)
- `/map` - Occupancy grid map (from SLAM)
- `/cmd_vel` - Velocity commands (to motor controller)
- `/global_path` - Planned path (from A* planner)
- `/tf` - Transform tree (robot structure)

---

## 🔍 Navigation Systems

### System 1: RRT-Based Navigation
**Files:**
- `rrt_planner.py` - RRT path planner
- `path_executor.py` - Path execution
- `full_navigation.launch.py` - Launch file

**Usage:**
```bash
ros2 launch rover_navigation full_navigation.launch.py
```

### System 2: A* + LWB Navigation (Current/Recommended)
**Files:**
- `global_planner_a_star.py` - A* global planner
- `local_planner_lwb.py` - LWB local planner
- `planner_manager.py` - Planner coordinator
- `a_star_lwb_navigation.launch.py` - Launch file

**Usage:**
```bash
ros2 launch rover_navigation a_star_lwb_navigation.launch.py
```

---

## 📊 System Architecture Summary

```
┌─────────────────────────────────────────┐
│         HARDWARE LAYER                   │
├─────────────────────────────────────────┤
│ RPLidar │ Encoders │ BTS7960 Motors     │
└────┬────┴────┬─────┴────────┬───────────┘
     │         │              │
┌────▼─────────▼──────────────▼───────────┐
│         ROS2 SOFTWARE LAYER              │
├─────────────────────────────────────────┤
│                                         │
│  Drivers:                               │
│  ├─ rplidar_ros → /scan                │
│  └─ motor_controller ← /cmd_vel        │
│                                         │
│  Localization:                          │
│  ├─ odometry_node → /odom              │
│  ├─ ekf_node → /tf (odom→base_link)    │
│  └─ slam_toolbox → /map, /tf (map→odom)│
│                                         │
│  Navigation:                            │
│  ├─ a_star_planner → /global_path      │
│  ├─ lwb_planner → /cmd_vel             │
│  └─ planner_manager (coordinates)      │
│                                         │
└─────────────────────────────────────────┘
```

---

## ✅ Checklist for Understanding

- [ ] Read `TESTING_SUMMARY.md` for overview
- [ ] Read `COMPLETE_TESTING_GUIDE.md` for procedures
- [ ] Understand `config/ekf.yaml` configuration
- [ ] Understand `config/slam_params.yaml` configuration
- [ ] Review `urdf/your_robot.urdf` robot structure
- [ ] Test `start_slam_mapping.sh` script
- [ ] Understand motor controller (`motor_node.py`)
- [ ] Understand odometry node (`odometry_node.py`)
- [ ] Understand navigation nodes (A* + LWB)
- [ ] Review `src/rover_navigation/README.md`
- [ ] Review `src/rover_navigation/TECHNICAL_GUIDE.md`

---

## 📚 Additional Resources

### External Documentation:
- ROS2 Jazzy documentation
- SLAM Toolbox documentation
- RPLidar documentation
- Robot Localization (EKF) documentation

### Generated Documentation:
- See `frames_*.pdf` for TF tree visualization
- Build logs in `log/` directory
- Package documentation in `install/` directory

---

**Last Updated**: Created with complete codebase analysis

**Legend:**
- 🆕 = New file created for documentation
- ⭐ = Critical/important file
- Generated files not shown in detail (build/, install/, log/)











