# Testing Summary: ROS2 Rover Navigation System

## 🎯 What This System Does

This is a **complete autonomous navigation system** for a differential drive robot that:

1. **Builds maps** of the environment using LiDAR (SLAM)
2. **Localizes** the robot within the map
3. **Plans paths** to goals while avoiding obstacles
4. **Controls motors** to execute planned paths
5. **Provides visualization** tools for monitoring

---

## 🔧 Hardware Setup Required

### Required Components:

1. **Raspberry Pi** - Main computer running ROS2 Jazzy
2. **RPLidar A1/A2** - USB LiDAR scanner (`/dev/ttyUSB0`)
3. **BTS7960 Motor Driver** - GPIO-controlled motor driver
   - GPIO12: Left PWM
   - GPIO5: Left Direction
   - GPIO13: Right PWM  
   - GPIO6: Right Direction
4. **Differential Drive Robot** - Two-wheeled chassis
5. **Wheel Encoders** (Optional) - For better odometry (`/dev/ttyACM0`)

### Software Setup:

```bash
# Install ROS2 packages
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-robot-localization

# Start GPIO daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Build workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 Quick Start Testing

### Option 1: Automated (Easiest)

```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```

This script automatically opens all required terminals in the correct order!

### Option 2: Manual Testing

Follow the step-by-step guide in `COMPLETE_TESTING_GUIDE.md`

---

## 📺 Terminal Overview

The system requires **6-9 terminals** depending on what you're testing:

| Terminal | Component | What It Does | Required? |
|----------|-----------|--------------|-----------|
| 1 | robot_state_publisher | Publishes robot structure (base_link→laser) | ✅ Yes |
| 2 | RPLidar Driver | Reads laser scanner, publishes `/scan` | ✅ Yes |
| 3 | Odometry Node | Tracks robot position, publishes `/odom` | ✅ Yes |
| 4 | EKF Filter | Improves localization, publishes transforms | ✅ Yes |
| 5 | SLAM Toolbox | Builds map, publishes `/map` | ✅ Yes |
| 6 | RViz2 | Visualization GUI | ⚠️ Recommended |
| 7 | Motor Controller | Controls motors, subscribes to `/cmd_vel` | ⚠️ Only if moving |
| 8 | Navigation Stack | Plans paths, publishes `/cmd_vel` | ❌ Optional |
| 9 | Goal Command | Sends navigation goals | ❌ Testing only |

---

## 🔄 Data Flow

```
┌─────────────┐
│   RPLidar   │ → Terminal 2 → /scan (5-10 Hz)
└─────────────┘
       │
       │
┌─────────────┐
│ Encoders    │ → Terminal 3 → /odom (20 Hz)
└─────────────┘
       │
       │
┌─────────────────────┐
│ robot_state_pub     │ → /tf (base_link→laser)
│ (Terminal 1)        │
└─────────────────────┘
       │
       │
┌─────────────────────┐
│ Odometry Node       │ → /odom
│ (Terminal 3)        │
└─────────────────────┘
       │
       │
┌─────────────────────┐
│ EKF                 │ → /tf (odom→base_link)
│ (Terminal 4)        │
└─────────────────────┘
       │
       │
┌─────────────────────┐
│ SLAM Toolbox        │ → /map (0.1-1 Hz)
│ (Terminal 5)        │ → /tf (map→odom)
└─────────────────────┘
       │
       │
┌─────────────────────┐      ┌──────────────────┐
│ Navigation Stack    │ →    │ /global_path     │
│ (Terminal 8)        │      │ /cmd_vel (15 Hz) │
└─────────────────────┘      └────────┬─────────┘
                                      │
                                      │
                              ┌───────▼───────┐
                              │ Motor         │
                              │ Controller    │ → GPIO → Motors
                              │ (Terminal 7)  │
                              └───────────────┘
```

---

## 📊 Expected Outputs

### Terminal 1: robot_state_publisher
- **Output**: Transform messages on `/tf`
- **Frequency**: Continuous
- **What to see**: "Publishing transforms..."

### Terminal 2: RPLidar
- **Output**: `/scan` topic with laser measurements
- **Frequency**: 5-10 Hz
- **What to see**: "RPLidar connected", scan data publishing

### Terminal 3: Odometry
- **Output**: `/odom` topic with robot pose
- **Frequency**: 20 Hz
- **What to see**: "Odometry Node Started", pose values

### Terminal 4: EKF
- **Output**: Transform messages on `/tf`
- **Frequency**: 30 Hz
- **What to see**: "EKF starting...", "Publishing transforms"

### Terminal 5: SLAM Toolbox
- **Output**: `/map` topic (map data)
- **Frequency**: 0.1-1 Hz
- **What to see**: "SLAM initialized" (after 10-30 seconds)
- **Important**: Robot must move for SLAM to initialize!

### Terminal 6: RViz2
- **Output**: GUI window
- **What to see**: Map, laser scans, robot pose visualization

### Terminal 7: Motor Controller
- **Output**: GPIO signals to motors
- **What to see**: "Motor Controller Started"
- **Warning**: Robot will move when receiving commands!

### Terminal 8: Navigation Stack
- **Output**: `/global_path`, `/cmd_vel`
- **Frequency**: 1 Hz (path), 15 Hz (velocity)
- **What to see**: "A* Planner started", "LWB Planner started"

---

## ✅ Testing Checklist

### Phase 1: Hardware Verification
- [ ] LiDAR connected (`/dev/ttyUSB0` exists)
- [ ] pigpio daemon running
- [ ] Serial port exists (if using encoders)
- [ ] Motors connected to GPIO pins

### Phase 2: Individual Components
- [ ] `/scan` topic publishing (~5-10 Hz)
- [ ] `/odom` topic publishing (~20 Hz)
- [ ] TF transforms exist (base_link→laser, odom→base_link)
- [ ] Motor controller responds to `/cmd_vel`

### Phase 3: SLAM System
- [ ] `/map` topic publishing (~0.1-1 Hz)
- [ ] Complete TF chain: map→odom→base_link→laser
- [ ] Map visible in RViz2
- [ ] Robot pose visible on map
- [ ] SLAM initialized (no queue errors)

### Phase 4: Navigation (Optional)
- [ ] Navigation nodes start successfully
- [ ] `/global_path` generates for goals
- [ ] Robot moves toward goal
- [ ] Obstacle avoidance works

---

## 🆘 Common Issues

### Problem: `/scan` not publishing
**Solution**: Check LiDAR connection, permissions (`sudo chmod 666 /dev/ttyUSB0`)

### Problem: SLAM not initializing
**Solution**: 
- Wait 10-30 seconds
- Robot must move slightly
- Check all TF transforms exist
- Verify no "queue full" errors

### Problem: No `/map` topic
**Solution**: 
- Wait for SLAM initialization (10-30 seconds)
- Robot must move
- Check SLAM logs for errors

### Problem: Motor controller errors
**Solution**: 
- Check pigpio daemon: `sudo systemctl status pigpiod`
- Verify GPIO permissions
- Check motor connections

### Problem: TF transform errors
**Solution**: 
- Start robot_state_publisher first
- Check URDF file exists
- Verify transform chain: `ros2 run tf2_tools view_frames`

---

## 🛑 Emergency Stop

**If robot is moving unexpectedly:**
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'
```

This sends zero velocity to stop immediately.

---

## 📚 Documentation Files

- **`COMPLETE_TESTING_GUIDE.md`** - Detailed step-by-step testing procedures
- **`TERMINAL_WORKFLOW.md`** - Detailed explanation of each terminal
- **`QUICK_START_MAPPING.md`** - Quick reference for SLAM
- **`src/rover_navigation/TESTING_GUIDE.md`** - Navigation-specific testing

---

## 🎯 Testing Phases Summary

### Phase 1: Hardware Check (5 minutes)
Verify all hardware connections and basic functionality

### Phase 2: Component Testing (10 minutes)
Test each ROS2 node individually

### Phase 3: SLAM Mapping (15-30 minutes)
Start full SLAM stack, build initial map

### Phase 4: Navigation Testing (30+ minutes)
Test autonomous navigation with path planning

---

## ⚠️ Safety Reminders

1. **Always supervise** first tests
2. **Use emergency stop** command ready
3. **Test in safe area** with no fragile objects
4. **Start with small movements** (0.1 m/s max)
5. **Check battery** before long tests
6. **Don't look into LiDAR** beam directly

---

## 🚀 Next Steps After Testing

1. **Tune Parameters**: Adjust velocities, inflation radius based on results
2. **Build Complete Map**: Drive robot around to build full environment map
3. **Test Navigation**: Try multiple goals and paths
4. **Long Duration Tests**: Test system reliability over time
5. **Optimize Performance**: Adjust SLAM and navigation parameters

---

**For detailed instructions, see `COMPLETE_TESTING_GUIDE.md`**

**For terminal-specific details, see `TERMINAL_WORKFLOW.md`**

---

**Good luck with your testing! 🚀**




