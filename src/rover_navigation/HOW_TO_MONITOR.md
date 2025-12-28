# 👀 How to See Encoded Values & Watch Rover Move

## Quick Summary

I've created **3 ways** to see your rover's encoder values and movement:

### **Method 1: Live Real-Time Monitor (BEST)** ⭐
```bash
python3 ~/ros2_ws/src/rover_navigation/monitor_rover.py
```
Shows position (X, Y), heading angle, velocity, and status updates every 0.5 seconds

### **Method 2: See Topics Directly**
```bash
ros2 topic echo /odom           # Current position from encoders
ros2 topic echo /cmd_vel        # Velocity commands being sent
ros2 topic echo /path           # Planned path waypoints
```

### **Method 3: Visual Display (RViz)**
```bash
ros2 run rviz2 rviz2
# Add displays for /odom, /path, /scan, /occupancy_grid
```

---

## 🚀 Run This Now (3 Terminals)

**Terminal 1:**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch rover_navigation full_navigation.launch.py
```

**Terminal 2 - SEE ENCODER VALUES:**
```bash
cd ~/ros2_ws && source install/setup.bash
python3 src/rover_navigation/monitor_rover.py
```

**Terminal 3 - MAKE IT MOVE:**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
}'
```

---

## 📊 What You'll See in Terminal 2

Live updating display showing:

```
╔══════════════════════════════════════════════════════╗
║       🚗 ROVER MOVEMENT MONITOR - 14:32:45         ║
╠══════════════════════════════════════════════════════╣
║  📍 POSITION & ORIENTATION:
║     X Position:    2.345 m          ← Encoder X
║     Y Position:    1.234 m          ← Encoder Y
║     Z Position:    0.000 m
║     Heading:       11.2° (0.196 rad) ← Current angle
║     Distance from origin: 2.604 m
╠══════════════════════════════════════════════════════╣
║  ⚙️  VELOCITY COMMANDS:
║     Linear X:   0.500 m/s           ← Forward speed
║     Linear Y:   0.000 m/s
║     Angular Z:  0.315 rad/s (18.1°/s) ← Rotation
║     Total speed: 0.500 m/s | Rotation: 0.315 rad/s
╠══════════════════════════════════════════════════════╣
║  📊 PLANNING STATUS:
║     Status:     ✅ SUCCESS - Path planned
║     Waypoints:  12 waypoints in current path
╚══════════════════════════════════════════════════════╝
```

---

## 🎯 What Each Value Means

### **Position (X, Y)**
- **X**: How far forward/backward (meters)
- **Y**: How far left/right (meters)
- These come from your encoders/odometry
- Update as rover moves

### **Heading (Yaw)**
- **In degrees** (11.2°): Easy to read
- **In radians** (0.196): Used in calculations
- 0° = facing East, 90° = facing North

### **Linear X Velocity**
- **0.5 m/s** = moving at half meter per second
- How fast your rover is going forward
- This is the command sent to motors

### **Angular Z Velocity**
- **0.315 rad/s** = rotating at this speed
- Convert to degrees: × 57.3 = 18.1°/second
- Rotation rate of rover

---

## 📺 Alternative: Simple Echo Commands

If you just want to see raw numbers:

**See just position:**
```bash
ros2 topic echo /odom --field pose.pose.position
```

**See just velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Watch updates continuously:**
```bash
ros2 topic echo /odom | head -50
```

---

## 📈 See How It Changes Over Time

**Record movement to file:**
```bash
python3 << 'EOF'
import rclpy
from geometry_msgs.msg import PoseStamped
import csv

data = []

def callback(msg):
    pos = msg.pose.position
    data.append({
        'X': pos.x,
        'Y': pos.y,
        'Time': msg.header.stamp.sec
    })
    if len(data) % 10 == 0:
        print(f"Recorded {len(data)} points...")

rclpy.init()
node = rclpy.node.Node('recorder')
sub = node.create_subscription(PoseStamped, '/odom', callback, 100)

import time
time.sleep(30)  # Record for 30 seconds

with open('/tmp/rover_position.csv', 'w') as f:
    writer = csv.DictWriter(f, fieldnames=['X', 'Y', 'Time'])
    writer.writeheader()
    writer.writerows(data)

print(f"Saved to /tmp/rover_position.csv")
EOF
```

**Open in Excel or view:**
```bash
cat /tmp/rover_position.csv
```

---

## 🎮 Send Different Goals to Watch Different Movements

**Move 5m forward:**
```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}'
```

**Turn left 45 degrees:**
```bash
# Send to left side
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 0.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
}'
```

**Move in circle:**
```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
}'
```

---

## 🔍 Check Topic Frequency

See how often data updates:
```bash
ros2 topic hz /odom        # Should show ~10 Hz
ros2 topic hz /cmd_vel     # Should show ~10 Hz
ros2 topic hz /path        # Should show ~5 Hz
```

---

## 📚 Files Created for You

1. **`monitor_rover.py`** - Real-time monitoring script (the main one to use!)
2. **`MONITORING_GUIDE.md`** - Detailed guide with all commands
3. **`QUICK_MONITOR.sh`** - Quick reference script

---

## 🎬 Example Scenario

**Start 3 terminals with:**

```
T1: ros2 launch rover_navigation full_navigation.launch.py
T2: python3 src/rover_navigation/monitor_rover.py
T3: ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{...}'
```

**You'll see in T2:**
- X, Y position values change as rover moves
- Heading angle updates as it turns
- Velocity commands show what's being sent
- Status updates when replanning occurs

**In real rover hardware**, these are your encoder values!

---

## ⚡ Quick Commands Cheat Sheet

```bash
# Monitor movement
python3 ~/ros2_ws/src/rover_navigation/monitor_rover.py

# See raw odometry
ros2 topic echo /odom

# See motor commands
ros2 topic echo /cmd_vel

# See path
ros2 topic echo /path

# See status
ros2 topic echo /plan_status

# Frequency check
ros2 topic hz /odom

# Visual (RViz)
ros2 run rviz2 rviz2
```

---

**All ready to use! Just run the commands above and watch your rover move! 🚀**
