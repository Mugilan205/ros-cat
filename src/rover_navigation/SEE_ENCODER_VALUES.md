# 🎯 COMPLETE GUIDE: See Encoder Values & Watch Rover Move

## TL;DR - Just Run These 3 Commands

```bash
# Terminal 1
cd ~/ros2_ws && source install/setup.bash
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2
cd ~/ros2_ws && source install/setup.bash
python3 src/rover_navigation/monitor_rover.py

# Terminal 3
cd ~/ros2_ws && source install/setup.bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
}'
```

**Terminal 2 shows your encoder values in real-time! 🚗**

---

## What You'll See

### Real-Time Monitor Output (Terminal 2)

```
╔══════════════════════════════════════════════════════════════╗
║       🚗 ROVER MOVEMENT MONITOR - 14:32:45                 ║
╠══════════════════════════════════════════════════════════════╣
║  📍 POSITION & ORIENTATION:
║     X Position:    2.345 m          ← ENCODER X
║     Y Position:    1.234 m          ← ENCODER Y
║     Z Position:    0.000 m
║     Heading:       11.2° (0.196 rad) ← ROTATION ANGLE
║     Distance from origin: 2.604 m
╠══════════════════════════════════════════════════════════════╣
║  ⚙️  VELOCITY COMMANDS:
║     Linear X:   0.500 m/s           ← FORWARD SPEED
║     Linear Y:   0.000 m/s
║     Angular Z:  0.315 rad/s (18.1°/s) ← TURN SPEED
║     Total speed: 0.500 m/s | Rotation: 0.315 rad/s
╠══════════════════════════════════════════════════════════════╣
║  📊 PLANNING STATUS:
║     Status:     ✅ SUCCESS - Path planned
║     Waypoints:  12 waypoints in current path
╚══════════════════════════════════════════════════════════════╝
```

**Updates every 0.5 seconds!** Watch the X and Y values change as your rover moves.

---

## Understanding the Values

### Position (X, Y) - ENCODER VALUES
- **X: 2.345 m** - Distance moved forward
- **Y: 1.234 m** - Distance moved sideways (for omnidirectional robots)
- These come directly from your wheel encoders
- Start at 0, increase as rover moves

### Heading - ROTATION ANGLE
- **11.2° in degrees** - Easy to read
- **0.196 radians** - Used in calculations
- 0° = facing East, 90° = facing North, 180° = facing West, 270° = facing South

### Velocity Commands - WHAT'S SENT TO MOTORS
- **Linear X: 0.5 m/s** - How fast moving forward (half meter per second)
- **Angular Z: 0.315 rad/s** - How fast rotating
  - Convert to degrees: 0.315 × 57.3 = 18.1°/second
  - Positive = turning left, Negative = turning right

---

## 5 Different Ways to See the Data

### 1. **Live Monitor Script (BEST)** ⭐
```bash
python3 src/rover_navigation/monitor_rover.py
```
- Real-time display
- Updates every 0.5 seconds
- Shows position, velocity, status
- **Easiest to understand!**

### 2. **Direct Topic Echo (Raw Data)**
```bash
ros2 topic echo /odom
```
Output:
```yaml
position:
  x: 2.345
  y: 1.234
orientation:
  z: 0.1959265
  w: 0.9805646
```

### 3. **Just Position**
```bash
ros2 topic echo /odom --field pose.pose.position
```

### 4. **Just Velocity Commands**
```bash
ros2 topic echo /cmd_vel
```

### 5. **Visual Display (RViz)**
```bash
ros2 run rviz2 rviz2
# Add displays: /odom, /path, /scan, /occupancy_grid
```
Watch blue arrow move along red path toward green goal!

---

## Example: Rover Moving From (0,0) to (10,0)

### Time progression:
```
T=0s: Goal sent (10, 0)
  Monitor shows:
    X Position: 0.000 m
    Y Position: 0.000 m
    Heading: 0.0°
    Linear X: 0.5 m/s ← Starting to move
    Angular Z: 0.0 rad/s

T=1s: Moved forward
  Monitor shows:
    X Position: 0.5 m ← Increased!
    Y Position: 0.0 m
    Heading: 0.0°
    Linear X: 0.5 m/s
    Angular Z: 0.0 rad/s

T=5s: Halfway there
  Monitor shows:
    X Position: 2.5 m ← Still increasing
    Y Position: 0.0 m
    Heading: 0.0°
    Linear X: 0.5 m/s
    Angular Z: 0.0 rad/s

T=10s: Near goal, slowing down
  Monitor shows:
    X Position: 4.8 m
    Y Position: 0.0 m
    Heading: 0.0°
    Linear X: 0.3 m/s ← Velocity decreasing
    Angular Z: 0.0 rad/s

T=20s: Reached goal
  Monitor shows:
    X Position: 10.0 m ← Target reached!
    Y Position: 0.0 m
    Heading: 0.0°
    Linear X: 0.0 m/s ← Stopped!
    Angular Z: 0.0 rad/s
```

---

## Common Questions

### Q: Where do the X, Y values come from?
**A:** From your wheel encoders! The odometry system reads encoder ticks and calculates position.

### Q: How often does it update?
**A:** Every ~100ms (10 Hz). The monitor script displays it every 500ms so you can read it.

### Q: Can I record the values?
**A:** Yes! See MONITORING_GUIDE.md for data export examples.

### Q: Why does the heading use degrees and radians?
**A:** Degrees (11.2°) are easy for humans. Radians (0.196) are used in math/programs.

### Q: What's the difference between /odom and /cmd_vel?
**A:** 
- `/odom` = where the rover IS (from encoders)
- `/cmd_vel` = where to MAKE it go (commands to motors)

### Q: How do I change the rover's speed?
**A:** Edit `/launch/full_navigation.launch.py` and change:
```python
{'linear_speed': 0.5}   # Change to 1.0 for faster
{'angular_speed': 1.0}  # Change to 2.0 for sharper turns
```

---

## System Flow

```
ROVER HARDWARE
    │
    ├─ Wheel Encoders (count ticks)
    │       ↓
    └─ Encoder Driver (publishes /odom)
           │
           ├─ X position from left/right wheel difference
           ├─ Y position (usually 0 for differential drive)
           └─ Heading from integrated rotation

    /odom TOPIC (where rover IS)
       ↓ (used by planner)
    ROS 2 Navigation Stack
       ├─ Obstacle Detection (from /scan)
       ├─ RRT Planner (creates path)
       └─ Path Executor (generates commands)
    
    /cmd_vel TOPIC (what to do)
       ↓
    Motor Controller (BTS driver)
       ├─ Converts velocity to PWM signals
       └─ Sends to motors
    
    Motors spin
       ↓
    Encoders see rotation
       ↓
    Cycle repeats (10 Hz)
```

---

## Real Rover Setup

On your actual rover, make sure you have:

✅ **LiDAR Publishing `/scan`**
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

✅ **Encoders Publishing `/odom`**
- Your odometry node publishes to `/odom`
- This feeds encoder data into the navigation system

✅ **Motor Controller Subscribing to `/cmd_vel`**
- Your BTS motor controller receives `/cmd_vel` commands
- Converts them to PWM for motors

The navigation system bridges all three! 🚀

---

## Files Created for You

| File | Purpose |
|------|---------|
| `monitor_rover.py` | **Main tool** - Real-time monitor script |
| `MONITORING_GUIDE.md` | Detailed guide with all commands |
| `HOW_TO_MONITOR.md` | This overview document |
| `QUICK_MONITOR.sh` | Quick reference script |

---

## Next Steps

1. **Run the 3 commands above** to see it in action
2. **Watch Terminal 2** as encoder values change
3. **Send different goals** to see different movements
4. **Try RViz** for visual feedback
5. **Read MONITORING_GUIDE.md** for advanced options

---

## Troubleshooting

**Monitor shows "Waiting for odometry data"**
- Make sure your odometry node is running
- Check: `ros2 topic hz /odom`

**Monitor shows "Waiting for velocity commands"**
- Normal at startup - will show once planning starts
- Send a goal: `ros2 topic pub -1 /goal ...`

**Values not changing**
- Check if rover is actually moving
- Look at velocity commands - are they zero?
- Check motor controller connection

**Heading stays at 0°**
- Normal if rover not rotating
- Turn the rover and watch it change

---

**Ready to see your rover move? Run the commands at the top now! 🚀**
