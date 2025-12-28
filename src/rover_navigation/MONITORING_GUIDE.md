# 🔍 Monitoring Rover Movement & Encoder Values

## 1️⃣ View Encoded/Odometry Values

### **Option A: Simple Topic Echo**

See current position and orientation:
```bash
ros2 topic echo /odom
```

**Output Example:**
```
header:
  stamp:
    sec: 1766589789
    nsec: 427878397
  frame_id: odom
pose:
  pose:
    position:
      x: 2.345      # Current X position (meters)
      y: 1.234      # Current Y position (meters)
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.195     # Rotation (Z-axis, radians ≈ 11 degrees)
      w: 0.981
  covariance: ...
```

### **Option B: Watch Odometry Change in Real-Time**

See values update continuously with timestamps:
```bash
ros2 topic echo /odom --once
# Then run again to see change:
ros2 topic echo /odom --once
```

### **Option C: Monitor Odometry Frequency**

Check how often odometry is publishing:
```bash
ros2 topic hz /odom
```

**Output Example:**
```
average rate: 10.00 Hz
  min: 100.00ms   max: 100.00ms   std dev: 0.01ms   window: 100
```

---

## 2️⃣ View Velocity Commands (What's Being Sent to Motors)

### **Monitor Motor Commands**

See what velocity commands the path executor is sending:
```bash
ros2 topic echo /cmd_vel
```

**Output Example:**
```
linear:
  x: 0.5      # Forward velocity (m/s)
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.315    # Rotation speed (rad/s) ≈ 18 degrees/sec
```

### **Real-Time Velocity Monitoring**

Watch velocity commands update as rover follows path:
```bash
watch -n 0.1 "ros2 topic echo /cmd_vel --once"
```

Or simpler:
```bash
ros2 topic echo /cmd_vel | head -50  # Show first 50 updates
```

---

## 3️⃣ Advanced: Custom Monitoring Script

Create a script to display everything nicely:

```bash
cat > ~/ros2_ws/monitor_rover.py << 'EOF'
#!/usr/bin/env python3
"""
Real-time rover movement monitoring
Shows: Position, Orientation, Velocity Commands, Path Status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
import sys
from datetime import datetime


class RoverMonitor(Node):
    def __init__(self):
        super().__init__('rover_monitor')
        
        # Subscribe to topics
        self.odom_sub = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/plan_status', self.status_callback, 10
        )
        
        self.odom_data = None
        self.cmd_vel_data = None
        self.status_data = None
        
        # Display timer
        self.display_timer = self.create_timer(0.5, self.display_callback)

    def odom_callback(self, msg):
        self.odom_data = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg

    def status_callback(self, msg):
        self.status_data = msg.data

    def display_callback(self):
        # Clear screen (works on Linux/Mac)
        sys.stdout.write('\033[2J\033[H')
        
        print("=" * 60)
        print(f"  🚗 ROVER MOVEMENT MONITOR - {datetime.now().strftime('%H:%M:%S')}")
        print("=" * 60)
        
        # Position and Orientation
        if self.odom_data:
            pos = self.odom_data.pose.position
            ori = self.odom_data.pose.orientation
            
            # Convert quaternion to yaw angle
            yaw = self.get_yaw_from_quat(ori)
            yaw_deg = math.degrees(yaw)
            
            print(f"\n📍 POSITION & ORIENTATION:")
            print(f"   X: {pos.x:7.3f} m")
            print(f"   Y: {pos.y:7.3f} m")
            print(f"   Heading (Yaw): {yaw_deg:6.1f}° ({yaw:6.3f} rad)")
            print(f"   Distance from origin: {math.sqrt(pos.x**2 + pos.y**2):6.3f} m")
        
        # Velocity Commands
        if self.cmd_vel_data:
            print(f"\n⚙️  VELOCITY COMMANDS:")
            print(f"   Linear X:  {self.cmd_vel_data.linear.x:6.3f} m/s")
            print(f"   Angular Z: {self.cmd_vel_data.angular.z:6.3f} rad/s ({math.degrees(self.cmd_vel_data.angular.z):6.1f}°/s)")
        
        # Status
        if self.status_data:
            status_symbol = {
                'SUCCESS': '✅',
                'FAILED': '❌',
                'REPLANNING': '🔄'
            }.get(self.status_data, '❓')
            print(f"\n📊 PLAN STATUS: {status_symbol} {self.status_data}")
        
        print("\n" + "=" * 60)
        sys.stdout.flush()

    @staticmethod
    def get_yaw_from_quat(quat):
        """Convert quaternion to yaw angle"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw


def main(args=None):
    rclpy.init(args=args)
    monitor = RoverMonitor()
    print("Starting Rover Monitor... (Press Ctrl+C to stop)")
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

chmod +x ~/ros2_ws/monitor_rover.py
```

**Run the monitor:**
```bash
cd ~/ros2_ws
python3 monitor_rover.py
```

**Output Example:**
```
============================================================
  🚗 ROVER MOVEMENT MONITOR - 14:32:45
============================================================

📍 POSITION & ORIENTATION:
   X:   2.345 m
   Y:   1.234 m
   Heading (Yaw):  11.2° (0.196 rad)
   Distance from origin: 2.604 m

⚙️  VELOCITY COMMANDS:
   Linear X:  0.500 m/s
   Angular Z: 0.315 rad/s ( 18.1°/s)

📊 PLAN STATUS: ✅ SUCCESS

============================================================
```

---

## 4️⃣ Visualize in RViz (Best for Seeing Movement)

### **Quick Setup**

**Terminal 1 - Start navigation:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rover_navigation full_navigation.launch.py
```

**Terminal 2 - Start RViz:**
```bash
source install/setup.bash
ros2 run rviz2 rviz2
```

### **Configure RViz**

1. **Set Fixed Frame** (top-left dropdown): `odom`

2. **Add Displays** (bottom-left "Add"):

   **a) Robot Position:**
   - Add → PoseStamped
   - Topic: `/odom`
   - Color: Blue
   - Shape: Arrow

   **b) Target Goal:**
   - Add → PoseStamped  
   - Topic: `/goal`
   - Color: Green
   - Shape: Sphere

   **c) Planned Path:**
   - Add → Path
   - Topic: `/path`
   - Color: Red
   - Line width: 0.05

   **d) Obstacles:**
   - Add → OccupancyGrid
   - Topic: `/occupancy_grid`
   - Color Scheme: Map

   **e) LiDAR Scan:**
   - Add → LaserScan
   - Topic: `/scan`
   - Color: White
   - Size: 0.05

3. **Enable Grid** (checkbox):
   - Add → Grid
   - Reference Frame: `odom`

### **Watch Movement**

After RViz is setup:

**Terminal 3 - Send goal:**
```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 5.0, y: 5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

**In RViz, you'll see:**
- 🔵 Blue arrow = current robot position
- 🔴 Red line = planned path
- 🟢 Green sphere = goal location
- ⚪ White dots = LiDAR scan
- ⬜ Gray grid = obstacles

Watch the blue arrow move along the red path toward the green goal!

---

## 5️⃣ Detailed Encoder Data Breakdown

### **Understanding the Odometry Message**

```
/odom (geometry_msgs/PoseStamped)
├── position (X, Y, Z)           ← From encoder/IMU integration
├── orientation (Quaternion)     ← Rotation from gyro/compass
├── linear velocity              ← Velocity estimate
└── angular velocity             ← Rotation rate

PoseStamped includes:
└── timestamp                    ← When measurement was taken
```

### **Extract Specific Values Only**

**Just X position:**
```bash
ros2 topic echo /odom --field pose.pose.position.x
```

**Just heading (Yaw):**
```bash
ros2 topic echo /odom --field pose.pose.orientation
```

**Real-time table of position changes:**
```bash
watch -n 0.2 'ros2 topic echo /odom --once | grep -A 3 position'
```

---

## 6️⃣ Compare Planned vs Actual Path

**See if rover follows planned path:**

**Terminal 1:**
```bash
ros2 launch rover_navigation full_navigation.launch.py
```

**Terminal 2:**
```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 10.0, y: 0.0, z: 0.0}}
}'
```

**Terminal 3 - Monitor position vs target:**
```bash
python3 << 'PYEOF'
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class PathChecker(rclpy.node.Node):
    def __init__(self):
        super().__init__('path_checker')
        self.create_subscription(PoseStamped, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.path = None
        self.create_timer(1.0, self.timer_cb)
    
    def odom_cb(self, msg):
        self.odom = msg
    
    def path_cb(self, msg):
        self.path = msg
    
    def timer_cb(self):
        if hasattr(self, 'odom') and self.path:
            pos = self.odom.pose.position
            print(f"\n🚗 Current: X={pos.x:.2f}, Y={pos.y:.2f}")
            if self.path.poses:
                goal = self.path.poses[-1].pose.position
                dist = math.sqrt((goal.x - pos.x)**2 + (goal.y - pos.y)**2)
                print(f"🎯 Goal:    X={goal.x:.2f}, Y={goal.y:.2f}")
                print(f"📏 Distance remaining: {dist:.2f}m")

rclpy.init()
node = PathChecker()
rclpy.spin(node)
PYEOF
```

---

## 7️⃣ Record & Playback (For Analysis)

### **Record all movement**

```bash
ros2 bag record -o rover_run /odom /cmd_vel /path /scan /plan_status
```

Press Ctrl+C when done.

### **Playback and analyze**

```bash
# Play it back (runs at recorded speed)
ros2 bag play rover_run

# In another terminal - monitor as it replays
ros2 topic echo /odom
```

### **Inspect recorded data**

```bash
ros2 bag info rover_run
```

---

## 8️⃣ Export Data to CSV (For Analysis in Excel/Python)

```bash
cat > ~/ros2_ws/export_odom.py << 'EOF'
#!/usr/bin/env python3
"""Export odometry data to CSV"""

import rclpy
from geometry_msgs.msg import PoseStamped
import csv
import math

data = []

def odom_callback(msg):
    pos = msg.pose.position
    ori = msg.pose.orientation
    
    # Convert quaternion to yaw
    yaw = math.atan2(2.0*(ori.w*ori.z + ori.x*ori.y), 
                     1.0-2.0*(ori.y**2 + ori.z**2))
    
    data.append({
        'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
        'x': pos.x,
        'y': pos.y,
        'z': pos.z,
        'yaw_rad': yaw,
        'yaw_deg': math.degrees(yaw),
        'distance': math.sqrt(pos.x**2 + pos.y**2)
    })
    
    if len(data) % 10 == 0:
        print(f"Recorded {len(data)} points...")

rclpy.init()
node = rclpy.node.Node('export_node')
sub = node.create_subscription(PoseStamped, '/odom', odom_callback, 100)

print("Recording for 30 seconds... Move the rover!")
import time
time.sleep(30)

# Save to CSV
with open('/tmp/rover_odom.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=data[0].keys())
    writer.writeheader()
    writer.writerows(data)

print(f"Saved {len(data)} records to /tmp/rover_odom.csv")
print("Open in Excel or Python for analysis!")
EOF

python3 ~/ros2_ws/export_odom.py
```

**Then analyze in Python:**
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/rover_odom.csv')

# Plot trajectory
plt.figure(figsize=(10, 8))
plt.plot(df['x'], df['y'], 'b-o', label='Actual Path')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Rover Trajectory')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

# Plot over time
plt.figure(figsize=(10, 5))
plt.plot(df['timestamp'], df['x'], label='X')
plt.plot(df['timestamp'], df['y'], label='Y')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid(True)
plt.show()
```

---

## 9️⃣ Quick Commands Reference

```bash
# See current position
ros2 topic echo /odom --once

# See velocity being sent
ros2 topic echo /cmd_vel --once

# See path waypoints
ros2 topic echo /path --once | head -20

# Monitor all 3 at once (separate terminals)
Terminal 1: ros2 topic echo /odom
Terminal 2: ros2 topic echo /cmd_vel
Terminal 3: ros2 topic echo /path

# See frequency of updates
ros2 topic hz /odom      # Should be ~10 Hz
ros2 topic hz /cmd_vel   # Should be ~10 Hz
ros2 topic hz /path      # Should be ~5 Hz

# Custom monitor script
python3 monitor_rover.py

# RViz visualization
ros2 run rviz2 rviz2
# Add displays: /odom (Arrow), /path (Path), /occupancy_grid (OccupancyGrid)
```

---

## 🎯 Complete Workflow Example

**See the rover move from start to finish:**

```bash
# Terminal 1 - Start system
cd ~/ros2_ws
source install/setup.bash
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2 - Start visualization
source install/setup.bash
ros2 run rviz2 rviz2
# → Add: /odom (PoseStamped), /path (Path), /scan (LaserScan)
# → Set Fixed Frame: odom

# Terminal 3 - Start real-time monitor
python3 monitor_rover.py
# Watch position and velocity update in real-time

# Terminal 4 - Send goal
source install/setup.bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 5.0, y: 5.0, z: 0.0}}
}'

# Now watch:
# - Terminal 2 (RViz): Blue arrow moves toward green goal along red path
# - Terminal 3 (Monitor): Position and velocity values update live
```

---

**That's it! You can now see exactly how the rover moves, where it's going, and what commands are being sent!** 🚀
