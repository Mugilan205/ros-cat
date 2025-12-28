# ROS 2 Rover Navigation: Obstacle Detection & RRT Path Planning

Complete implementation of obstacle detection and dynamic RRT-based path planning for your ROS 2 Jazzy rover with LiDAR.

## 📋 Overview

This package provides three coordinated nodes that work together to enable autonomous navigation with obstacle avoidance:

```
LiDAR (/scan)
    ↓
[Obstacle Detection Node]
    ↓ (/obstacles)
[RRT Planner Node]
    ↓ (/path)
[Path Executor Node]
    ↓ (/cmd_vel)
Motor Controller
```

## 🚀 Features

- **Obstacle Detection**: Real-time LiDAR processing with distance-based detection
- **RRT Path Planning**: Rapidly-exploring Random Tree with 10% goal biasing
- **Dynamic Re-planning**: Monitors obstacles and re-plans when collisions detected
- **Pure Pursuit Control**: Smooth path tracking for differential drive
- **Configurable Parameters**: Tunable safety margins, speeds, and planning parameters

## 📦 Package Structure

```
rover_navigation/
├── rover_navigation/
│   ├── obstacle_detection.py    # LiDAR processing node
│   ├── rrt_planner.py          # RRT planning node
│   └── path_executor.py        # Velocity control node
├── launch/
│   ├── obstacle_detection.launch.py
│   ├── rrt_planner.launch.py
│   └── full_navigation.launch.py
├── package.xml
└── setup.py
```

## 🔧 Installation

1. **Create the package** (already done in `src/rover_navigation`)

2. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select rover_navigation
source install/setup.bash
```

3. **Verify dependencies** are installed:
```bash
sudo apt-get install python3-numpy python3-scipy
```

## 📡 Topics & Interfaces

### Subscribed Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | Raw LiDAR data |
| `/odom` | `geometry_msgs/PoseStamped` | Odometry node | Current robot pose |
| `/goal` | `geometry_msgs/PoseStamped` | Manual/autonomous | Target position |

### Published Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/obstacles` | `geometry_msgs/PoseStamped` | Obstacle Detection | Detected obstacle positions |
| `/occupancy_grid` | `nav_msgs/OccupancyGrid` | Obstacle Detection | 2D grid of obstacles (visualization) |
| `/path` | `nav_msgs/Path` | RRT Planner | Planned collision-free path |
| `/plan_status` | `std_msgs/String` | RRT Planner | Planning status (SUCCESS/FAILED/REPLANNING) |
| `/cmd_vel` | `geometry_msgs/Twist` | Path Executor | Velocity commands for rover |

## 🎯 Node Descriptions

### 1. Obstacle Detection Node
**Executable**: `obstacle_detection`

Processes LiDAR scans and detects obstacles.

#### Algorithm Flow:
```
1. Subscribe to /scan (LaserScan)
2. For each beam:
   a. Skip invalid/out-of-range measurements
   b. Convert polar → Cartesian coordinates
   c. If distance < threshold → mark as obstacle
3. Update 2D occupancy grid
4. Publish obstacle positions and grid
```

#### Parameters:
```yaml
obstacle_distance_threshold: 1.5  # Obstacle detection distance (m)
max_scan_range: 10.0              # Max range to process (m)
grid_size: 20.0                   # Occupancy grid size (m x m)
grid_resolution: 0.1              # Grid cell size (m)
frame_id: "laser"                 # LiDAR frame ID
```

#### Example Usage:
```bash
ros2 launch rover_navigation obstacle_detection.launch.py \
  obstacle_threshold:=1.5 \
  max_range:=10.0
```

---

### 2. RRT Planner Node
**Executable**: `rrt_planner`

Plans collision-free paths using RRT algorithm with dynamic re-planning.

#### RRT Algorithm:
```
Initialize:
  tree = [start_node]

Main Loop (up to max_iterations):
  1. Sample random point:
     - 90% uniform random in workspace
     - 10% biased toward goal (goal biasing)
  
  2. Find nearest tree node using Euclidean distance
  
  3. Extend tree:
     - Create new node at step_size distance toward sample
     - Check collision on connecting segment
     - If collision-free, add to tree
  
  4. Check goal reached:
     - If new node close to goal (< tolerance)
     - Try direct connection to goal
     - If collision-free, return path
  
  5. Early exit if path found after 100 iterations

Return: Path from start to goal or None
```

#### Dynamic Re-planning:
```
Main Planning Loop (5 Hz):
  1. Monitor incoming obstacles
  2. If path exists, validate against new obstacles
  3. If collision detected:
     → Log warning
     → Publish REPLANNING status
     → Clear current path
  4. If no path or replanning needed:
     → Run RRT algorithm
     → Publish new path or FAILED status
```

#### Collision Detection:
Uses circle-based approximation with clearance:
```
For each obstacle (center, radius):
  - Compute closest point on path segment to obstacle
  - If distance < radius → collision detected
  - Use parametric line segment equation: P(t) = P1 + t(P2-P1), t ∈ [0,1]
```

#### Parameters:
```yaml
step_size: 0.5                    # RRT extension distance (m)
max_iterations: 5000              # Max planning iterations
obstacle_clearance: 0.3           # Safety margin (m)
robot_radius: 0.2                 # Collision radius (m)
replan_frequency: 5.0             # Re-planning check rate (Hz)
goal_tolerance: 0.3               # Distance to goal for success (m)
workspace_size: 20.0              # Workspace bounds (m x m)
frame_id: "odom"                  # Planning frame ID
```

#### Example Usage:
```bash
ros2 launch rover_navigation rrt_planner.launch.py \
  step_size:=0.5 \
  max_iterations:=5000 \
  obstacle_clearance:=0.3
```

#### Performance Tips:
- **Increase `step_size`** for faster (but less smooth) paths
- **Decrease `max_iterations`** for real-time constraints
- **Increase `obstacle_clearance`** for more cautious planning
- **Use smaller `workspace_size`** for constrained environments

---

### 3. Path Executor Node
**Executable**: `path_executor`

Executes planned paths and controls rover velocity.

#### Pure Pursuit Algorithm:
```
Main Control Loop (10 Hz):
  1. Update waypoint index based on proximity
     - Check distance to current waypoint
     - Advance index if within path_tolerance
  
  2. Find lookahead point:
     - Traverse path waypoints
     - Find point at lookahead_distance ahead
     - Use goal if lookahead exceeds path end
  
  3. Compute velocity command:
     a. Calculate angle to lookahead point
     b. Compute heading error (angle difference)
     c. Angular velocity: proportional to heading error
        - Clamp to ±angular_speed
     d. Linear velocity: depends on alignment
        - Full speed when aligned (cos(error) ≈ 1)
        - Reduced when turning (cos(error) < 1)
        - Zero when error > 90°
```

#### Velocity Commands:
```
cmd_vel.linear.x  = linear_speed * max(0, cos(heading_error))
cmd_vel.angular.z = clamp(heading_error * 2.0, -angular_speed, angular_speed)
```

#### Parameters:
```yaml
linear_speed: 0.5                 # Forward velocity (m/s)
angular_speed: 1.0                # Max rotation speed (rad/s)
lookahead_distance: 0.5           # Pure pursuit distance (m)
path_tolerance: 0.3               # Distance to waypoint (m)
```

#### Example Usage:
```bash
ros2 run rover_navigation path_executor \
  --ros-args -p linear_speed:=0.5 -p angular_speed:=1.0
```

#### Tuning Guide:
- **Increase `lookahead_distance`** for smoother turns
- **Decrease `linear_speed`** for more precise control
- **Increase `angular_speed`** for tighter turns

---

## 🎮 Usage Examples

### Launch Individual Nodes

**Obstacle Detection Only:**
```bash
ros2 launch rover_navigation obstacle_detection.launch.py
```

**RRT Planner Only:**
```bash
ros2 launch rover_navigation rrt_planner.launch.py
```

**Full Navigation Stack:**
```bash
ros2 launch rover_navigation full_navigation.launch.py
```

### Send Goal to Planner

From another terminal, publish a goal:

```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 5.0, y: 5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

### Monitor Planning Status

```bash
ros2 topic echo /plan_status
```

### Visualize in RViz

```bash
# Terminal 1: Launch navigation
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2: Open RViz
ros2 run rviz2 rviz2

# In RViz, add displays:
# - LaserScan: /scan (points)
# - Grid: /occupancy_grid
# - Path: /path (as line strip)
# - PoseStamped: /odom (robot pose)
```

## 🔍 Re-planning Logic in Detail

The system implements continuous monitoring for dynamic replanning:

### Trigger Conditions:
1. **New obstacle detected** in existing path
2. **Obstacle moved** (within grid cells of existing path)
3. **Path becomes invalid** (collision with updated obstacles)

### Re-planning Procedure:
```python
def planning_callback():
    if goal_pose is None or current_pose is None:
        return
    
    # Check path validity
    if current_path is not None:
        if not is_path_valid(current_path):
            log_warn("Path collides with new obstacle")
            publish_status("REPLANNING")
            current_path = None
    
    # Plan if needed
    if current_path is None:
        plan_path()
```

### Validation Check:
```python
def is_path_valid(path):
    """Check if path is still collision-free"""
    for i in range(len(path.poses) - 1):
        x1, y1 = path.poses[i].position
        x2, y2 = path.poses[i+1].position
        
        # Check collision for each segment
        if not is_collision_free(x1, y1, x2, y2):
            return False  # Path invalid
    
    return True  # Path still valid
```

### Performance:
- **Validation**: O(n) where n = number of waypoints
- **Planning**: O(m) where m = max_iterations (typically 5000)
- **Frequency**: 5 Hz ensures real-time responsiveness

## 📊 Algorithm Complexity

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| Obstacle Detection | O(n) | n = number of beams |
| Nearest Neighbor | O(m) | m = tree size (use KD-tree for O(log m)) |
| Collision Check | O(k) | k = number of obstacles |
| Path Validation | O(n) | n = path waypoints |
| Total Planning | O(m·k) per iteration | m iterations × k obstacles |

## ⚙️ Tuning Guide

### For Smooth Paths:
```yaml
# Parameters
step_size: 1.0              # Larger steps → fewer waypoints
max_iterations: 10000       # More iterations → better optimization
lookahead_distance: 1.0     # Smoother tracking
```

### For Speed:
```yaml
step_size: 0.3              # Smaller steps → faster computation
max_iterations: 1000        # Fewer iterations for quick plans
linear_speed: 1.0           # Faster movement
```

### For Tight Spaces:
```yaml
step_size: 0.2              # Finer granularity
obstacle_clearance: 0.5     # More conservative
robot_radius: 0.3           # Account for size
```

## 🐛 Debugging

### Check Topics:
```bash
# List all topics
ros2 topic list

# Echo obstacle positions
ros2 topic echo /obstacles

# Echo planned path
ros2 topic echo /path

# Echo velocity commands
ros2 topic echo /cmd_vel
```

### Enable Debug Logging:
```bash
ros2 run rover_navigation obstacle_detection \
  --ros-args --log-level rover_navigation:=DEBUG
```

### Visualize Occupancy Grid:
```bash
# Terminal 1
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2
ros2 run rviz2 rviz2

# Add OccupancyGrid display: /occupancy_grid
```

## 🔗 Integration with Existing Nodes

Your rover should have these nodes already running:

1. **LiDAR Driver**: Publishing `/scan`
2. **Odometry Node**: Publishing `/odom`
3. **Motor Controller**: Subscribing to `/cmd_vel`

Integration is automatic through topic names.

## 📝 Example Mission

```bash
# Terminal 1: Start navigation stack
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2: Send goal
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 10.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'

# Watch rover navigate autonomously, avoiding obstacles
# Monitor with: ros2 topic echo /plan_status
```

## 🚨 Limitations & Future Improvements

### Current Limitations:
- 2D planning only (no elevation changes)
- No dynamic velocity replanning
- Single robot (no multi-robot coordination)

### Planned Enhancements:
- RRT* (asymptotically optimal)
- Bidirectional RRT for faster planning
- KD-tree for efficient nearest-neighbor
- Motion primitives for non-holonomic vehicles
- Velocity ramp limiting

## 📚 References

- **RRT Algorithm**: LaValle, S. M. (1998). "Rapidly-exploring random trees"
- **Collision Detection**: Real-time collision detection for dynamic scenes
- **Pure Pursuit**: Implementation of real-time autonomous vehicle control

## 💡 Tips

1. **Tune obstacle_clearance** based on your rover's actual size and acceleration
2. **Use occupancy_grid** in RViz to debug obstacle detection
3. **Monitor plan_status** to verify re-planning events
4. **Increase lookahead_distance** if rover oscillates around path
5. **Decrease step_size** for finer path quality in tight spaces

---

**Created for ROS 2 Jazzy with Python 3.11+**
