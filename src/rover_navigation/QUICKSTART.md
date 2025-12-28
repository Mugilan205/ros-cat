# 🚀 Quick Start Guide

## Installation (5 minutes)

### 1. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select rover_navigation
source install/setup.bash
```

### 2. Verify Installation
```bash
ros2 pkg list | grep rover_navigation
```

Should output:
```
rover_navigation
```

## Running the System

### Option A: Full Navigation Stack (Recommended)
```bash
# Terminal 1: Launch all nodes
ros2 launch rover_navigation full_navigation.launch.py

# Terminal 2: Send a goal
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'

# Terminal 3: Monitor status
ros2 topic echo /plan_status
```

### Option B: Step-by-Step
```bash
# Terminal 1: Obstacle detection
ros2 launch rover_navigation obstacle_detection.launch.py

# Terminal 2: RRT planner
ros2 launch rover_navigation rrt_planner.launch.py

# Terminal 3: Path executor
ros2 run rover_navigation path_executor

# Terminal 4: Send goal
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "odom"},
  pose: {position: {x: 10.0, y: 5.0, z: 0.0}}
}'
```

## Visualization

### RViz Setup
```bash
ros2 run rviz2 rviz2

# Add displays:
# 1. LaserScan: /scan (topic) - white points
# 2. Path: /path (topic) - green line
# 3. OccupancyGrid: /occupancy_grid - obstacles
# 4. PoseStamped: /odom - robot pose
```

## Monitoring

### Check Topics
```bash
# List all rover_navigation topics
ros2 topic list | grep -E "(scan|obstacles|path|cmd_vel|plan_status)"

# Monitor specific topic
ros2 topic echo /plan_status
ros2 topic echo /obstacles
ros2 topic echo /cmd_vel
```

### Check Nodes
```bash
ros2 node list | grep -E "obstacle|planner|executor"

# Node info
ros2 node info /obstacle_detection
ros2 node info /rrt_planner
ros2 node info /path_executor
```

## Configuration

### Modify Parameters
Edit `/launch/full_navigation.launch.py`:

```python
# Change obstacle detection threshold
{'obstacle_distance_threshold': 2.0},  # Default: 1.5

# Change RRT parameters
{'step_size': 1.0},                   # Default: 0.5
{'max_iterations': 10000},            # Default: 5000

# Change execution speed
{'linear_speed': 0.8},                # Default: 0.5
{'angular_speed': 1.5},               # Default: 1.0
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select rover_navigation
source install/setup.bash
ros2 launch rover_navigation full_navigation.launch.py
```

## Testing

### Run Test Scenarios
```bash
# Scenario 1: Simple goal without obstacles
ros2 run rover_navigation test_navigation --scenario 1

# Scenario 2: Goal with static obstacle
ros2 run rover_navigation test_navigation --scenario 2

# Scenario 3: Dynamic obstacles (replanning)
ros2 run rover_navigation test_navigation --scenario 3 --verbose
```

## Troubleshooting

### No obstacles detected
```bash
# Check LiDAR is publishing
ros2 topic hz /scan
# Should show frequency > 1 Hz

# Check obstacle threshold
# Increase obstacle_distance_threshold in parameters
```

### Path not generated
```bash
# Check goal received
ros2 topic echo /goal
# Should show goal position

# Check for obstacles blocking path
ros2 topic echo /occupancy_grid
# Visual check: can you see a path around obstacles?

# Increase max_iterations for complex environments
```

### Robot oscillates on path
```bash
# Increase lookahead_distance in path_executor
{'lookahead_distance': 1.0},  # from 0.5

# Or decrease angular speed
{'angular_speed': 0.5},  # from 1.0
```

### Too many replans
```bash
# Increase clustering_distance to group obstacles
# Or decrease obstacle_clearance
{'obstacle_clearance': 0.2},  # from 0.3
```

## Next Steps

1. **Read README.md** for detailed documentation
2. **Read TECHNICAL_GUIDE.md** for algorithm explanations
3. **Tune parameters** for your specific rover
4. **Run on real hardware** with your LiDAR and motors
5. **Extend the system** with additional features

## Integration with Your Rover

Your current setup should have:
- ✅ LiDAR driver publishing `/scan`
- ✅ Odometry publishing `/odom`  
- ✅ Motor controller subscribing to `/cmd_vel`

If not:
1. **LiDAR Driver**: Use rplidar_ros package
   ```bash
   ros2 launch rplidar_ros rplidar_a1_launch.py
   ```

2. **Odometry**: Use your robot's encoder driver or IMU

3. **Motor Controller**: Use your BTS motor driver node

## File Structure

```
rover_navigation/
├── README.md                    # Full documentation
├── TECHNICAL_GUIDE.md          # Deep dive into algorithms
├── setup.py                    # Package configuration
├── setup.cfg
├── package.xml
├── rover_navigation/           # Python package
│   ├── __init__.py
│   ├── obstacle_detection.py   # LiDAR processing
│   ├── rrt_planner.py         # RRT planning
│   └── path_executor.py       # Velocity control
├── launch/                     # Launch files
│   ├── obstacle_detection.launch.py
│   ├── rrt_planner.launch.py
│   └── full_navigation.launch.py
└── test_navigation.py         # Test scenarios
```

## Support

For issues or questions:
1. Check the README.md or TECHNICAL_GUIDE.md
2. Review launch file parameters
3. Check ROS 2 node logs: `ros2 run rover_navigation <executable> --ros-args --log-level DEBUG`
4. Monitor topic messages: `ros2 topic echo <topic>`

---

**Happy autonomous navigation! 🚀**
