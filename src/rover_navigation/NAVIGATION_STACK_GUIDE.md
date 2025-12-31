# A* + LWB Navigation Stack Guide

## Overview

This document describes the new A* global planner + Local Weighted Band (LWB) local planner navigation stack that replaces the RRT planner.

## Architecture

```
┌─────────────────┐
│   SLAM Toolbox  │ → /map (OccupancyGrid)
└────────┬────────┘
         │
┌────────▼──────────────────┐
│   A* Global Planner       │ → /global_path (Path)
└────────┬──────────────────┘
         │
┌────────▼──────────────────┐
│   LWB Local Planner       │ → /cmd_vel (Twist)
│   (Velocity Sampling)     │ → /local_trajectory (Path)
└────────┬──────────────────┘
         │
┌────────▼──────────────────┐
│   Motor Controller        │
└───────────────────────────┘
```

## Nodes

### 1. A* Global Planner (`a_star_planner`)

**Purpose**: Plans optimal global path from current pose to goal using grid-based A* search.

**Subscribes**:
- `/map` (nav_msgs/OccupancyGrid): Map from SLAM Toolbox
- `/goal` (geometry_msgs/PoseStamped): Target goal pose

**Publishes**:
- `/global_path` (nav_msgs/Path): Global path from start to goal
- `/planner/debug/state` (std_msgs/String): Planner state

**Key Parameters**:
- `inflation_radius` (default: 0.5 m): Obstacle inflation for safety
- `allow_unknown` (default: False): Allow planning through unknown cells
- `plan_frequency` (default: 1.0 Hz): Planning frequency
- `tolerance_xy` (default: 0.2 m): Goal position tolerance

**Features**:
- 8-connected grid search
- Euclidean heuristic
- Obstacle inflation
- Automatic replanning on new goals

### 2. LWB Local Planner (`lwb_planner`)

**Purpose**: Reactive local planning with obstacle avoidance using velocity sampling.

**Subscribes**:
- `/global_path` (nav_msgs/Path): Global path from A* planner
- `/scan` (sensor_msgs/LaserScan): LiDAR data for local obstacles
- `/odom` (geometry_msgs/PoseStamped): Current robot pose

**Publishes**:
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/local_trajectory` (nav_msgs/Path): Best sampled trajectory
- `/planner/debug/vel_samples` (std_msgs/Float32MultiArray): Debug velocity samples

**Key Parameters**:
- `max_vel_x` (default: 0.5 m/s): Maximum linear velocity
- `max_vel_theta` (default: 1.0 rad/s): Maximum angular velocity
- `vx_samples` (default: 20): Number of linear velocity samples
- `vtheta_samples` (default: 40): Number of angular velocity samples
- `sim_time` (default: 1.5 s): Forward simulation time
- `path_distance_bias` (default: 32.0): Weight for path alignment cost
- `goal_distance_bias` (default: 24.0): Weight for goal distance cost
- `obstacle_cost_bias` (default: 1.0): Weight for obstacle cost
- `local_costmap_size` (default: 4.0 m): Size of local costmap
- `inflation_radius` (default: 0.3 m): Local obstacle inflation

**Features**:
- Velocity space sampling (v, ω)
- Forward trajectory simulation
- Multi-objective cost function (obstacle, path, goal, smoothness)
- Local costmap from LiDAR
- Runs at 15 Hz

### 3. Planner Manager (`planner_manager`)

**Purpose**: Coordinates global and local planners, monitors execution.

**Subscribes**:
- `/goal` (geometry_msgs/PoseStamped): Goal pose
- `/global_path` (nav_msgs/Path): Global path
- `/odom` (geometry_msgs/PoseStamped): Current pose

**Publishes**:
- `/planner/debug/state` (std_msgs/String): Planner state

**States**:
- `IDLE`: No active goal
- `PLANNING`: Planning in progress
- `ACTIVE`: Following path
- `GOAL_REACHED`: Goal achieved
- `FAILED`: Planning failed
- `WAITING_FOR_PATH`: Waiting for global path

### 4. Navigation Dashboard (`navigation_dashboard`)

**Purpose**: Visualization and debugging for development.

**Subscribes**:
- `/odom`: Robot pose
- `/global_path`: Global path
- `/local_trajectory`: Local trajectory
- `/cmd_vel`: Velocity commands
- `/scan`: Laser scan
- `/planner/debug/state`: Planner state

**Publishes**:
- `/planner/debug/obstacles` (visualization_msgs/MarkerArray): Obstacle markers
- `/planner/debug/robot_pose` (visualization_msgs/Marker): Robot pose marker

## Usage

### Launch Navigation Stack

```bash
# Basic launch (uses default parameters)
ros2 launch rover_navigation a_star_lwb_navigation.launch.py

# With custom parameters
ros2 launch rover_navigation a_star_lwb_navigation.launch.py \
  frame_id:=map \
  robot_frame:=base_link \
  odom_frame:=odom
```

### Send Goal

```bash
ros2 topic pub -1 /goal geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

### Monitor Status

```bash
# Planner state
ros2 topic echo /planner/debug/state

# Global path
ros2 topic echo /global_path

# Velocity commands
ros2 topic echo /cmd_vel

# Local trajectory
ros2 topic echo /local_trajectory
```

### Visualize in RViz2

1. Launch RViz2: `rviz2`
2. Add displays:
   - **Map**: `/map` (OccupancyGrid)
   - **Global Path**: `/global_path` (Path, color: blue)
   - **Local Trajectory**: `/local_trajectory` (Path, color: green)
   - **Robot Pose**: `/planner/debug/robot_pose` (Marker)
   - **Obstacles**: `/planner/debug/obstacles` (MarkerArray)
   - **Laser Scan**: `/scan` (LaserScan)

## Integration with Existing Stack

The new navigation stack integrates seamlessly with your existing setup:

1. **Motor Controller**: Subscribes to `/cmd_vel` (unchanged)
2. **SLAM Toolbox**: Publishes `/map` (unchanged)
3. **EKF**: Fuses `/odom_raw` (unchanged)
4. **Odometry Node**: Publishes `/odom` (unchanged)
5. **RPLidar**: Publishes `/scan` (unchanged)

## Topic Graph

```
/map → [A* Planner] → /global_path
/goal → [A* Planner]
       → [Planner Manager]
/scan → [LWB Planner] → /cmd_vel → [Motor Controller]
/odom → [LWB Planner]
       → [Planner Manager]
       → [Dashboard]
/global_path → [LWB Planner] → /local_trajectory
             → [Planner Manager]
             → [Dashboard]
```

## Tuning Guide

### For Faster Navigation
- Increase `max_vel_x` (e.g., 0.8 m/s)
- Increase `max_vel_theta` (e.g., 1.5 rad/s)
- Increase `sim_time` for longer lookahead
- Increase `path_distance_bias` for better path following

### For Safer Navigation
- Increase `inflation_radius` in both planners
- Decrease `max_vel_x` and `max_vel_theta`
- Increase `obstacle_cost_bias`
- Increase `vx_samples` and `vtheta_samples` for more thorough search

### For Tight Spaces
- Decrease `local_costmap_size` (e.g., 3.0 m)
- Increase `local_costmap_resolution` (e.g., 0.05 m)
- Increase `inflation_radius` for safety margins

## Debugging

### Check Planner State
```bash
ros2 topic echo /planner/debug/state
```

### Monitor Path Quality
```bash
# Check path length
ros2 topic echo /global_path --once | grep -A 5 "poses"

# Check velocity commands
ros2 topic echo /cmd_vel
```

### Visualize Costmaps
The LWB planner uses local costmaps internally. To debug:
1. Monitor `/scan` to ensure LiDAR data is arriving
2. Check `/local_trajectory` to see sampled paths
3. Use dashboard markers to visualize obstacles

## Migration from RRT Planner

The new stack is independent of the RRT planner. You can:

1. **Keep both systems**: Run either navigation stack based on your needs
2. **Switch gradually**: Test A*+LWB alongside RRT before full migration
3. **Replace completely**: Use only A*+LWB by launching `a_star_lwb_navigation.launch.py` instead of `full_navigation.launch.py`

## Known Limitations

1. **2D Planning Only**: No elevation or 3D obstacles
2. **Differential Drive Only**: Optimized for differential drive kinematics
3. **Single Robot**: No multi-robot coordination
4. **No Dynamic Obstacles**: Assumes static map from SLAM

## Future Enhancements

- Dynamic obstacle tracking
- Velocity obstacle method integration
- Recovery behaviors (rotate in place, backward motion)
- Path smoothing (B-splines, bezier curves)
- Adaptive sampling densities
- Machine learning for cost weights

## References

- A* Algorithm: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
- DWB Local Planner: Macenski, S., et al. "The Dynamic Window Approach to Collision Avoidance"
- Costmap 2D: Navigation stack costmap implementation



















