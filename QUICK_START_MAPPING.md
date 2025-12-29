# Quick Start: SLAM Mapping (Fixed)

## The Problem

You were seeing:
```
Message Filter dropping message: frame 'laser' ... 'discarding message because the queue is full'
```

This happens because SLAM Toolbox needs TF transforms in the correct order, but they weren't available when SLAM started.

## The Solution

### Option 1: Use the Launch Script (Easiest)

```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```

This script starts everything in the correct order with proper delays.

### Option 2: Use Launch File with Correct Syntax

The launch file exists at `~/ros2_ws/launch/full_mapping.launch.py`. To use it:

```bash
cd ~/ros2_ws
ros2 launch launch/full_mapping.launch.py
```

**Note:** You must use the file path `launch/full_mapping.launch.py`, NOT just `full_mapping.launch.py`

### Option 3: Manual Start (Recommended for Testing)

Start components in this exact order:

**Terminal 1: robot_state_publisher**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```

**Terminal 2: RPLidar**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

**Terminal 3: Odometry** (Waits 2 seconds after Terminal 1)
```bash
sleep 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rover_navigation odometry_node
```

**Terminal 4: EKF** (Waits 3 seconds after Terminal 3)
```bash
sleep 3
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_localization ekf_node --ros-args -p config_file:=/home/caterpillar/ros2_ws/config/ekf.yaml
```

**Terminal 5: SLAM Toolbox** (Waits 3 seconds after Terminal 4)
```bash
sleep 3
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

## Verification

After everything starts, verify the TF chain:

```bash
# Check base_link → laser transform
ros2 run tf2_ros tf2_echo base_link laser

# Check odom → base_link transform
ros2 run tf2_ros tf2_echo odom base_link

# Check map → odom transform (after SLAM initializes)
ros2 run tf2_ros tf2_echo map odom

# View full TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

Check topics:

```bash
# Should see /scan publishing
ros2 topic hz /scan

# Should see /map after SLAM initializes (may take 10-30 seconds)
ros2 topic echo /map --once

# Should see /odom
ros2 topic hz /odom
```

## Why This Fixes It

The launch file uses `TimerAction` to delay startup, but the timing might not be sufficient. The key is:

1. **robot_state_publisher** must start FIRST (publishes base_link → laser)
2. **Odometry** must start SECOND (publishes odom → base_link)
3. **SLAM** must start LAST (needs both transforms to exist)

If SLAM starts before the transforms are ready, it fills up its message filter queue and starts dropping messages.

## Troubleshooting

**Still seeing queue full errors?**

1. Increase delays between node startups
2. Check if transforms are actually being published:
   ```bash
   ros2 topic echo /tf --once
   ```

3. Verify URDF is correct:
   ```bash
   cat ~/ros2_ws/urdf/your_robot.urdf | grep -A 5 "laser_joint"
   ```

4. Check robot_state_publisher is running:
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

**No /map topic?**

- Wait 10-30 seconds for SLAM to initialize
- Make sure robot is moving (SLAM needs motion to initialize)
- Check SLAM logs for errors

**TF errors?**

- Verify URDF file exists: `ls -la ~/ros2_ws/urdf/your_robot.urdf`
- Check transform chain is complete
- Ensure all nodes are running












