# Fix: SLAM Toolbox Message Filter Queue Full

## Problem
SLAM Toolbox is dropping messages because the TF transform queue is full. This happens when:
- TF transforms are missing (base_link → laser)
- TF transforms are published too slowly
- robot_state_publisher is not running correctly

## Solutions

### Solution 1: Launch Correctly with File Path

The launch file is in the workspace root, not in a package. Use:

```bash
cd ~/ros2_ws
ros2 launch launch/full_mapping.launch.py
```

**NOT:** `ros2 launch full_mapping.launch.py` ❌

### Solution 2: Verify TF Transform Chain

Check if transforms are being published:

```bash
# Check if base_link → laser transform exists
ros2 run tf2_ros tf2_echo base_link laser

# Check full transform chain
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Solution 3: Ensure robot_state_publisher is Running

The launch file should start robot_state_publisher. Verify:

```bash
ros2 node list | grep robot_state_publisher
```

If not running, the URDF file path might be wrong. Check:

```bash
ls -la ~/ros2_ws/urdf/your_robot.urdf
```

### Solution 4: Fix robot_state_publisher Path

If URDF doesn't exist, you have two options:

**Option A: Create proper package structure**

Create a package for your robot description:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake simple_rover_description
mkdir -p simple_rover_description/urdf
# Copy your URDF there
```

Then update the launch file to use the package.

**Option B: Use absolute path correctly**

The launch file uses:
```python
arguments=['/home/caterpillar/ros2_ws/urdf/your_robot.urdf']
```

Make sure this file exists and has the correct laser transform.

### Solution 5: Verify URDF Has Laser Transform

Check your URDF file (`~/ros2_ws/urdf/your_robot.urdf`) contains:

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link" />
  <child link="laser" />
  <origin xyz="0.12 0 0.25" rpy="0 0 0" />
</joint>
```

### Solution 6: Increase SLAM Message Queue Size

If transforms are slow, increase the queue size in slam_params.yaml:

```yaml
slam_toolbox:
  ros__parameters:
    # Increase message queue
    scan_buffer_size: 50  # Default is 20
```

### Solution 7: Ensure Odometry is Publishing TF

SLAM needs odom → base_link transform. Check:

```bash
# Check if odometry node is running
ros2 node list | grep odom

# Check if /odom topic exists
ros2 topic hz /odom

# Check odom → base_link transform
ros2 run tf2_ros tf2_echo odom base_link
```

### Quick Fix: Complete Launch Sequence

Run everything in order:

**Terminal 1:**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start robot_state_publisher manually first
ros2 run robot_state_publisher robot_state_publisher \
  /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```

**Terminal 2:**
```bash
source ~/ros2_ws/install/setup.bash

# Start RPLidar
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

**Terminal 3:**
```bash
source ~/ros2_ws/install/setup.bash

# Start odometry (this publishes odom → base_link)
ros2 run rover_navigation odometry_node
```

**Terminal 4:**
```bash
source ~/ros2_ws/install/setup.bash

# Wait a few seconds, then start SLAM
sleep 3
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p scan_topic:=/scan \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p base_frame:=base_link
```

**Verify TF chain:**
```bash
ros2 run tf2_ros tf2_echo map base_link
```

## Recommended: Create Proper Launch Package

For better organization, create a proper ROS2 package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python rover_bringup
mkdir -p rover_bringup/launch rover_bringup/config

# Move launch file
mv ~/ros2_ws/launch/full_mapping.launch.py rover_bringup/launch/

# Create setup.py
# ... (setup.py configuration)
# Build and use: ros2 launch rover_bringup full_mapping.launch.py
```









