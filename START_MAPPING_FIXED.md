# Fixed: How to Start SLAM Mapping

## The Two Issues You Had

1. **Wrong launch command**: `ros2 launch full_mapping.launch.py` ❌
   - Should be: `ros2 launch launch/full_mapping.launch.py` ✓

2. **TF synchronization issue**: SLAM Toolbox queue full
   - Fixed by: Adding odometry node and proper timing

## Quick Fix - Use This Command

```bash
cd ~/ros2_ws
ros2 launch launch/full_mapping.launch.py
```

I've updated the launch file to:
- Add odometry node (publishes odom → base_link TF)
- Use `async_slam_toolbox_node` (was incorrectly using sync)
- Increase timing delays for proper TF chain setup

## Or Use the Script (Even Easier)

```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```

This starts everything in separate terminals with proper timing.

## What Changed in the Launch File

1. **Added odometry node** - Needed for odom → base_link transform
2. **Changed to async_slam_toolbox_node** - Matches your error messages
3. **Increased delays** - Gives more time for TF transforms to be ready

## Verification After Starting

Wait 5-10 seconds, then check:

```bash
# Check TF transforms exist
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom base_link

# Check topics
ros2 topic list | grep -E "scan|map|odom"

# Check SLAM is running (no more queue errors)
ros2 topic hz /map
```

If you still see queue errors, the script method is more reliable.












