# EKF TF Issue: Quick Fix Guide

## Problem
`ros2 run tf2_ros tf2_echo odom base_link` fails with: "frame does not exist"

## Root Cause
EKF filter hasn't initialized because `/odom` has **0 publishers** (odometry node not running/discovered).

## Quick Fix (3 Commands)

```bash
# 1. Kill existing odometry node
pkill -9 -f odometry_node

# 2. Set environment (if not already set)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
source /home/caterpillar/ros2_ws/install/setup.bash

# 3. Start odometry node
ros2 run rover_navigation odometry_node
```

## Verify Fix (Wait 5 seconds, then run)

```bash
# Check odometry is publishing
ros2 topic info /odom | grep "Publisher count: 1" && echo "✅ OK" || echo "❌ FAIL"

# Check EKF is publishing filtered odometry
timeout 2 ros2 topic echo /odometry/filtered --once > /dev/null && echo "✅ OK" || echo "❌ FAIL"

# Check TF transform works
ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -3 | grep -q "At time" && echo "✅ OK" || echo "❌ FAIL"
```

All three should show ✅ OK.

## Why This Works

1. Odometry node publishes `/odom` messages
2. EKF receives first message → filter initializes
3. EKF starts publishing `/odometry/filtered` and TF transforms
4. `odom → base_link` transform becomes available in TF tree

## Detailed Documentation

- **Full diagnosis**: `EKF_TF_ROOT_CAUSE.md`
- **Complete test**: `test_ekf_tf_complete.sh`
- **Detailed analysis**: `EKF_TF_DIAGNOSIS.md`



