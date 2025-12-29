# EKF Node `enabled` Parameter Test Procedure

## Hypothesis
In ROS 2 Jazzy, `ekf_node` from `robot_localization` requires `enabled: true` parameter to remain running. Without this parameter, the node initializes but exits immediately.

## Configuration Change
Added `enabled: true` to `/home/caterpillar/ros2_ws/config/ekf.yaml` under `ros__parameters`.

## Minimal Test Procedure

### Important Note
**The PRIMARY test (does node stay running?) can be performed even if `/odom` is not publishing.** The `enabled: true` parameter should prevent the node from exiting immediately, regardless of whether input data is available. The node should remain running and wait for data.

### Step 0 (Optional): Start Odometry Node for Full Validation
If `/odom` is not currently publishing, start the odometry node in a separate terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rover_navigation odometry_node
```
**Expected:** Node starts and begins publishing `/odom` messages.

### Step 1: Verify `/odom` is Publishing (Optional but Recommended)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /odom --once
```
**Expected:** Should see one Odometry message with correct frame IDs.
**Note:** If this fails, you can still test the `enabled` parameter - the node should stay running even without input data.

### Step 2: Launch EKF Node (PRIMARY TEST)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/config/ekf.yaml
```
**Expected:** Node should remain running (does NOT return to shell prompt immediately).
**This is the KEY test:** With `enabled: true`, the node should stay alive even if `/odom` is not publishing yet. Without `enabled: true`, the node would exit immediately.

### Step 3: Verify Node is Active
In a **separate terminal**:
```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
```
**Expected:** Should see `/ekf_filter_node` in the output.

### Step 4: Verify Transform Publication
In the **same separate terminal**:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
**Expected:** Should display transform data continuously (may take a few seconds to start).

### Step 5: Verify Parameter is Set
```bash
ros2 param get /ekf_filter_node enabled
```
**Expected:** Should return `Boolean value is: True`

## Success Criteria
- ✅ `ekf_node` stays running in foreground (does not exit)
- ✅ `ros2 node list` shows `/ekf_filter_node`
- ✅ `ros2 run tf2_ros tf2_echo odom base_link` works and shows transforms
- ✅ `ros2 param get /ekf_filter_node enabled` returns `True`

## Failure Indicators
- ❌ Node exits immediately after initialization
- ❌ `/ekf_filter_node` does not appear in `ros2 node list`
- ❌ Transform echo shows "Could not find transform" errors

## Notes
- This test validates process lifecycle behavior only
- No IMU, GPS, or hardware changes required
- No EKF math or noise parameter modifications
- Focus is on ROS 2 Jazzy + robot_localization lifecycle behavior

