# Odometry Node ESP Relay - Modification Summary

## Overview
Modified `odometry_node.py` to act as a **pass-through relay** for ESP-provided odometry, removing all internal velocity/encoder integration logic.

---

## What Was REMOVED ❌

### 1. **Wheel/Encoder Parameters**
- `wheel_radius` parameter
- `wheel_base` parameter  
- `encoder_resolution` parameter

### 2. **Internal State Variables**
- `self.x`, `self.y`, `self.theta` (position/orientation state)
- `self.linear_x`, `self.angular_z` (velocity state)

### 3. **Velocity Integration Logic**
- `cmd_vel_sub` subscription to `/cmd_vel`
- `cmd_vel_callback()` method
- All differential drive kinematics calculations
- Delta position/orientation integration
- Theta normalization loops

### 4. **Timer-Based Publishing**
- `self.timer` (periodic publishing at 20Hz)
- `publish_odom()` method with internal computation

### 5. **Helper Functions**
- `euler_to_quaternion()` static method (no longer needed)

### 6. **Internal Computation**
- All position/orientation calculations
- All velocity integration
- All encoder tick processing

---

## What Was ADDED ✅

### 1. **ESP Odometry Subscription**
```python
self.esp_odom_sub = self.create_subscription(
    Odometry,
    esp_odom_topic,  # Default: "/esp/odom"
    self.esp_odom_callback,
    10
)
```

### 2. **Pass-Through Callback**
- `esp_odom_callback()` method that:
  - Receives ESP odometry message
  - Copies pose data (position, orientation)
  - Copies twist data (if provided)
  - Copies covariance matrices (if provided)
  - Sets correct frame IDs (`odom` → `base_link`)
  - Republishes to `/odom`

### 3. **Configurable Input Topic**
- Parameter: `esp_odom_topic` (default: `/esp/odom`)
- Can be changed via launch file or parameter override

### 4. **Enhanced Logging**
- Logs when ESP odometry is received
- Periodic status logs (every 50 messages)
- Shows position/orientation being relayed

---

## Data Flow

```
ESP Module
    ↓
Publishes: /esp/odom (nav_msgs/Odometry)
    ↓
odometry_node subscribes
    ↓
esp_odom_callback() receives message
    ↓
Copies data + sets frame IDs
    ↓
Publishes: /odom (nav_msgs/Odometry)
    ↓
robot_localization EKF subscribes
    ↓
EKF processes and publishes TF: odom → base_link
    ↓
SLAM Toolbox uses TF for mapping
```

---

## Message Format Requirements

### Input: `/esp/odom` (from ESP)
- **Type:** `nav_msgs/msg/Odometry`
- **Required fields:**
  - `pose.pose.position` (x, y, z)
  - `pose.pose.orientation` (quaternion)
- **Optional fields:**
  - `pose.covariance` (36-element array)
  - `twist.twist` (linear/angular velocities)
  - `twist.covariance` (36-element array)

### Output: `/odom` (to EKF)
- **Type:** `nav_msgs/msg/Odometry`
- **Frame IDs:**
  - `header.frame_id = "odom"` ✅
  - `child_frame_id = "base_link"` ✅
- **Data:** Copied directly from ESP (no modification)

---

## Code Changes Summary

### Before (138 lines)
- 138 lines of code
- Complex integration logic
- Timer-based publishing
- Velocity integration
- Encoder processing

### After (95 lines)
- 95 lines of code (35% reduction)
- Simple subscription → callback → publish
- Event-driven (no timer)
- Zero computation
- Pure pass-through relay

---

## Testing Checklist

### 1. Verify ESP is Publishing
```bash
ros2 topic list | grep esp
# Should show: /esp/odom

ros2 topic echo /esp/odom --once
# Should show Odometry message with position/orientation
```

### 2. Verify odometry_node is Subscribed
```bash
ros2 topic info /esp/odom
# Should show odometry_node in Subscriptions list
```

### 3. Verify /odom is Published
```bash
ros2 topic info /odom
# Should show odometry_node as Publisher
# Should show ekf_filter_node in Subscriptions list

ros2 topic echo /odom --once
# Should show Odometry message with:
#   header.frame_id = "odom"
#   child_frame_id = "base_link"
```

### 4. Verify TF Chain
```bash
ros2 run tf2_ros tf2_echo odom base_link
# Should show continuous transform updates
```

### 5. Verify EKF is Processing
```bash
ros2 topic echo /odometry/filtered --once
# Should show filtered odometry from EKF
```

### 6. Verify SLAM Toolbox
```bash
# Check SLAM logs - should NOT see:
#   "Failed to compute odom pose"
# Should see:
#   "SLAM initialized"
#   "Publishing map updates"
```

---

## Configuration

### Default Configuration
- ESP topic: `/esp/odom`
- Output topic: `/odom`
- Frame IDs: `odom` → `base_link`

### Override ESP Topic (if needed)
```bash
ros2 run rover_navigation odometry_node \
  --ros-args -p esp_odom_topic:=/custom/esp/odom
```

Or in launch file:
```python
Node(
    package='rover_navigation',
    executable='odometry_node',
    parameters=[{
        'esp_odom_topic': '/custom/esp/odom'
    }]
)
```

---

## Expected Behavior

### ✅ Success Indicators
1. Node starts without errors
2. Logs: "Odometry Node Started (ESP Relay Mode)"
3. Logs: "ESP odometry relayed: X=... Y=... θ=..." (periodic)
4. `/odom` topic is publishing
5. EKF receives odometry and publishes TF
6. SLAM Toolbox can compute odom pose

### ❌ Failure Indicators
1. Node starts but no logs about ESP odometry
   - **Cause:** ESP not publishing to `/esp/odom`
   - **Fix:** Verify ESP is running and publishing

2. `/odom` topic exists but no messages
   - **Cause:** ESP not sending messages
   - **Fix:** Check ESP connection/status

3. TF `odom → base_link` not appearing
   - **Cause:** EKF not receiving `/odom` or not initialized
   - **Fix:** Check EKF configuration and `/odom` topic

---

## Migration Notes

### If ESP Topic Name is Different
If your ESP publishes to a different topic (e.g., `/esp8266/odom`), you can override:
```bash
ros2 param set /odometry_node esp_odom_topic /esp8266/odom
```

### If ESP Message Format is Different
If ESP publishes a different message type (e.g., `geometry_msgs/PoseStamped`), you'll need to:
1. Create a converter node, OR
2. Modify ESP to publish `nav_msgs/Odometry`

---

## Files Modified

- ✅ `src/rover_navigation/rover_navigation/odometry_node.py`
  - Complete rewrite (138 → 95 lines)
  - Removed all computation logic
  - Added ESP subscription and relay

---

## Files NOT Modified (as requested)

- ❌ `config/ekf.yaml` - No changes
- ❌ `config/slam_params.yaml` - No changes
- ❌ Frame names - Still `odom` and `base_link`
- ❌ TF tree structure - Unchanged

---

## Next Steps

1. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rover_navigation
   source install/setup.bash
   ```

2. **Test ESP connection:**
   ```bash
   ros2 topic echo /esp/odom --once
   ```

3. **Start odometry_node:**
   ```bash
   ros2 run rover_navigation odometry_node
   ```

4. **Verify data flow:**
   ```bash
   ros2 topic hz /odom
   # Should show ~same rate as /esp/odom
   ```

5. **Start EKF and verify TF:**
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

---

## Summary

The `odometry_node` is now a **pure relay** that:
- ✅ Subscribes to ESP odometry (`/esp/odom`)
- ✅ Republishes to `/odom` with correct frame IDs
- ✅ Does NOT compute anything internally
- ✅ Does NOT integrate velocity
- ✅ Does NOT process encoders
- ✅ Maintains compatibility with EKF and SLAM Toolbox

**Result:** ESP odometry flows directly through to EKF → TF → SLAM Toolbox without any intermediate computation or conflicts.

