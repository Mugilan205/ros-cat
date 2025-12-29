# Quick SLAM Parameter Verification Commands

## Immediate Diagnostic (Run These Now)

### 1. Find SLAM Node Name
```bash
ros2 node list | grep -i slam
```
**Record the output** (e.g., `/slam_toolbox` or `/async_slam_toolbox_node`)

---

### 2. Check if Parameters Were Loaded
```bash
# Replace /slam_toolbox with your actual node name from step 1
ros2 param list /slam_toolbox
```

**Look for:**
- `odom_frame`
- `base_frame`
- `map_frame`
- `scan_topic`

**If these are MISSING:** ✅ **Cause B CONFIRMED** - Parameters not loaded

---

### 3. Get Parameter Values
```bash
ros2 param get /slam_toolbox odom_frame
ros2 param get /slam_toolbox base_frame
ros2 param get /slam_toolbox scan_topic
```

**Expected:**
- `odom_frame`: `odom`
- `base_frame`: `base_link`
- `scan_topic`: `/scan`

**If values DON'T MATCH:** ✅ **Cause B CONFIRMED** - Wrong values

---

### 4. Check Config File Path Issue
```bash
# Check if the file in your launch command exists
ls -la /home/caterpillar/ros2_ws/config/slam_toolbox.yaml

# Check if the actual config file exists
ls -la /home/caterpillar/ros2_ws/config/slam_params.yaml
```

**If `slam_toolbox.yaml` doesn't exist but `slam_params.yaml` does:**
- ✅ **Cause B CONFIRMED** - Wrong file path in launch command
- **Fix:** Change launch command to use `slam_params_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml`

---

### 5. Verify TF Lookup
```bash
ros2 run tf2_ros tf2_echo odom base_link --once
```

**Should show transform data.** If it fails, TF chain is broken.

---

### 6. Check Topic Subscription
```bash
ros2 topic info /scan
```

**Should show your SLAM node in "Subscriptions" section.**

---

## All-in-One Command

Run this to get all diagnostic info at once:

```bash
NODE=$(ros2 node list | grep -i slam | head -1) && \
echo "Node: $NODE" && \
echo "" && \
echo "=== Parameters ===" && \
ros2 param list $NODE | grep -E "(odom_frame|base_frame|scan_topic)" && \
echo "" && \
echo "=== Parameter Values ===" && \
ros2 param get $NODE odom_frame && \
ros2 param get $NODE base_frame && \
ros2 param get $NODE scan_topic && \
echo "" && \
echo "=== Config Files ===" && \
ls -la /home/caterpillar/ros2_ws/config/slam*.yaml && \
echo "" && \
echo "=== TF Check ===" && \
timeout 2 ros2 run tf2_ros tf2_echo odom base_link --once 2>&1 | head -5
```

---

## Most Likely Issue

Based on your launch command:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/caterpillar/ros2_ws/config/slam_toolbox.yaml
```

**The file `/home/caterpillar/ros2_ws/config/slam_toolbox.yaml` probably doesn't exist.**

**Your actual file is:** `/home/caterpillar/ros2_ws/config/slam_params.yaml`

**Fix:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml
```

This would explain:
- "Parameter file path is not a file" warning
- Parameters not loading
- "Failed to compute odom pose" (using wrong defaults)

