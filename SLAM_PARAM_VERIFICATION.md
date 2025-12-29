# SLAM Toolbox Parameter/Frame/Topic Verification Procedure

## Objective
Verify whether "Failed to compute odom pose" is caused by parameter/frame/topic mismatch (Cause B) on a STATIC robot.

---

## STEP 1: Identify SLAM Toolbox Node Name

**Command:**
```bash
ros2 node list | grep -i slam
```

**Expected Output:**
- Should show something like `/slam_toolbox` or `/async_slam_toolbox_node`

**Note:** Record the exact node name (e.g., `/slam_toolbox`) for use in subsequent commands.

**If no node found:**
- SLAM Toolbox is not running
- Start it first using your launch command

---

## STEP 2: Verify Parameter File Was Loaded

### 2a. List ALL Parameters

**Command:**
```bash
ros2 param list /slam_toolbox
```
*(Replace `/slam_toolbox` with actual node name from Step 1)*

**Expected Parameters:**
- `odom_frame` (should exist)
- `base_frame` (should exist)
- `map_frame` (should exist)
- `scan_topic` (should exist)
- `mode` (should exist)

**If parameters are MISSING:**
- ✅ **Cause B CONFIRMED:** Parameter file was NOT loaded
- The node is using default values (which may be wrong)

**If parameters EXIST:**
- Continue to Step 2b to verify VALUES

### 2b. Get Critical Parameter Values

**Commands:**
```bash
ros2 param get /slam_toolbox odom_frame
ros2 param get /slam_toolbox base_frame
ros2 param get /slam_toolbox map_frame
ros2 param get /slam_toolbox scan_topic
ros2 param get /slam_toolbox mode
```

**Expected Values (from your config):**
- `odom_frame`: `odom`
- `base_frame`: `base_link`
- `map_frame`: `map`
- `scan_topic`: `/scan`
- `mode`: `mapping`

**If values DON'T MATCH:**
- ✅ **Cause B CONFIRMED:** Wrong parameter values loaded
- Check if parameter file path was correct in launch command

**If values MATCH:**
- Continue to Step 3

---

## STEP 3: Verify TF Frame Lookup

SLAM Toolbox performs this TF lookup to compute odom pose:
```
odom_frame → base_frame
```

In your case, this should be:
```
odom → base_link
```

### 3a. Test TF Lookup Directly

**Command:**
```bash
ros2 run tf2_ros tf2_echo odom base_link
```

**Expected:**
- Should print transform data continuously
- No "Could not find transform" errors
- Transform should be STATIC (since robot is not moving)

**If TF lookup FAILS:**
- ✅ **Cause B CONFIRMED:** TF chain is broken
- Even if `odom → base_link` exists, slam_toolbox may be looking for wrong frames

**If TF lookup SUCCEEDS:**
- Continue to Step 3b

### 3b. Verify TF Lookup with Exact Parameter Values

**Critical Test:** Use the EXACT frame names from slam_toolbox parameters:

**Commands:**
```bash
# Get the actual parameter values
ODOM_FRAME=$(ros2 param get /slam_toolbox odom_frame 2>/dev/null | grep -oP 'String value is: \K.*' || echo "odom")
BASE_FRAME=$(ros2 param get /slam_toolbox base_frame 2>/dev/null | grep -oP 'String value is: \K.*' || echo "base_link")

echo "Testing TF lookup: $ODOM_FRAME → $BASE_FRAME"
ros2 run tf2_ros tf2_echo $ODOM_FRAME $BASE_FRAME
```

**Expected:**
- Should work if parameters match actual TF frames

**If this FAILS:**
- ✅ **Cause B CONFIRMED:** Parameter values don't match actual TF frames
- Example: Parameter says `odom_frame: odometry` but TF uses `odom`

---

## STEP 4: Verify Topic Subscription

### 4a. Check if SLAM Toolbox is Subscribed to /scan

**Command:**
```bash
ros2 topic info /scan
```

**Expected Output:**
- Should show `/slam_toolbox` (or your node name) in the "Subscription count" section
- Should list `sensor_msgs/msg/LaserScan` as message type

**If slam_toolbox is NOT subscribed:**
- ✅ **Cause B CONFIRMED:** Topic name mismatch
- slam_toolbox is looking for wrong topic name

**If subscribed:**
- Continue to Step 4b

### 4b. Verify Topic Name Parameter Matches Actual Topic

**Command:**
```bash
ros2 param get /slam_toolbox scan_topic
```

**Expected:**
- Should return `/scan`

**Then verify topic exists:**
```bash
ros2 topic list | grep scan
```

**Expected:**
- Should show `/scan`

**If parameter doesn't match actual topic:**
- ✅ **Cause B CONFIRMED:** Topic name mismatch
- Example: Parameter says `scan_topic: scan` but actual topic is `/scan`

---

## STEP 5: Test Static Robot TF Lookup

**Question:** Can slam_toolbox compute odom pose on a STATIC robot if TF is valid?

**Answer:** YES. slam_toolbox only needs:
1. Valid TF transform: `odom → base_link`
2. Valid scan messages

Motion is NOT required. The "odom pose" is just the current pose from TF, which can be static.

**Test:**
```bash
# Verify TF exists and is static
ros2 run tf2_ros tf2_echo odom base_link --once
```

**Expected:**
- Should return ONE transform (even if robot is static)
- Transform values should be constant (x=0, y=0, z=0, rotation=identity if at origin)

**If this works but slam_toolbox still fails:**
- ✅ **Cause B CONFIRMED:** slam_toolbox is using wrong frame names internally

---

## STEP 6: Check Launch Command Parameter File Path

**Your launch command:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/caterpillar/ros2_ws/config/slam_toolbox.yaml
```

**Check if file exists:**
```bash
ls -la /home/caterpillar/ros2_ws/config/slam_toolbox.yaml
```

**If file DOES NOT EXIST:**
- ✅ **Cause B CONFIRMED:** Wrong file path
- Parameter file was never loaded
- Node is using defaults (which may be wrong)

**Actual file location (from your workspace):**
```bash
ls -la /home/caterpillar/ros2_ws/config/slam_params.yaml
```

**Fix:** Use correct file path:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml
```

---

## STEP 7: Dump All Parameters for Comparison

**Command:**
```bash
ros2 param dump /slam_toolbox > /tmp/slam_actual_params.yaml
cat /tmp/slam_actual_params.yaml
```

**Compare with expected config:**
```bash
cat /home/caterpillar/ros2_ws/config/slam_params.yaml
```

**If parameters don't match:**
- ✅ **Cause B CONFIRMED:** Parameters not loaded correctly

---

## CONCLUSION PROCEDURE

Run these commands in sequence and record results:

```bash
# 1. Node name
NODE_NAME=$(ros2 node list | grep -i slam | head -1)
echo "SLAM node: $NODE_NAME"

# 2. Check if parameters exist
echo "=== Parameter Check ==="
ros2 param list $NODE_NAME | grep -E "(odom_frame|base_frame|map_frame|scan_topic)" || echo "MISSING PARAMETERS"

# 3. Get parameter values
echo "=== Parameter Values ==="
ros2 param get $NODE_NAME odom_frame 2>&1
ros2 param get $NODE_NAME base_frame 2>&1
ros2 param get $NODE_NAME scan_topic 2>&1

# 4. Check TF
echo "=== TF Check ==="
ros2 run tf2_ros tf2_echo odom base_link --once 2>&1 | head -10

# 5. Check topic subscription
echo "=== Topic Subscription ==="
ros2 topic info /scan | grep -A 5 "Subscriptions"

# 6. Check parameter file path
echo "=== Config File Check ==="
ls -la /home/caterpillar/ros2_ws/config/slam_toolbox.yaml 2>&1
ls -la /home/caterpillar/ros2_ws/config/slam_params.yaml 2>&1
```

---

## FINAL CONCLUSION

**Cause B CONFIRMED if ANY of these are true:**
1. ❌ Parameters missing from `ros2 param list`
2. ❌ Parameter values don't match expected (odom, base_link, /scan)
3. ❌ TF lookup fails with parameter frame names
4. ❌ Topic subscription missing or wrong topic name
5. ❌ Parameter file path in launch command is wrong/non-existent

**Cause B RULED OUT if ALL of these are true:**
1. ✅ All parameters exist and have correct values
2. ✅ TF lookup works with parameter frame names
3. ✅ Topic subscription exists and matches parameter
4. ✅ Parameter file path is correct and file exists

---

## QUICK DIAGNOSTIC SCRIPT

Save this as `verify_slam_params.sh`:

```bash
#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

echo "=========================================="
echo "SLAM Toolbox Parameter Verification"
echo "=========================================="
echo ""

# Find node
NODE_NAME=$(ros2 node list | grep -i slam | head -1)
if [ -z "$NODE_NAME" ]; then
    echo "❌ ERROR: SLAM Toolbox node not found"
    echo "   Run: ros2 node list"
    exit 1
fi

echo "✓ Found node: $NODE_NAME"
echo ""

# Check parameters
echo "=== STEP 1: Parameter Existence ==="
MISSING_PARAMS=0
for param in odom_frame base_frame map_frame scan_topic; do
    if ros2 param list $NODE_NAME | grep -q "$param"; then
        echo "✓ $param exists"
    else
        echo "❌ $param MISSING"
        MISSING_PARAMS=1
    fi
done
echo ""

if [ $MISSING_PARAMS -eq 1 ]; then
    echo "⚠️  CAUSE B CONFIRMED: Parameters missing"
    exit 1
fi

# Get parameter values
echo "=== STEP 2: Parameter Values ==="
ODOM_FRAME=$(ros2 param get $NODE_NAME odom_frame 2>/dev/null | grep -oP 'String value is: \K.*' || echo "")
BASE_FRAME=$(ros2 param get $NODE_NAME base_frame 2>/dev/null | grep -oP 'String value is: \K.*' || echo "")
SCAN_TOPIC=$(ros2 param get $NODE_NAME scan_topic 2>/dev/null | grep -oP 'String value is: \K.*' || echo "")

echo "odom_frame: $ODOM_FRAME"
echo "base_frame: $BASE_FRAME"
echo "scan_topic: $SCAN_TOPIC"
echo ""

# Verify values match expected
if [ "$ODOM_FRAME" != "odom" ] || [ "$BASE_FRAME" != "base_link" ] || [ "$SCAN_TOPIC" != "/scan" ]; then
    echo "⚠️  CAUSE B CONFIRMED: Parameter values don't match expected"
    echo "   Expected: odom_frame=odom, base_frame=base_link, scan_topic=/scan"
    exit 1
fi

# Check TF
echo "=== STEP 3: TF Lookup ==="
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link --once 2>&1 | grep -q "Translation"; then
    echo "✓ TF lookup odom → base_link works"
else
    echo "❌ TF lookup odom → base_link FAILED"
    echo "⚠️  CAUSE B CONFIRMED: TF lookup fails"
    exit 1
fi
echo ""

# Check topic subscription
echo "=== STEP 4: Topic Subscription ==="
if ros2 topic info /scan 2>/dev/null | grep -q "$NODE_NAME"; then
    echo "✓ $NODE_NAME is subscribed to /scan"
else
    echo "❌ $NODE_NAME is NOT subscribed to /scan"
    echo "⚠️  CAUSE B CONFIRMED: Topic subscription mismatch"
    exit 1
fi
echo ""

# Check config file
echo "=== STEP 5: Config File Path ==="
if [ -f "/home/caterpillar/ros2_ws/config/slam_toolbox.yaml" ]; then
    echo "✓ Config file exists: /home/caterpillar/ros2_ws/config/slam_toolbox.yaml"
elif [ -f "/home/caterpillar/ros2_ws/config/slam_params.yaml" ]; then
    echo "⚠️  Config file mismatch:"
    echo "   Launch uses: /home/caterpillar/ros2_ws/config/slam_toolbox.yaml"
    echo "   Actual file: /home/caterpillar/ros2_ws/config/slam_params.yaml"
    echo "⚠️  CAUSE B CONFIRMED: Wrong config file path"
    exit 1
else
    echo "❌ Config file not found"
    exit 1
fi
echo ""

echo "=========================================="
echo "✓ CAUSE B RULED OUT: Config is correct"
echo "=========================================="
echo ""
echo "All parameters loaded correctly."
echo "Frames match TF tree."
echo "Topic subscription is correct."
echo ""
echo "If 'Failed to compute odom pose' persists,"
echo "the issue is NOT a parameter/frame/topic mismatch."
```

Make it executable:
```bash
chmod +x verify_slam_params.sh
./verify_slam_params.sh
```

