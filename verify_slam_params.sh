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

