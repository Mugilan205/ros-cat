# FastDDS SHM Disable Fix for EKF Node Discovery

## Problem

The `ekf_node` process runs but does not appear in `ros2 node list`. This is caused by FastDDS shared-memory (SHM) transport discovery failure in ROS 2 Jazzy.

## Solution

Disable FastDDS SHM transport and force UDP-only communication.

## Files Created

1. **FastDDS XML Profile**: `/tmp/fastrtps_disable_shm.xml`
   - Disables builtin transports (including SHM)
   - Enables UDPv4 transport only
   - Sets as default participant profile

## Exact Commands

### Step 1: Export Environment Variables

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml
export ROS_DOMAIN_ID=0
```

### Step 2: Source ROS 2 Environment

```bash
source /opt/ros/jazzy/setup.bash
source /home/caterpillar/ros2_ws/install/setup.bash
```

### Step 3: Kill Existing ekf_node (if running)

```bash
pkill -9 -f ekf_node
```

### Step 4: Start ekf_node with Correct Parameters

```bash
ros2 run robot_localization ekf_node --ros-args --params-file /home/caterpillar/ros2_ws/config/ekf.yaml
```

## Verification Checklist

Run these commands to verify the fix:

### 1. Check Node Visibility
```bash
ros2 node list
```
**Expected**: Should show `/ekf_filter_node`

### 2. Check Topic Visibility
```bash
ros2 topic list
```
**Expected**: Should show `/odometry/filtered`

### 3. Check TF Transform
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
**Expected**: Should show transform data

### 4. Check Node Info
```bash
ros2 node info /ekf_filter_node
```
**Expected**: Should show node details, publishers, subscribers

## Quick Start (Automated Script)

Run the provided script:
```bash
cd /home/caterpillar/ros2_ws
./fix_ekf_dds_discovery.sh
```

## Why FastDDS SHM Causes Invisible Nodes

FastDDS uses shared-memory (SHM) transport by default for inter-process communication on the same machine. However, in some configurations (particularly with ROS 2 Jazzy), the SHM transport can fail to properly register nodes in the ROS graph discovery mechanism. This results in:

1. **Process runs**: The executable starts successfully
2. **DDS participant exists**: The FastDDS participant is created
3. **ROS graph invisible**: The node doesn't appear in `ros2 node list` because discovery fails
4. **No communication**: Other nodes cannot discover or communicate with it

By disabling SHM and using UDP transport only:
- All communication goes through the network stack
- Discovery uses standard DDS discovery protocols over UDP
- Nodes become visible in the ROS graph
- Inter-process communication works reliably

## Persistent Configuration

To make this permanent, add the export commands to your `~/.bashrc`:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_disable_shm.xml' >> ~/.bashrc
```

## XML Profile Location

The XML profile is stored at `/tmp/fastrtps_disable_shm.xml`. This location is used because:
- `/tmp` is universally accessible
- FastDDS reads from this path when `FASTRTPS_DEFAULT_PROFILES_FILE` is set
- The file persists across reboots (unless `/tmp` is cleared)

If you need to move it, update `FASTRTPS_DEFAULT_PROFILES_FILE` accordingly.

