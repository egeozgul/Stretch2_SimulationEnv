# ROS 2 Communication Testing Guide

This guide walks you through testing the ROS 2 communication setup for the Stretch 2 simulation.

## Prerequisites Check

First, verify ROS 2 is installed and accessible:

```bash
# Check if ROS 2 is sourced
echo $ROS_DISTRO

# If empty, source ROS 2 (adjust path for your version)
source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash

# Verify ROS 2 Python package
conda activate simenv
python -c "import rclpy; print('ROS 2 OK')"
```

## Quick Test Script

Run the automated test script:

```bash
conda activate simenv
source /opt/ros/humble/setup.bash
python test_ros2_communication.py
```

This will:
- ✓ Check ROS 2 imports
- ✓ Test topic creation
- ✓ Send test commands (optional)

## Manual Testing Steps

### Step 1: Start the Simulation Node

**Terminal 1:**
```bash
conda activate simenv
source /opt/ros/humble/setup.bash
python stretch_ros2_sim.py
```

You should see:
- MuJoCo viewer window opens
- Log messages showing the node started
- Messages like: "Stretch 2 ROS 2 Simulation Node started"

### Step 2: Verify Topics are Available

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash

# List all topics
ros2 topic list

# You should see:
# /stretch/cmd_vel
# /stretch/joint_commands
# /stretch/joint_states
# /stretch/navigate_to_anchor

# Check topic info
ros2 topic info /stretch/cmd_vel
ros2 topic info /stretch/joint_states
```

### Step 3: Test Base Velocity Commands

**Terminal 2 (while simulation is running):**

```bash
source /opt/ros/humble/setup.bash

# Move forward
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected result:** The robot base should move in the MuJoCo viewer.

### Step 4: Test Joint Commands

```bash
# Lift up
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Extend arm
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, 0.0, 0.0, 0.0, 0.0]}"

# Reset
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Expected result:** The robot's lift and arm should move in the viewer.

### Step 5: Test Anchor Navigation Commands

```bash
# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"

# Navigate to Anchor B
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'B'}"
```

**Expected result:** You should see log messages in Terminal 1 indicating the command was received. (Navigation logic not yet implemented, so robot won't move yet.)

### Step 6: Monitor Joint States

```bash
# Echo joint states (should update continuously)
ros2 topic echo /stretch/joint_states
```

**Expected result:** You should see joint state messages with positions, velocities, and efforts updating.

### Step 7: Test Keyboard Controller

**Terminal 2 (stop the echo command with Ctrl+C, then):**

```bash
conda activate simenv
source /opt/ros/humble/setup.bash
python stretch_keyboard_controller.py
```

Then press:
- **A, B, C, or D** - Should send anchor navigation commands
- **W/S** - Should move base forward/backward
- **ESC** - Exit

**Expected result:** Commands should be sent and visible in Terminal 1 logs.

## Continuous Testing

For continuous base movement testing:

```bash
# Move forward continuously (press Ctrl+C to stop)
ros2 topic pub -r 10 /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Troubleshooting

### Issue: "No module named 'rclpy'"
**Solution:**
```bash
conda activate simenv
pip install rclpy
source /opt/ros/humble/setup.bash
```

### Issue: Topics not appearing
**Solution:**
- Make sure the simulation node is running
- Check ROS 2 is sourced: `echo $ROS_DISTRO`
- Verify with: `ros2 node list` (should show `stretch_sim`)

### Issue: Commands not working
**Solution:**
- Check topic names match: `ros2 topic list`
- Verify message types: `ros2 topic info /stretch/cmd_vel`
- Check for errors in Terminal 1 (simulation node)

### Issue: "ImportError: cannot import name 'Twist'"
**Solution:**
- Source ROS 2 before running Python scripts
- Make sure ROS 2 message packages are installed (part of ROS 2 desktop installation)

## Expected Behavior Summary

| Test | Command | Expected Result |
|------|---------|----------------|
| Base velocity | `ros2 topic pub ... /stretch/cmd_vel` | Robot base moves |
| Joint commands | `ros2 topic pub ... /stretch/joint_commands` | Robot joints move |
| Anchor command | `ros2 topic pub ... /stretch/navigate_to_anchor` | Log message appears |
| Joint states | `ros2 topic echo /stretch/joint_states` | Continuous state updates |
| Keyboard controller | `python stretch_keyboard_controller.py` | Commands sent on keypress |

## Next Steps

Once communication is verified:
1. Implement navigation logic to move robot to anchors
2. Add path planning
3. Add obstacle avoidance
4. Integrate with higher-level planning systems

