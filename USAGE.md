# Usage Guide

Complete guide for using the Stretch 2 Simulation Environment.

## Quick Start

### Regular Simulation (No ROS 2)

**Teleoperation (Interactive Control)**
```bash
conda activate simenv
python teleop.py
```

**View World**
```bash
conda activate simenv
python view_world.py
```

**Check Mesh Properties**
```bash
conda activate simenv
python checkmesh.py
```

### ROS 2 Simulation

**Start the Simulation Node**
```bash
# Option 1: Use wrapper script
./run_ros2_sim.sh

# Option 2: Manual
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

**Use Keyboard Controller**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_keyboard_controller.py
```

## Keyboard Controls

### Base Movement
- **W/S** - Move forward/backward
- **A/D** - Turn left/right
- **1/2/3/4** - Navigate to anchors (A, B, C, D)

### Arm & Gripper
- **Q/E** - Lift up/down
- **R/F** - Arm extend/retract
- **T/G** - Wrist yaw rotate
- **V/B** - Wrist roll rotate
- **Z/X** - Gripper open/close

### Camera/Head
- **Arrow Keys** - Pan/tilt head

### General
- **ESC** - Exit

## ROS 2 Communication

### Running ROS 2 Simulation

**Terminal 1: Start the Simulation Node**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

This will:
- Load the MuJoCo simulation
- Open the MuJoCo viewer
- Start listening for ROS 2 commands
- Publish joint states and camera feed

**Terminal 2: Send Commands**

**Option A: Keyboard Controller (Recommended)**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_keyboard_controller.py
```

**Option B: ROS 2 Command Line**
```bash
source /opt/ros/jazzy/setup.bash

# Move forward
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# Turn left
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Send joint command (lift up)
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"
```

**Option C: Monitor Topics**
```bash
source /opt/ros/jazzy/setup.bash

# List all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /stretch/joint_states

# Check topic info
ros2 topic info /stretch/cmd_vel
```

## Navigation System

### Overview

The navigation system allows the Stretch 2 robot to autonomously move to anchor positions (A, B, C, D) defined in the world.

### How It Works

The `NavigationController` class implements a proportional controller:

1. **Position Calculation**: Computes distance and direction to target
2. **Orientation Control**: First turns robot to face the target
3. **Forward Movement**: Moves forward once aligned
4. **Direction Alignment**: Rotates to target direction if specified
5. **Arrival Detection**: Stops when within tolerance (0.15m)

### Control Strategy

```
1. Calculate distance and angle to target
2. If angle error > 45°:
   → Turn towards target (angular velocity only)
3. If angle error < 45°:
   → Move forward + small angular correction
4. If distance < tolerance:
   → Stop and rotate to target direction (if specified)
5. If direction aligned:
   → Mark as reached
```

### Parameters

Located in `navigation.py`:
- `POSITION_TOLERANCE = 0.15` meters - Stop distance from target
- `ALIGNMENT_THRESHOLD = 15°` - Alignment threshold before moving
- `MAX_ANGLE_FOR_MOVEMENT = 45°` - Maximum angle error to allow forward movement
- `DIRECTION_TOLERANCE = 5°` - Tolerance for final direction alignment
- `MAX_LINEAR_VEL = 2.5` m/s - Maximum forward speed
- `MAX_ANGULAR_VEL = 2.0` rad/s - Maximum turning speed
- `K_P_ANGULAR = 2.0` - Proportional gain for angular velocity

### Usage

**Via Keyboard Controller**
```bash
python stretch_keyboard_controller.py
# Press 1, 2, 3, or 4 to navigate to respective anchor
```

**Via ROS 2 Topic**
```bash
# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"

# Navigate to Anchor B
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'B'}"
```

**Check Navigation Status**
```bash
# Check if navigation is active
ros2 topic echo /stretch/navigation_active
```

### Manual Override

Manual velocity commands automatically cancel navigation:
```bash
# This will cancel navigation and use manual control
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### Anchor Positions

Anchors are defined in `table_world.xml`:
- **Anchor A**: Position (-0.65, 1.0, 0.0) - Green sphere
- **Anchor B**: Position (0.65, 1.0, 0.0) - Red sphere  
- **Anchor C**: Position (-1.0, 5.0, 0.0) - Red sphere
- **Anchor D**: Position (1.0, 5.0, 0.0) - Red sphere

Each anchor can have an optional `direction` attribute (in degrees) that the robot will align to after reaching the position.

### Coordinate System

- **X-axis**: Forward/backward (positive = forward)
- **Y-axis**: Left/right (positive = left)
- **Z-axis**: Up/down (positive = up)
- **Yaw**: Rotation around Z-axis (0 = facing +X direction)

## Testing

### Quick Test Script

Run the automated test script:
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python test_ros2_communication.py
```

This will:
- ✓ Check ROS 2 imports
- ✓ Test topic creation
- ✓ Send test commands (optional)

### Manual Testing Steps

**1. Start the Simulation Node**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

**2. Verify Topics are Available**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

**3. Test Base Velocity Commands**
```bash
# Move forward
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# Turn left
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**4. Test Joint Commands**
```bash
# Lift up
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Extend arm
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**5. Test Anchor Navigation**
```bash
# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"
```

**6. Monitor Joint States**
```bash
ros2 topic echo /stretch/joint_states
```

### Expected Behavior

| Test | Command | Expected Result |
|------|---------|----------------|
| Base velocity | `ros2 topic pub ... /stretch/cmd_vel` | Robot base moves |
| Joint commands | `ros2 topic pub ... /stretch/joint_commands` | Robot joints move |
| Anchor command | `ros2 topic pub ... /stretch/navigate_to_anchor` | Robot navigates to anchor |
| Joint states | `ros2 topic echo /stretch/joint_states` | Continuous state updates |
| Keyboard controller | `python stretch_keyboard_controller.py` | Commands sent on keypress |

## Troubleshooting

### Robot doesn't move during navigation
- Check if navigation is active: `ros2 topic echo /stretch/navigation_active`
- Verify target was set: Check logs for "Navigating to anchor..."
- Check for manual override: Any cmd_vel commands cancel navigation

### Robot overshoots target
- Reduce `K_P_LINEAR` gain in `navigation.py`
- Increase `POSITION_TOLERANCE` for earlier stopping

### Robot oscillates
- Reduce `K_P_ANGULAR` gain in `navigation.py`
- Increase `ALIGNMENT_THRESHOLD` for less strict alignment

### Camera window shows black
- Ensure the camera is properly initialized
- Check that the robot's head is positioned correctly
- Try moving the head with arrow keys

### Topics not appearing
- Make sure the simulation node is running
- Check ROS 2 is sourced: `echo $ROS_DISTRO`
- Verify with: `ros2 node list` (should show `stretch_sim`)

## Available Scripts

| Script | Purpose | Environment |
|--------|---------|-------------|
| `stretch_ros2_sim.py` | Main simulation node | simenv_ros2 |
| `stretch_keyboard_controller.py` | Keyboard input → ROS 2 | simenv_ros2 |
| `teleop.py` | Direct keyboard control | simenv |
| `view_world.py` | View world only | simenv |
| `test_ros2_communication.py` | Test ROS 2 setup | simenv_ros2 |

