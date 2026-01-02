# Navigation Implementation Guide

## Overview

The navigation system allows the Stretch 2 robot to autonomously move to anchor positions (A, B, C, D) defined in the world.

## How It Works

### Navigation Controller

The `NavigationController` class implements a simple proportional controller:

1. **Position Calculation**: Computes distance and direction to target
2. **Orientation Control**: First turns robot to face the target
3. **Forward Movement**: Moves forward once aligned
4. **Arrival Detection**: Stops when within tolerance (0.1m)

### Control Strategy

```
1. Calculate distance and angle to target
2. If angle error > threshold:
   → Turn towards target (angular velocity only)
3. If angle error < threshold:
   → Move forward + small angular correction
4. If distance < tolerance:
   → Stop and mark as reached
```

## Parameters

Located in `navigation.py`:

- `POSITION_TOLERANCE = 0.1` meters - Stop distance from target
- `ANGLE_TOLERANCE = 0.1` radians - Alignment threshold before moving
- `MAX_LINEAR_VEL = 1.0` m/s - Maximum forward speed
- `MAX_ANGULAR_VEL = 1.0` rad/s - Maximum turning speed
- `K_P_LINEAR = 2.0` - Proportional gain for linear velocity
- `K_P_ANGULAR = 2.0` - Proportional gain for angular velocity

## Usage

### Via Keyboard Controller

```bash
python stretch_keyboard_controller.py
# Press A, B, C, or D to navigate to respective anchor
```

### Via ROS 2 Topic

```bash
# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"

# Navigate to Anchor B
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'B'}"
```

### Check Navigation Status

```bash
# Check if navigation is active
ros2 topic echo /stretch/navigation_active
```

## Manual Override

Manual velocity commands automatically cancel navigation:

```bash
# This will cancel navigation and use manual control
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Anchor Positions

- **Anchor A**: [-0.65, 0.37, 0.00] - Green sphere (near table 1)
- **Anchor B**: [0.65, 0.37, 0.00] - Red sphere (near table 1)
- **Anchor C**: [-1.0, 5.42, 0.00] - Red sphere (near table 2)
- **Anchor D**: [1.0, 5.42, 0.00] - Red sphere (near table 2)

## Implementation Details

### Coordinate System

- **X-axis**: Forward/backward (positive = forward)
- **Y-axis**: Left/right (positive = left)
- **Z-axis**: Up/down (positive = up)
- **Yaw**: Rotation around Z-axis (0 = facing +X direction)

### Robot Pose

The robot's pose is extracted from MuJoCo's `qpos`:
- `qpos[0:3]`: Position [x, y, z]
- `qpos[3:7]`: Quaternion [w, x, y, z]

### Navigation Flow

1. User sends anchor command via ROS 2
2. Navigation controller sets target position
3. Each simulation step:
   - Get current robot pose
   - Calculate control velocities
   - Apply to robot actuators
4. When target reached, navigation stops

## Troubleshooting

### Robot doesn't move
- Check if navigation is active: `ros2 topic echo /stretch/navigation_active`
- Verify target was set: Check logs for "Navigating to anchor..."
- Check for manual override: Any cmd_vel commands cancel navigation

### Robot overshoots target
- Reduce `K_P_LINEAR` gain
- Increase `POSITION_TOLERANCE` for earlier stopping

### Robot oscillates
- Reduce `K_P_ANGULAR` gain
- Increase `ANGLE_TOLERANCE` for less strict alignment

### Robot turns in wrong direction
- Check quaternion format (should be [w, x, y, z])
- Verify yaw calculation in `navigation.py`

## Future Improvements

- [ ] Path planning around obstacles
- [ ] Speed control based on distance to target
- [ ] Smooth deceleration near target
- [ ] Waypoint following
- [ ] Obstacle avoidance
- [ ] Visual feedback in viewer

