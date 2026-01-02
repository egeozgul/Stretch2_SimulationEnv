# ROS 2 Setup Guide for Stretch 2 Simulation

This guide explains how to set up ROS 2 communication for the Stretch 2 MuJoCo simulation.

## Prerequisites

1. **Install ROS 2** (Humble or Jazzy recommended)
   - Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html
   - For Ubuntu 22.04: ROS 2 Humble
   - For Ubuntu 24.04: ROS 2 Jazzy

2. **Activate your conda environment**
   ```bash
   conda activate simenv
   ```

3. **Install ROS 2 Python packages**
   ```bash
   pip install rclpy
   ```

   Note: The ROS 2 message packages (`geometry_msgs`, `sensor_msgs`, `std_msgs`) should be available if ROS 2 is properly installed. If not, you may need to source the ROS 2 setup script.

## ROS 2 Topics

The simulation node publishes and subscribes to the following topics:

### Subscribed Topics (Commands)
- `/stretch/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
  - `linear.x`: Forward/backward velocity
  - `angular.z`: Rotational velocity

- `/stretch/joint_commands` (std_msgs/Float64MultiArray) - Joint position commands
  - Array format: `[lift, arm_extend, wrist_yaw, gripper, head_pan, head_tilt]`

- `/stretch/navigate_to_anchor` (std_msgs/String) - Navigate to anchor command
  - Values: "A", "B", "C", or "D"

### Published Topics (State)
- `/stretch/joint_states` (sensor_msgs/JointState) - Current joint positions and velocities

## Running the ROS 2 Simulation

### Terminal 1: Start the Simulation Node
```bash
conda activate simenv
source /opt/ros/humble/setup.bash  # Adjust path for your ROS 2 version
python stretch_ros2_sim.py
```

This will:
- Load the MuJoCo simulation
- Open the MuJoCo viewer
- Start listening for ROS 2 commands
- Publish joint states

### Terminal 2: Send Commands (Optional - for testing)

**Using the keyboard controller:**
```bash
conda activate simenv
source /opt/ros/humble/setup.bash
python stretch_keyboard_controller.py
```

Then press:
- **A, B, C, or D** - Navigate to respective anchor
- **W/S** - Move forward/backward
- **ESC** - Exit

**Using ROS 2 command line:**
```bash
source /opt/ros/humble/setup.bash

# Move forward
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# Turn
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Send joint commands
ros2 topic pub /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.2, 0.0, 0.0, 0.0, 0.0]}"
```

**View published data:**
```bash
# List all topics
ros2 topic list

# Echo joint states
ros2 topic echo /stretch/joint_states

# Check topic info
ros2 topic info /stretch/cmd_vel
```

## Anchor Positions

The anchors are defined in `table_world.xml`:
- **Anchor A**: Position (-0.65, 0.37, 0.00) - Green sphere
- **Anchor B**: Position (0.65, 0.37, 0.00) - Red sphere  
- **Anchor C**: Position (-1.0, 5.42, 0.00) - Red sphere
- **Anchor D**: Position (1.0, 5.42, 0.00) - Red sphere

## Next Steps

The navigation logic to move the robot to these anchors is not yet implemented. You'll need to:
1. Subscribe to `/stretch/navigate_to_anchor` in the simulation node
2. Implement path planning or simple navigation logic
3. Send appropriate `cmd_vel` commands to move the robot to the target position

## Troubleshooting

**Issue: "No module named 'rclpy'"**
- Solution: Make sure ROS 2 is installed and you've sourced the setup script
- Try: `source /opt/ros/humble/setup.bash` (adjust path for your version)

**Issue: "ImportError: cannot import name 'Twist'"**
- Solution: Source ROS 2 setup script before running Python scripts
- The message packages are part of the ROS 2 installation

**Issue: Topics not appearing**
- Solution: Make sure both nodes are running and ROS 2 is properly sourced
- Check with: `ros2 topic list`

