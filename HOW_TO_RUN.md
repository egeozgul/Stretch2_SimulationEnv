# How to Run ROS 2 Communication

## Quick Start

### Step 1: Start the Simulation Node

Open **Terminal 1** and run:

```bash
# Activate ROS 2 environment
conda activate simenv_ros2

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Run the simulation
python stretch_ros2_sim.py
```

**Or use the wrapper script:**
```bash
./run_ros2_sim.sh
```

You should see:
- MuJoCo viewer window opens
- Log messages: "Stretch 2 ROS 2 Simulation Node started"
- The robot in the simulation

### Step 2: Send Commands (Choose one method)

#### Option A: Keyboard Controller (Recommended)

Open **Terminal 2** and run:

```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_keyboard_controller.py
```

Then press:
- **A, B, C, or D** - Navigate to anchors
- **W/S** - Move forward/backward
- **ESC** - Exit

#### Option B: ROS 2 Command Line

Open **Terminal 2** and run:

```bash
source /opt/ros/jazzy/setup.bash

# Move forward
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# Turn left
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Send joint command (lift up)
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Navigate to Anchor A
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"
```

#### Option C: Monitor Topics

Open **Terminal 2** and run:

```bash
source /opt/ros/jazzy/setup.bash

# List all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /stretch/joint_states

# Check topic info
ros2 topic info /stretch/cmd_vel
```

## Complete Example Session

**Terminal 1 (Simulation):**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

**Terminal 2 (Controller):**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_keyboard_controller.py
# Then press A, B, C, or D keys
```

## Troubleshooting

### "No module named 'rclpy'"
Make sure you:
1. Activated the correct environment: `conda activate simenv_ros2`
2. Sourced ROS 2: `source /opt/ros/jazzy/setup.bash`

### "Topics not found"
Make sure the simulation node is running in Terminal 1 first!

### "MuJoCo viewer doesn't open"
This is normal if running over SSH or without display. The simulation still runs, you just can't see it.

## All Available Scripts

| Script | Purpose | Environment |
|--------|---------|-------------|
| `stretch_ros2_sim.py` | Main simulation node | simenv_ros2 |
| `stretch_keyboard_controller.py` | Keyboard input → ROS 2 | simenv_ros2 |
| `stretch_ros2_controller.py` | Programmatic controller | simenv_ros2 |
| `teleop.py` | Direct keyboard control | simenv |
| `view_world.py` | View world only | simenv |

## Quick Reference

**Start simulation:**
```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash && python stretch_ros2_sim.py
```

**Use keyboard controller:**
```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash && python stretch_keyboard_controller.py
```

**Send command via CLI:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

