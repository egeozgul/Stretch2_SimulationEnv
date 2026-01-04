# Stretch2_SimulationEnv

A lightweight simulation environment for the **Hello Robot Stretch 2** platform using **MuJoCo**, **Python**, and optionally **ROS 2**.

## Features

- ✅ MuJoCo-based physics simulation
- ✅ Interactive keyboard control
- ✅ ROS 2 communication (optional)
- ✅ Autonomous navigation to anchor points
- ✅ Real-time camera feed
- ✅ Joint and base control
- ✅ Custom environments with tables and objects

## Quick Start

### Regular Simulation (No ROS 2)

```bash
# Create and activate environment
conda env create -f environment.yml
conda activate simenv

# Run interactive simulation
python teleop.py
```

### ROS 2 Simulation

```bash
# Create ROS 2 environment
conda env create -f environment_ros2.yml
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

# Start simulation
./run_ros2_sim.sh

# In another terminal, use keyboard controller
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_keyboard_controller.py
```

## Keyboard Controls

- **W/S** - Move forward/backward
- **A/D** - Turn left/right
- **1/2/3/4** - Navigate to anchors (A, B, C, D)
- **Q/E** - Lift up/down
- **R/F** - Arm extend/retract
- **T/G** - Wrist yaw rotate
- **V/B** - Wrist roll rotate
- **Z/X** - Gripper open/close
- **Arrow Keys** - Pan/tilt head
- **ESC** - Exit

## Documentation

- **[SETUP.md](SETUP.md)** - Complete setup instructions (environments, ROS 2, Python version)
- **[USAGE.md](USAGE.md)** - Usage guide (running, navigation, testing, troubleshooting)
- **[TEST_RESULTS.md](TEST_RESULTS.md)** - Test results and verification

## Project Structure

```
Stretch2_SimulationEnv-main/
├── stretch.xml              # Robot model
├── table_world.xml          # World with anchors
├── stretch_ros2_sim.py     # Main ROS 2 simulation node
├── stretch_keyboard_controller.py  # Keyboard controller
├── navigation.py            # Navigation controller
├── teleop.py               # Direct keyboard control
├── environment.yml         # Conda environment (Python 3.11)
├── environment_ros2.yml    # Conda environment (Python 3.12, ROS 2)
└── assets/                 # 3D models and textures
```

## ROS 2 Topics

**Subscribed:**
- `/stretch/cmd_vel` - Base velocity commands
- `/stretch/joint_commands` - Joint position commands
- `/stretch/navigate_to_anchor` - Navigate to anchor command

**Published:**
- `/stretch/joint_states` - Current joint states
- `/stretch/navigation_active` - Navigation status
- `/stretch/camera/image_raw` - Camera feed

## Goals

Build a modular simulation environment for the **Stretch 2 robot**, supporting:

* Custom MuJoCo-based environments
* Robot joint and wheel actuation
* Interactive scenes with tables, objects, and ingredients
* ROS 2 control and visualization
* Autonomous navigation

## Resources

* [Stretch 2 ROS 2 Description](https://docs.hello-robot.com/0.2/stretch-ros2/stretch_description/)
* [Stretch 2 STEP/URDF Files](https://github.com/hello-robot/stretch_tool_share/tree/master/tool_share/stretch_2_STEP)
* [Stretch Tool Share Repository](https://github.com/hello-robot/stretch_tool_share/)

## License

[Add your license here]
