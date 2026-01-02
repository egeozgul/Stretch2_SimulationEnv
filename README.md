# Stretch2_SimulationEnv

A lightweight simulation environment for the **Hello Robot Stretch 2** platform.
This project uses **MuJoCo**, **Python**, and optionally **ROS 2** to simulate robot motion, interactions, and custom environments.

---

## 🔧 Setup Instructions

### Prerequisites

- Conda (Miniconda or Anaconda)
- Python 3.11

### Installation

**1. Clone the repository**

```bash
git clone <repository-url>
cd Stretch2_SimulationEnv-main
```

**2. Create the conda environment (first time)**

```bash
conda env create -f environment.yml
```

**3. Update the environment (after pulling updates from GitHub)**

```bash
conda env update -f environment.yml --prune
```

**4. Activate the environment**

```bash
conda activate simenv
```

**5. Verify the setup**

```bash
python verify_setup.py
```

This script will check:
- Conda environment activation
- Required Python packages
- Required files and directories
- MuJoCo model loading

### Running the Simulation

**Teleoperation (Interactive Control)**

```bash
conda activate simenv
python teleop.py
```

This will launch an interactive MuJoCo viewer where you can control the Stretch robot using keyboard controls:
- **W/S**: Move base forward/backward
- **A/D**: Turn left/right
- **Q/E**: Lift up/down
- **R/F**: Arm extend/retract
- **T/G**: Wrist yaw rotate
- **Z/X**: Gripper open/close
- **Arrow Keys**: Head pan/tilt
- **ESC**: Exit

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

**ROS 2 Control (Optional)**

For ROS 2 communication, first install ROS 2 (Humble or Jazzy) and then:

```bash
# Install ROS 2 Python package
conda activate simenv
pip install rclpy

# Source ROS 2 (adjust path for your version)
source /opt/ros/humble/setup.bash

# Start the simulation node
python stretch_ros2_sim.py
```

In another terminal, use the keyboard controller:
```bash
conda activate simenv
source /opt/ros/humble/setup.bash
python stretch_keyboard_controller.py
```

Press **A, B, C, or D** to send navigation commands to those anchors.

See [ROS2_SETUP.md](ROS2_SETUP.md) for detailed ROS 2 setup instructions.

### Portability

This repository is designed to work on any computer without modification:
- ✅ All paths are relative to the repository root
- ✅ No hardcoded user-specific paths
- ✅ Works on Linux, macOS, and Windows (with conda)
- ✅ Scripts can be run from any directory

---

## 🎯 Goal

Build a modular simulation environment for the **Stretch 2 robot**, supporting:

* Custom MuJoCo-based environments
* Robot joint and wheel actuation
* Interactive scenes with tables, objects, and ingredients
* (Optional) ROS 2 control and visualization

---

## 📝 TODO

* [ ] Finalize Conda environment file with all dependencies
* [ ] Evaluate alternative simulators (e.g., PyBullet)
* [x] Add ROS 2 integration (Jazzy/Humble) - Basic communication implemented
* [ ] Implement navigation to anchors (A, B, C, D)
* [ ] Implement wheel and actuator motion
* [ ] Add environment objects (tables, containers, ingredients)
* [ ] Create demo scripts and visualization tools

---

## 🔗 Useful Stretch 2 Resources

* Stretch 2 ROS 2 Description
  [https://docs.hello-robot.com/0.2/stretch-ros2/stretch_description/](https://docs.hello-robot.com/0.2/stretch-ros2/stretch_description/)

* Stretch 2 STEP/URDF Files
  [https://github.com/hello-robot/stretch_tool_share/tree/master/tool_share/stretch_2_STEP](https://github.com/hello-robot/stretch_tool_share/tree/master/tool_share/stretch_2_STEP)

* Stretch Tool Share Repository
  [https://github.com/hello-robot/stretch_tool_share/?tab=readme-ov-file](https://github.com/hello-robot/stretch_tool_share/?tab=readme-ov-file)

---
