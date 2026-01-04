# Setup Guide

Complete setup instructions for the Stretch 2 Simulation Environment.

## Prerequisites

- Conda (Miniconda or Anaconda)
- ROS 2 (Humble or Jazzy) - Optional, for ROS 2 communication
- Python 3.11 (for regular simulation) or Python 3.12 (for ROS 2)

## Environment Setup

### Two Environments Strategy

We use **two separate conda environments** to avoid breaking existing packages:

#### 1. `simenv` (Python 3.11) - Regular Simulation
- **Purpose:** All existing work (teleop, view_world, etc.)
- **Python:** 3.11.14
- **Status:** ✅ Keep as-is, no changes needed

#### 2. `simenv_ros2` (Python 3.12) - ROS 2 Environment  
- **Purpose:** ROS 2 communication with Stretch simulation
- **Python:** 3.12
- **Status:** ⚙️ Create this new environment

### Installation Steps

**1. Clone the repository**
```bash
git clone <repository-url>
cd Stretch2_SimulationEnv-main
```

**2. Create the regular environment (first time)**
```bash
conda env create -f environment.yml
conda activate simenv
```

**3. Update the environment (after pulling updates)**
```bash
conda env update -f environment.yml --prune
```

**4. Create ROS 2 environment (optional, for ROS 2 features)**
```bash
# Option 1: Use the automated script
./setup_ros2_env.sh

# Option 2: Manual creation
conda env create -f environment_ros2.yml
conda activate simenv_ros2
```

**5. Verify the setup**
```bash
python verify_setup.py
```

This script checks:
- Conda environment activation
- Required Python packages
- Required files and directories
- MuJoCo model loading

## ROS 2 Setup (Optional)

### Prerequisites

1. **Install ROS 2** (Humble or Jazzy recommended)
   - Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html
   - For Ubuntu 22.04: ROS 2 Humble
   - For Ubuntu 24.04: ROS 2 Jazzy

2. **Activate ROS 2 environment**
   ```bash
   conda activate simenv_ros2
   ```

3. **Install ROS 2 Python packages**
   ```bash
   pip install rclpy
   ```

   Note: The ROS 2 message packages (`geometry_msgs`, `sensor_msgs`, `std_msgs`) should be available if ROS 2 is properly installed.

### ROS 2 Topics

The simulation node publishes and subscribes to the following topics:

**Subscribed Topics (Commands):**
- `/stretch/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/stretch/joint_commands` (std_msgs/Float64MultiArray) - Joint position commands
- `/stretch/navigate_to_anchor` (std_msgs/String) - Navigate to anchor command

**Published Topics (State):**
- `/stretch/joint_states` (sensor_msgs/JointState) - Current joint positions and velocities
- `/stretch/navigation_active` (std_msgs/Bool) - Navigation status
- `/stretch/camera/image_raw` (sensor_msgs/Image) - Camera feed

## Python Version Fixes

### Problem: Python Version Mismatch

ROS 2 Jazzy requires Python 3.12, but your conda environment might use Python 3.11.

### Solution: Separate Environments (Recommended)

Create a **new conda environment** specifically for ROS 2 work, keeping your original `simenv` intact:

```bash
# Create new environment with Python 3.12
conda env create -f environment_ros2.yml

# Activate it
conda activate simenv_ros2

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Test it
python -c "import rclpy; import mujoco; print('✓ Everything works!')"
```

**Benefits:**
- ✅ Your original `simenv` stays untouched
- ✅ All existing packages continue to work
- ✅ ROS 2 has its own clean environment
- ✅ Easy to switch between environments

### Alternative: Update Existing Environment

If you prefer to update your existing environment:

```bash
conda activate simenv
conda install python=3.12 -y
pip install --force-reinstall mujoco mujoco-python-viewer numpy scipy
```

## Environment Comparison

| Feature | simenv (3.11) | simenv_ros2 (3.12) |
|---------|---------------|-------------------|
| Python | 3.11.14 | 3.12 |
| MuJoCo | ✅ | ✅ |
| ROS 2 | ❌ | ✅ |
| teleop.py | ✅ | ✅ |
| view_world.py | ✅ | ✅ |
| stretch_ros2_sim.py | ❌ | ✅ |

## Switching Between Environments

```bash
# Switch to regular environment
conda activate simenv

# Switch to ROS 2 environment
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
```

## Troubleshooting

**Q: "No module named 'rclpy'"**
- Solution: Make sure ROS 2 is installed and you've sourced the setup script
- Try: `source /opt/ros/jazzy/setup.bash` (adjust path for your version)

**Q: "ImportError: cannot import name 'Twist'"**
- Solution: Source ROS 2 setup script before running Python scripts
- The message packages are part of the ROS 2 installation

**Q: Do I need both environments?**  
A: Only if you want to use ROS 2. For regular simulation, just use `simenv`.

**Q: Can I delete simenv_ros2 later?**  
A: Yes! It's completely separate. Just `conda env remove simenv_ros2`.

**Q: Topics not appearing**
- Solution: Make sure both nodes are running and ROS 2 is properly sourced
- Check with: `ros2 topic list`

## Portability

This repository is designed to work on any computer without modification:
- ✅ All paths are relative to the repository root
- ✅ No hardcoded user-specific paths
- ✅ Works on Linux, macOS, and Windows (with conda)
- ✅ Scripts can be run from any directory

