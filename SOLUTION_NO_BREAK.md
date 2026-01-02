# Solution: Keep Python 3.11, Use Separate Environment for ROS 2

## Problem

You don't want to break your existing Python 3.11 environment, but ROS 2 Jazzy needs Python 3.12.

## Best Solution: Create Separate Environment

Create a **new conda environment** specifically for ROS 2 work, keeping your original `simenv` intact.

### Option 1: Create New Environment (Recommended)

```bash
# Create new environment with Python 3.12
conda env create -f environment_ros2.yml

# Activate it
conda activate simenv_ros2

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Run ROS 2 simulation
python stretch_ros2_sim.py
```

**Benefits:**
- ✅ Your original `simenv` stays untouched
- ✅ All existing packages continue to work
- ✅ ROS 2 has its own clean environment
- ✅ Easy to switch between environments

### Option 2: Use System Python 3.12 (Alternative)

If you don't want to create a new conda environment, you can use system Python 3.12:

```bash
# Install MuJoCo for system Python 3.12
python3.12 -m pip install --user mujoco mujoco-python-viewer numpy scipy pynput click

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Run with system Python
python3.12 stretch_ros2_sim.py
```

**Benefits:**
- ✅ No conda environment changes
- ✅ Quick setup
- ✅ Works immediately

**Drawbacks:**
- ⚠️ Uses `--user` install (might conflict with system packages)
- ⚠️ Need to manage packages separately

### Option 3: Hybrid Approach (Advanced)

Use system Python 3.12 but access conda packages:

```bash
# Create a wrapper script that uses system Python 3.12
# but adds conda packages to PYTHONPATH
export PYTHONPATH="$HOME/miniconda3/envs/simenv/lib/python3.11/site-packages:$PYTHONPATH"
python3.12 stretch_ros2_sim.py
```

**Note:** This might have issues with C extensions compiled for Python 3.11.

## Recommended: Option 1

I recommend **Option 1** - creating a separate environment. It's the cleanest solution and won't affect your existing work.

## Quick Setup

```bash
# 1. Create the new environment
conda env create -f environment_ros2.yml

# 2. Activate it
conda activate simenv_ros2

# 3. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 4. Test it
python -c "import rclpy; import mujoco; print('✓ Everything works!')"

# 5. Run simulation
python stretch_ros2_sim.py
```

## Switching Between Environments

```bash
# For regular work (Python 3.11)
conda activate simenv

# For ROS 2 work (Python 3.12)
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
```

## What About Other Scripts?

Your other scripts (like `teleop.py`, `view_world.py`) can continue using `simenv` (Python 3.11). Only the ROS 2 scripts need the new environment.

You can even create wrapper scripts:

```bash
#!/bin/bash
# run_teleop.sh
conda activate simenv
python teleop.py
```

```bash
#!/bin/bash
# run_ros2_sim.sh
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

## Summary

- ✅ Keep `simenv` (Python 3.11) for existing work
- ✅ Create `simenv_ros2` (Python 3.12) for ROS 2 work
- ✅ No breaking changes to existing packages
- ✅ Easy to switch between environments

