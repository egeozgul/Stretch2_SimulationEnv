# Fixing Python Version Mismatch for ROS 2

## Problem

ROS 2 Jazzy requires Python 3.12, but your conda environment `simenv` uses Python 3.11. This causes import errors because ROS 2's C extensions are compiled for Python 3.12.

## Solution Options

### Option 1: Update Conda Environment to Python 3.12 (Recommended)

Update your conda environment to use Python 3.12:

```bash
# Deactivate current environment
conda deactivate

# Update the environment to Python 3.12
conda activate simenv
conda install python=3.12 -y

# Reinstall packages that might need recompilation
pip install --force-reinstall mujoco mujoco-python-viewer numpy
```

**Pros:** Clean solution, everything works together  
**Cons:** Need to reinstall some packages

### Option 2: Create New Environment with Python 3.12

Create a new conda environment specifically for ROS 2:

```bash
# Create new environment with Python 3.12
conda create -n simenv_ros2 python=3.12 -y
conda activate simenv_ros2

# Install required packages
conda install -c conda-forge numpy scipy -y
pip install mujoco mujoco-python-viewer pynput click

# ROS 2 will use system Python 3.12 packages
source /opt/ros/jazzy/setup.bash
```

### Option 3: Use System Python 3.12 (Quick Fix)

Install MuJoCo for system Python 3.12:

```bash
# Install MuJoCo for system Python 3.12
python3.12 -m pip install --user mujoco mujoco-python-viewer numpy pynput click

# Then run with system Python
source /opt/ros/jazzy/setup.bash
python3.12 stretch_ros2_sim.py
```

**Pros:** Quick, no environment changes  
**Cons:** System Python packages, might conflict with conda

### Option 4: Use ROS 2 Humble (Python 3.10)

If you prefer to keep Python 3.11, you could switch to ROS 2 Humble which supports Python 3.10 (and might work with 3.11):

```bash
# Install ROS 2 Humble instead
# (Follow ROS 2 Humble installation guide)
```

## Recommended: Option 1

I recommend **Option 1** - updating your existing conda environment to Python 3.12. This ensures everything works together.

## Quick Fix Script

Run this to update your environment:

```bash
#!/bin/bash
conda activate simenv
conda install python=3.12 -y
pip install --force-reinstall mujoco mujoco-python-viewer numpy scipy
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

