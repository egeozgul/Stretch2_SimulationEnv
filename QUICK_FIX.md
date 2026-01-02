# Quick Fix for Python Version Issue

## The Problem

Your conda environment uses Python 3.11, but ROS 2 Jazzy requires Python 3.12. This causes the error:
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

## Quick Solution

Run this command to update your environment:

```bash
conda activate simenv
conda install python=3.12 -y
pip install --force-reinstall mujoco mujoco-python-viewer numpy scipy pynput click
```

Then test:
```bash
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

## Alternative: Use the Fix Script

```bash
./fix_python_version.sh
```

This will automatically update your environment to Python 3.12.

## After Fixing

Once Python 3.12 is installed, you should be able to run:

```bash
conda activate simenv
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

And it should work! 🎉

