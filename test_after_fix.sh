#!/bin/bash
# Test script to verify the fix works

echo "Testing ROS 2 with Python 3.12..."
echo ""

# Activate conda environment
eval "$(conda shell.bash hook)"
conda activate simenv

# Check Python version
PYTHON_VERSION=$(python --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

if [[ "$PYTHON_VERSION" == "3.12"* ]]; then
    echo "✓ Python 3.12 detected"
else
    echo "✗ Wrong Python version. Run: conda install python=3.12 -y"
    exit 1
fi

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Test imports
echo ""
echo "Testing imports..."
python -c "import rclpy; print('✓ rclpy works')" 2>&1
python -c "import mujoco; print('✓ mujoco works')" 2>&1

echo ""
echo "If both imports work, you're ready to run:"
echo "  python stretch_ros2_sim.py"
