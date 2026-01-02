#!/bin/bash
# Script to create a separate ROS 2 environment without breaking existing setup

echo "=========================================="
echo "Creating ROS 2 Environment (Python 3.12)"
echo "=========================================="
echo ""
echo "This will create a NEW conda environment 'simenv_ros2'"
echo "Your existing 'simenv' environment will remain unchanged."
echo ""

read -p "Continue? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

# Check if environment file exists
if [ ! -f "environment_ros2.yml" ]; then
    echo "Error: environment_ros2.yml not found!"
    exit 1
fi

echo "Creating environment..."
conda env create -f environment_ros2.yml

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Environment created successfully!"
    echo "=========================================="
    echo ""
    echo "To use the new environment:"
    echo "  1. conda activate simenv_ros2"
    echo "  2. source /opt/ros/jazzy/setup.bash"
    echo "  3. python stretch_ros2_sim.py"
    echo ""
    echo "Your original 'simenv' environment is unchanged."
else
    echo ""
    echo "Error creating environment. Check the output above."
    exit 1
fi

