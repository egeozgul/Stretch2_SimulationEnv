#!/usr/bin/env python3.12
"""
Wrapper script that uses system Python 3.12 (for ROS 2 compatibility)
while accessing conda environment packages (like MuJoCo).
"""

import sys
import os

# Add conda environment site-packages to path
conda_env_path = os.path.expanduser('~/miniconda3/envs/simenv/lib/python3.11/site-packages')
if os.path.exists(conda_env_path):
    sys.path.insert(0, conda_env_path)

# Now import and run the actual script
if __name__ == '__main__':
    # Import the main module
    import stretch_ros2_sim
    
    # Run main
    stretch_ros2_sim.main()

