#!/bin/bash
# Wrapper to run teleop with original environment

# Activate original environment
eval "$(conda shell.bash hook)"
conda activate simenv

# Run teleop
python teleop.py

