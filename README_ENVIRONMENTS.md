# Environment Setup Guide

## Two Environments Strategy

To avoid breaking your existing packages, we use **two separate conda environments**:

### 1. `simenv` (Python 3.11) - Original Environment
- **Purpose:** All your existing work (teleop, view_world, etc.)
- **Python:** 3.11.14
- **Status:** ✅ Keep as-is, no changes needed

### 2. `simenv_ros2` (Python 3.12) - ROS 2 Environment  
- **Purpose:** ROS 2 communication with Stretch simulation
- **Python:** 3.12
- **Status:** ⚙️ Create this new environment

## Quick Start

### Create ROS 2 Environment

```bash
# Option 1: Use the automated script
./setup_ros2_env.sh

# Option 2: Manual creation
conda env create -f environment_ros2.yml
```

### Running Scripts

**For regular simulation (no ROS 2):**
```bash
./run_teleop.sh
# or
conda activate simenv
python teleop.py
```

**For ROS 2 simulation:**
```bash
./run_ros2_sim.sh
# or
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
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

## Benefits of This Approach

✅ **No breaking changes** - Your original environment stays intact  
✅ **Clean separation** - ROS 2 has its own isolated environment  
✅ **Easy switching** - Simple conda activate commands  
✅ **Safe testing** - Can experiment in ROS 2 env without risk  

## Troubleshooting

**Q: Do I need both environments?**  
A: Only if you want to use ROS 2. For regular simulation, just use `simenv`.

**Q: Can I delete simenv_ros2 later?**  
A: Yes! It's completely separate. Just `conda env remove simenv_ros2`.

**Q: Will this take up a lot of disk space?**  
A: The new environment will use ~500MB-1GB. You can share packages between environments using conda's package cache.

**Q: What if I want to update packages?**  
A: Update them separately in each environment:
- `conda activate simenv && pip install --upgrade <package>`
- `conda activate simenv_ros2 && pip install --upgrade <package>`

