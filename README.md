---

# Stretch2_SimulationEnv

A lightweight simulation environment for the **Hello Robot Stretch 2** platform.
This project uses **MuJoCo**, **Python**, and optionally **ROS 2** to simulate robot motion, interactions, and custom environments.

---

## 🔧 Setup Instructions

### **Create the environment (first time)**

```bash
conda env create -f environment.yml
```

### **Update the environment (after pulling updates from GitHub)**

```bash
conda env update -f environment.yml --prune
```

### **Activate the environment**

```bash
conda activate simenv
```

### **Run a test script**

```bash
python3 test.py
```

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
* [ ] Add ROS 2 integration (Jazzy/Humble)
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
