import mujoco
import mujoco.viewer
import numpy as np
from pynput import keyboard
import time
from time import sleep # You can keep this or just use time.sleep()
import click
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(script_dir, 'table_world.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)


ctrl_state = {
    'forward': 0.0,
    'turn': 0.0,
    'lift': 0.0,
    'arm_extend': 0.0,
    'wrist_yaw': 0.0,
    'gripper': 0.0,
    'head_pan': 0.0,
    'head_tilt': 0.0
}

actuator_ids = {
    'forward': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'forward'),
    'turn': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'turn'),
    'lift': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'lift'),
    'arm_extend': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'arm_extend'),
    'wrist_yaw': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'wrist_yaw'),
    'gripper': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'grip'),
    'head_pan': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'head_pan'),
    'head_tilt': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'head_tilt')
}

key_buffer = []
running = True

def print_controls():
    click.secho("\n       Keyboard Controls:", fg="yellow")
    click.secho("=====================================", fg="yellow")
    print("W / S: Move BASE Forward/Backward")
    print("A / D: TURN Left/Right")
    print("Q / E: LIFT Up/Down")
    print("R / F: ARM Extend/Retract")
    print("T / G: WRIST YAW Rotate")
    print("Z / X: GRIPPER Open/Close")
    print("Arrow Keys: HEAD Pan/Tilt")
    print("ESC: Stop")
    click.secho("=====================================", fg="yellow")

def keyboard_control(key_char: str):
    if key_char == 'w':
        ctrl_state['forward'] = 2.0
    elif key_char == 's':
        ctrl_state['forward'] = -2.0
    elif key_char == 'a':
        ctrl_state['turn'] = 2.0
    elif key_char == 'd':
        ctrl_state['turn'] = -2.0
    elif key_char == 'q':
        ctrl_state['lift'] = min(ctrl_state['lift'] + 0.05, 0.6)
    elif key_char == 'e':
        ctrl_state['lift'] = max(ctrl_state['lift'] - 0.05, -0.5)
    
    # Arm extend
    elif key_char == 'r':
        ctrl_state['arm_extend'] = min(ctrl_state['arm_extend'] + 0.05, 0.52)
    elif key_char == 'f':
        ctrl_state['arm_extend'] = max(ctrl_state['arm_extend'] - 0.05, 0.0)
    
    # Wrist yaw
    elif key_char == 't':
        ctrl_state['wrist_yaw'] = min(ctrl_state['wrist_yaw'] + 0.1, 4.0)
    elif key_char == 'g':
        ctrl_state['wrist_yaw'] = max(ctrl_state['wrist_yaw'] - 0.1, -1.75)
    
    # Gripper
    elif key_char == 'z':
        ctrl_state['gripper'] = min(ctrl_state['gripper'] + 0.005, 0.04)
    elif key_char == 'x':
        ctrl_state['gripper'] = max(ctrl_state['gripper'] - 0.005, -0.005)

def keyboard_control_release(key_char: str):
    if key_char in ('w', 's'):
        ctrl_state['forward'] = 0.0
    elif key_char in ('a', 'd'):
        ctrl_state['turn'] = 0.0

def on_press(key):
    global key_buffer, running
    
    try:
        if key == keyboard.Key.esc:
            running = False
            return False
        elif key == keyboard.Key.left:
            ctrl_state['head_pan'] = min(ctrl_state['head_pan'] + 0.1, 1.5)
        elif key == keyboard.Key.right:
            ctrl_state['head_pan'] = max(ctrl_state['head_pan'] - 0.1, -3.9)
        elif key == keyboard.Key.up:
            ctrl_state['head_tilt'] = min(ctrl_state['head_tilt'] + 0.1, 0.79)
        elif key == keyboard.Key.down:
            ctrl_state['head_tilt'] = max(ctrl_state['head_tilt'] - 0.1, -1.53)
        
        # Handle character keys
        if hasattr(key, 'char') and key.char:
            if key not in key_buffer and len(key_buffer) < 5:
                key_buffer.append(key)
    except AttributeError:
        pass

def on_release(key):
    global key_buffer
    
    if key in key_buffer:
        key_buffer.remove(key)
    
    if hasattr(key, 'char') and key.char:
        keyboard_control_release(key.char)

# Initialize robot position
data.qpos[7] = 0.0  # Lift at bottom
data.qpos[8:12] = [0.0, 0.0, 0.0, 0.0]  # Arm retracted
mujoco.mj_forward(model, data)

print_controls()

# Start keyboard listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0, 3, 1]
    viewer.cam.distance = 5
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 90
    
    try:
        prev_render_time = time.time()

        while viewer.is_running() and running:
            step_start = time.time()  # Record start of the cycle

            # 1. Process all keys in buffer
            for key in key_buffer:
                if hasattr(key, 'char') and key.char:
                    keyboard_control(key.char)
            
            # 2. Apply controls
            data.ctrl[actuator_ids['forward']] = ctrl_state['forward']
            # ... (apply the rest of your ctrl_state mapping here) ...

            # 3. Step the physics
            mujoco.mj_step(model, data)

            # ELEGANT FIX: Only render at ~60fps (every 0.016 seconds)
            # This frees up the CPU to focus on physics
            now = time.time()
            if now - prev_render_time > 1.0/20.0:
                viewer.sync()
                prev_render_time = now

            # Sync to real-time
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        listener.stop()
