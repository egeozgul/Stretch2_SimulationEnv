#!/usr/bin/env python3
"""Keyboard controller for Stretch 2 robot via ROS 2."""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from pynput import keyboard
from anchor_utils import get_anchor_list


class StretchKeyboardController(Node):
    """Keyboard controller that sends ROS 2 commands based on key presses."""
    
    JOINT_LIMITS = {
        'lift': (-0.5, 0.6),
        'arm_extend': (0.0, 0.52),
        'wrist_yaw': (-1.75, 4.0),
        'wrist_roll': (-3.14, 3.14),
        'gripper': (-0.005, 0.04),
        'head_pan': (-3.9, 1.5),
        'head_tilt': (-1.53, 0.79)
    }
    
    JOINT_ORDER = ['lift', 'arm_extend', 'wrist_yaw', 'wrist_roll', 'gripper', 'head_pan', 'head_tilt']
    
    # Key mappings: (key, joint_name, delta)
    JOINT_CONTROLS = {
        'Q': ('lift', 0.05), 'E': ('lift', -0.05),
        'R': ('arm_extend', 0.05), 'F': ('arm_extend', -0.05),
        'T': ('wrist_yaw', 0.1), 'G': ('wrist_yaw', -0.1),
        'V': ('wrist_roll', 0.1), 'B': ('wrist_roll', -0.1),
        'Z': ('gripper', 0.01), 'X': ('gripper', -0.01)
    }
    
    HEAD_CONTROLS = {
        keyboard.Key.left: ('head_pan', 0.1),
        keyboard.Key.right: ('head_pan', -0.1),
        keyboard.Key.up: ('head_tilt', 0.1),
        keyboard.Key.down: ('head_tilt', -0.1)
    }
    
    BASE_VELOCITY = {
        'W': {'linear_x': -1.0}, 'S': {'linear_x': 1.0},
        'A': {'angular_z': -1.0}, 'D': {'angular_z': 1.0}
    }
    
    def __init__(self):
        super().__init__('stretch_keyboard_controller')
        
        try:
            anchor_list = get_anchor_list()
            self.anchor_map = {str(i): letter for i, letter in 
                              enumerate(sorted(anchor_list)[:4], start=1)}
        except Exception as e:
            self.get_logger().warn(f'Failed to load anchors: {e}, using default')
            self.anchor_map = {'1': 'A', '2': 'B', '3': 'C', '4': 'D'}
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.anchor_pub = self.create_publisher(String, '/stretch/navigate_to_anchor', 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/stretch/joint_commands', 10)
        
        self.running = True
        self.base_vel = {'linear_x': 0.0, 'angular_z': 0.0}
        self.joint_state = {key: 0.0 for key in self.JOINT_LIMITS.keys()}
        
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()
        self.create_timer(0.1, self._publish_base_velocity)
    
    def _on_press(self, key):
        """Handle key press events."""
        try:
            if key == keyboard.Key.esc:
                self.running = False
                return False
            
            if key in self.HEAD_CONTROLS:
                joint_name, delta = self.HEAD_CONTROLS[key]
                self._update_joint(joint_name, delta)
                return
            
            if hasattr(key, 'char') and key.char:
                key_char = key.char.upper()
                
                if key_char in self.anchor_map:
                    self._navigate_to_anchor(self.anchor_map[key_char])
                    return
                
                if key_char in self.BASE_VELOCITY:
                    self.base_vel.update(self.BASE_VELOCITY[key_char])
                    return
                
                if key_char in self.JOINT_CONTROLS:
                    joint_name, delta = self.JOINT_CONTROLS[key_char]
                    self._update_joint(joint_name, delta)
        except AttributeError:
            pass
    
    def _on_release(self, key):
        """Handle key release events."""
        try:
            if hasattr(key, 'char') and key.char:
                key_char = key.char.upper()
                if key_char in ['W', 'S']:
                    self.base_vel['linear_x'] = 0.0
                    self._publish_base_velocity()
                elif key_char in ['A', 'D']:
                    self.base_vel['angular_z'] = 0.0
                    self._publish_base_velocity()
        except AttributeError:
            pass
    
    def _update_joint(self, joint_name, delta):
        """Update joint position and publish command."""
        min_val, max_val = self.JOINT_LIMITS[joint_name]
        self.joint_state[joint_name] = max(min_val, min(max_val, 
                                                         self.joint_state[joint_name] + delta))
        self._publish_joint_commands()
    
    def _navigate_to_anchor(self, anchor_key):
        """Send navigation command to specified anchor."""
        msg = String()
        msg.data = anchor_key
        self.anchor_pub.publish(msg)
        self.get_logger().info(f'Navigating to anchor {anchor_key}')
    
    def _publish_base_velocity(self):
        """Publish current base velocity."""
        msg = Twist()
        msg.linear.x = self.base_vel['linear_x']
        msg.angular.z = self.base_vel['angular_z']
        self.cmd_vel_pub.publish(msg)
    
    def _publish_joint_commands(self):
        """Publish current joint commands."""
        msg = Float64MultiArray()
        msg.data = [self.joint_state[key] for key in self.JOINT_ORDER]
        self.joint_cmd_pub.publish(msg)
    
    def stop(self):
        """Stop the controller."""
        self.running = False
        self.base_vel = {'linear_x': 0.0, 'angular_z': 0.0}
        self._publish_base_velocity()
        if self.listener:
            self.listener.stop()


def main(args=None):
    rclpy.init(args=args)
    controller = StretchKeyboardController()
    
    try:
        anchor_list = get_anchor_list()
        anchor_map = {str(i): letter for i, letter in 
                     enumerate(sorted(anchor_list)[:4], start=1)}
        anchor_str = '/'.join([f'{k}→{v}' for k, v in sorted(anchor_map.items())])
    except:
        anchor_str = "1→A/2→B/3→C/4→D"
    
    print("\n" + "="*50)
    print("Stretch 2 Keyboard Controller")
    print("="*50)
    print("Base Movement:")
    print(f"  {anchor_str} - Navigate to anchors (1-4)")
    print("  W/S - Move forward/backward")
    print("  A/D - Turn left/right")
    print("\nArm & Gripper:")
    print("  Q/E - Lift up/down")
    print("  R/F - Arm extend/retract")
    print("  T/G - Wrist yaw rotate")
    print("  V/B - Wrist roll rotate")
    print("  Z/X - Gripper open/close")
    print("\nCamera/Head:")
    print("  Arrow Keys - Pan/tilt head")
    print("\n  ESC - Exit")
    print("="*50 + "\n")
    
    try:
        while controller.running:
            rclpy.spin_once(controller, timeout_sec=0.1)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
