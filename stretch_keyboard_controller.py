#!/usr/bin/env python3
"""Keyboard controller for Stretch 2 robot via ROS 2."""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard
from anchor_utils import get_anchor_list


class StretchKeyboardController(Node):
    """Keyboard controller that sends ROS 2 commands based on key presses."""
    
    def __init__(self):
        super().__init__('stretch_keyboard_controller')
        
        # Load anchor list dynamically from XML
        try:
            self.anchors = get_anchor_list()
            self.get_logger().info(f'Loaded anchors from XML: {self.anchors}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load anchors from XML: {e}, using default')
            self.anchors = ['A', 'B', 'C', 'D']  # Fallback
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.anchor_pub = self.create_publisher(String, '/stretch/navigate_to_anchor', 10)
        
        self.running = True
        self.base_vel = {'linear_x': 0.0, 'angular_z': 0.0}
        
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()
        
        self.create_timer(0.1, self._publish_base_velocity)
        
        anchor_str = '/'.join(self.anchors)
        self.get_logger().info(f'Press {anchor_str} for anchors, W/S for forward/backward, ESC to exit')
    
    def _on_press(self, key):
        """Handle key press events."""
        try:
            if key == keyboard.Key.esc:
                self.running = False
                return False
            
            if hasattr(key, 'char') and key.char:
                key_char = key.char.upper()
                
                # Anchor navigation
                if key_char in self.anchors:
                    self._navigate_to_anchor(key_char)
                    return
                
                # Base movement
                if key_char == 'W':
                    self.base_vel['linear_x'] = 1.0
                elif key_char == 'S':
                    self.base_vel['linear_x'] = -1.0
                elif key_char == 'A':
                    self.base_vel['angular_z'] = 1.0
                elif key_char == 'D':
                    self.base_vel['angular_z'] = -1.0
        except AttributeError:
            pass
    
    def _on_release(self, key):
        """Handle key release events."""
        try:
            if hasattr(key, 'char') and key.char:
                key_char = key.char.upper()
                if key_char in ['W', 'S']:
                    self.base_vel['linear_x'] = 0.0
                elif key_char in ['A', 'D']:
                    self.base_vel['angular_z'] = 0.0
        except AttributeError:
            pass
    
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
    
    # Get anchor list for display
    try:
        anchors = get_anchor_list()
        anchor_str = '/'.join(anchors)
    except:
        anchor_str = "A/B/C/D"
    
    print("\n" + "="*50)
    print("Stretch 2 Keyboard Controller")
    print("="*50)
    print("Controls:")
    print(f"  {anchor_str} - Navigate to anchors")
    print("  W/S - Move forward/backward")
    print("  A/D - Turn left/right (when not used for anchors)")
    print("  ESC - Exit")
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
