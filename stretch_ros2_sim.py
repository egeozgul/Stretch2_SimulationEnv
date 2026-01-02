#!/usr/bin/env python3
"""ROS 2 node for Stretch 2 MuJoCo simulation."""

import os
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool
import mujoco
import mujoco.viewer
import numpy as np
from navigation import NavigationController, MAX_LINEAR_VEL, MAX_ANGULAR_VEL
from anchor_utils import load_anchors_from_xml

JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt'
]

JOINT_QPOS_MAP = {
    'joint_lift': 7,
    'joint_arm_l0': 8,
    'joint_arm_l1': 9,
    'joint_arm_l2': 10,
    'joint_arm_l3': 11
}

JOINT_LIMITS = {
    'lift': (-0.5, 0.6),
    'arm_extend': (0.0, 0.52),
    'wrist_yaw': (-1.75, 4.0),
    'gripper': (-0.005, 0.04),
    'head_pan': (-3.9, 1.5),
    'head_tilt': (-1.53, 0.79)
}

ACTUATOR_NAMES = ['forward', 'turn', 'lift', 'arm_extend', 'wrist_yaw', 'grip', 'head_pan', 'head_tilt']


class StretchSimNode(Node):
    """ROS 2 node that runs MuJoCo simulation and handles ROS 2 communication."""
    
    def __init__(self):
        super().__init__('stretch_sim')
        
        # Load MuJoCo model
        xml_path = os.path.join(os.path.dirname(__file__), 'table_world.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        try:
            self.anchors = load_anchors_from_xml(xml_path)
            self.get_logger().info(f'Loaded {len(self.anchors)} anchors: {sorted(self.anchors.keys())}')
            for letter, pos in sorted(self.anchors.items()):
                self.get_logger().info(f'  {letter}: {pos}')
        except Exception as e:
            self.get_logger().error(f'Failed to load anchors: {e}')
            self.anchors = {}
        
        # Initialize robot position
        self.data.qpos[7] = 0.0  # Lift at bottom
        self.data.qpos[8:12] = 0.0  # Arm retracted
        mujoco.mj_forward(self.model, self.data)
        
        # Get actuator IDs
        self.actuator_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        }
        
        # Control state (all zeros initially)
        self.ctrl_state = {name: 0.0 for name in ACTUATOR_NAMES}
        
        self.joint_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        }
        
        # Get base_link body ID for navigation
        self.base_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        
        self.nav_controller = NavigationController()
        self.manual_control = False
        
        # ROS 2 setup
        self._setup_ros2()
        
        # Simulation state
        self.running = True
        self.sim_thread = None
        
        self.get_logger().info('Stretch 2 ROS 2 Simulation Node started')
    
    def _setup_ros2(self):
        """Setup ROS 2 publishers and subscribers."""
        self.create_subscription(Twist, '/stretch/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/stretch/joint_commands', 
                                self._joint_commands_callback, 10)
        self.create_subscription(String, '/stretch/navigate_to_anchor', 
                                self._navigate_to_anchor_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, '/stretch/joint_states', 10)
        self.nav_status_pub = self.create_publisher(Bool, '/stretch/navigation_active', 10)
    
    def _cmd_vel_callback(self, msg):
        """Handle base velocity commands."""
        has_manual_input = abs(msg.linear.x) > 0.05 or abs(msg.angular.z) > 0.05
        
        if has_manual_input:
            if self.nav_controller.is_active():
                self.manual_control = True
                self.nav_controller.cancel()
                self.get_logger().info('Manual control override')
            self.ctrl_state['forward'] = msg.linear.x
            self.ctrl_state['turn'] = msg.angular.z
    
    def _joint_commands_callback(self, msg):
        """Handle joint position commands."""
        if len(msg.data) < 6:
            return
        
        commands = ['lift', 'arm_extend', 'wrist_yaw', 'gripper', 'head_pan', 'head_tilt']
        for cmd, value in zip(commands, msg.data):
            min_val, max_val = JOINT_LIMITS[cmd]
            self.ctrl_state[cmd] = np.clip(value, min_val, max_val)
    
    def _navigate_to_anchor_callback(self, msg):
        """Handle navigate to anchor command."""
        anchor_key = msg.data.strip().upper()
        if anchor_key in self.anchors:
            target_pos = self.anchors[anchor_key]
            current_pos, _ = self._get_robot_pose()
            distance = np.linalg.norm(np.array(target_pos[:2]) - current_pos[:2])
            self.nav_controller.set_target(target_pos)
            self.manual_control = False
            self.get_logger().info(f'Navigating to anchor {anchor_key} at {target_pos} (distance: {distance:.2f}m)')
        else:
            available = ', '.join(sorted(self.anchors.keys())) if self.anchors else 'none'
            self.get_logger().warn(f'Unknown anchor: {anchor_key}. Available: {available}')
    
    def _get_robot_pose(self):
        """Get current robot position and orientation."""
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()
    
    def _log_navigation_status(self, pos, quat, linear_vel, angular_vel):
        """Log navigation status for debugging."""
        target = self.nav_controller.target_pos
        if target is None:
            return
        
        distance = np.linalg.norm(target - pos[:2])
        diff = target - pos[:2]
        desired_angle = np.arctan2(diff[1], diff[0])
        w, x, y, z = quat
        current_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        angle_error = np.arctan2(np.sin(desired_angle - current_yaw),
                                np.cos(desired_angle - current_yaw))
        angle_error_deg = np.degrees(abs(angle_error))
        aligned = angle_error_deg <= 10.0
        actual_ctrl = self.ctrl_state.get('forward', 0.0)
        
        self.get_logger().info(f'Nav: dist={distance:.2f}m, lin={linear_vel:.2f} m/s, ctrl={actual_ctrl:.3f}, '
                             f'ang={angular_vel:.2f}, err={angle_error_deg:.1f}°, aligned={aligned}')
    
    def _update_navigation(self):
        """Update navigation controller and apply its commands."""
        nav_status = Bool()
        nav_status.data = self.nav_controller.is_active() and not self.manual_control
        self.nav_status_pub.publish(nav_status)
        
        if not self.nav_controller.is_active() or self.manual_control:
            return
        
        pos, quat = self._get_robot_pose()
        linear_vel, angular_vel = self.nav_controller.get_control(pos, quat)
        
        self._nav_log_counter = getattr(self, '_nav_log_counter', 0) + 1
        if self._nav_log_counter % (10 if linear_vel > 0.01 else 50) == 0:
            self._log_navigation_status(pos, quat, linear_vel, angular_vel)
        
        # Scale to actuator control (2.5 m/s -> 2.0 control, matching teleop.py)
        self.ctrl_state['forward'] = linear_vel * (2.0 / MAX_LINEAR_VEL)
        self.ctrl_state['turn'] = angular_vel * (2.0 / MAX_ANGULAR_VEL)
        
        if self.nav_controller.has_reached():
            self.get_logger().info('✓ Reached target position!')
            self.ctrl_state['forward'] = 0.0
            self.ctrl_state['turn'] = 0.0
    
    def _get_joint_state(self, joint_name):
        """Get position and velocity for a joint."""
        # Try known qpos index first
        if joint_name in JOINT_QPOS_MAP:
            idx = JOINT_QPOS_MAP[joint_name]
            if idx < len(self.data.qpos):
                return (float(self.data.qpos[idx]), 
                       float(self.data.qvel[idx]) if idx < len(self.data.qvel) else 0.0)
        
        # Try using joint ID
        if joint_name in self.joint_ids:
            try:
                joint_id = self.joint_ids[joint_name]
                qpos_addr = self.model.jnt_qposadr[joint_id]
                qvel_addr = self.model.jnt_dofadr[joint_id]
                pos = self.data.qpos[qpos_addr] if 0 <= qpos_addr < len(self.data.qpos) else 0.0
                vel = self.data.qvel[qvel_addr] if 0 <= qvel_addr < len(self.data.qvel) else 0.0
                return (float(pos), float(vel))
            except (AttributeError, IndexError):
                pass
        
        return (0.0, 0.0)
    
    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for joint_name in JOINT_NAMES:
            pos, vel = self._get_joint_state(joint_name)
            msg.name.append(joint_name)
            msg.position.append(pos)
            msg.velocity.append(vel)
            msg.effort.append(0.0)
        
        self.joint_state_pub.publish(msg)
    
    def run_simulation(self):
        """Run the MuJoCo simulation loop."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.lookat[:] = [0, 3, 1]
            viewer.cam.distance = 5
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 90
            
            prev_render_time = prev_pub_time = time.time()
            
            while self.running and viewer.is_running():
                step_start = time.time()
                
                self._update_navigation()
                
                for name, actuator_id in self.actuator_ids.items():
                    self.data.ctrl[actuator_id] = self.ctrl_state[name]
                
                mujoco.mj_step(self.model, self.data)
                
                now = time.time()
                if now - prev_pub_time > 1.0/30.0:
                    self.publish_joint_states()
                    prev_pub_time = now
                
                if now - prev_render_time > 1.0/20.0:
                    viewer.sync()
                    prev_render_time = now
                
                # Sync to real-time
                elapsed = time.time() - step_start
                if elapsed < self.model.opt.timestep:
                    time.sleep(self.model.opt.timestep - elapsed)
    
    def start_simulation(self):
        """Start simulation in a separate thread."""
        self.sim_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.sim_thread.start()
    
    def stop(self):
        """Stop the simulation."""
        self.running = False
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = StretchSimNode()
    node.start_simulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
