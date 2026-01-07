#!/usr/bin/env python3
"""ROS 2 node for Stretch 2 MuJoCo simulation."""

import os
import time
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray, String, Bool
import mujoco
import mujoco.viewer
import numpy as np
import cv2
from navigation import NavigationController
from anchor_utils import load_anchors_from_xml

JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt'
]

JOINT_QPOS_MAP = {
    'joint_lift': 7, 'joint_arm_l0': 8, 'joint_arm_l1': 9,
    'joint_arm_l2': 10, 'joint_arm_l3': 11
}

JOINT_LIMITS = {
    'lift': (-0.5, 0.6), 'arm_extend': (0.0, 0.52), 'wrist_yaw': (-1.75, 4.0),
    'wrist_roll': (-3.14, 3.14), 'gripper': (-0.005, 0.04),
    'head_pan': (-3.9, 1.5), 'head_tilt': (-1.53, 0.79)
}

ACTUATOR_NAMES = ['forward', 'turn', 'lift', 'arm_extend', 'wrist_yaw', 'wrist_roll', 
                  'grip', 'head_pan', 'head_tilt']

# Joint command mapping: (command_name, actuator_name)
JOINT_COMMAND_MAP = [
    ('lift', 'lift'), ('arm_extend', 'arm_extend'), ('wrist_yaw', 'wrist_yaw'),
    ('wrist_roll', 'wrist_roll'), ('gripper', 'grip'), ('head_pan', 'head_pan'),
    ('head_tilt', 'head_tilt')
]


class StretchSimNode(Node):
    """ROS 2 node that runs MuJoCo simulation and handles ROS 2 communication."""
    
    def __init__(self):
        super().__init__('stretch_sim')
        
        xml_path = os.path.join(os.path.dirname(__file__), 'table_world.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        try:
            self.anchors = load_anchors_from_xml(xml_path)
            self.get_logger().info(f'Loaded {len(self.anchors)} anchors: {sorted(self.anchors.keys())}')
            for letter, data in sorted(self.anchors.items()):
                self.get_logger().info(f'  {letter}: {data}')
        except Exception as e:
            self.get_logger().error(f'Failed to load anchors: {e}')
            self.anchors = {}
        
        self.data.qpos[7] = 0.0  # Lift at bottom
        self.data.qpos[8:12] = 0.0  # Arm retracted
        mujoco.mj_forward(self.model, self.data)
        
        self.actuator_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        }
        self.ctrl_state = {name: 0.0 for name in ACTUATOR_NAMES}
        
        self.joint_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        }
        
        self.base_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'camera_rgb')
        if self.camera_id < 0:
            self.get_logger().warn('Camera "camera_rgb" not found, camera display disabled')
            self.camera_id = None
        else:
            self.camera_width, self.camera_height = 640, 480
            self.camera_renderer = None
            self.camera_obj = None
            self.get_logger().info(f'Camera "camera_rgb" found (ID: {self.camera_id})')
        
        self.nav_controller = NavigationController()
        self.manual_control = False
        self._setup_ros2()
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
        self.camera_pub = self.create_publisher(Image, '/stretch/camera/image_raw', 10)
    
    def _cmd_vel_callback(self, msg):
        """Handle base velocity commands."""
        has_manual_input = abs(msg.linear.x) > 0.05 or abs(msg.angular.z) > 0.05
        
        if has_manual_input and self.nav_controller.is_active():
            self.manual_control = True
            self.nav_controller.cancel()
            self.get_logger().info('Manual control override')
        
        self.ctrl_state['forward'] = msg.linear.x
        self.ctrl_state['turn'] = msg.angular.z
    
    def _joint_commands_callback(self, msg):
        """Handle joint position commands."""
        if len(msg.data) < len(JOINT_COMMAND_MAP):
            return
        
        for (cmd_name, actuator_name), value in zip(JOINT_COMMAND_MAP, msg.data):
            min_val, max_val = JOINT_LIMITS[cmd_name]
            self.ctrl_state[actuator_name] = np.clip(value, min_val, max_val)
    
    def _navigate_to_anchor_callback(self, msg):
        """Handle navigate to anchor command."""
        anchor_key = msg.data.strip().upper()
        if anchor_key in self.anchors:
            anchor_data = self.anchors[anchor_key]
            target_pos = anchor_data['pos']
            target_direction = anchor_data.get('direction')
            
            current_pos, _ = self._get_robot_pose()
            distance = np.linalg.norm(np.array(target_pos[:2]) - current_pos[:2])
            
            self.nav_controller.set_target(target_pos, target_direction)
            self.manual_control = False
            
            direction_info = f", direction={math.degrees(target_direction):.1f}°" if target_direction else ""
            self.get_logger().info(f'Navigating to anchor {anchor_key} at {target_pos} '
                                 f'(distance: {distance:.2f}m{direction_info})')
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
        
        self.get_logger().info(f'Nav: dist={distance:.2f}m, lin={linear_vel:.2f} m/s, '
                             f'ctrl={actual_ctrl:.3f}, ang={angular_vel:.2f}, '
                             f'err={angle_error_deg:.1f}°, aligned={aligned}')
    
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
        if self._nav_log_counter % (10 if abs(linear_vel) > 0.01 else 50) == 0:
            self._log_navigation_status(pos, quat, linear_vel, angular_vel)
        
        self.ctrl_state['forward'] = linear_vel
        self.ctrl_state['turn'] = angular_vel
        
        if self.nav_controller.has_reached():
            self.get_logger().info('✓ Reached target position!')
            self.ctrl_state['forward'] = 0.0
            self.ctrl_state['turn'] = 0.0
    
    def _get_joint_state(self, joint_name):
        """Get position and velocity for a joint."""
        if joint_name in JOINT_QPOS_MAP:
            idx = JOINT_QPOS_MAP[joint_name]
            if idx < len(self.data.qpos):
                return (float(self.data.qpos[idx]), 
                       float(self.data.qvel[idx]) if idx < len(self.data.qvel) else 0.0)
        
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
    
    def _render_camera(self):
        """Render camera view and display it."""
        if self.camera_id is None or self.camera_renderer is None:
            return
        
        try:
            self.camera_obj.fixedcamid = self.camera_id
            self.camera_obj.type = mujoco.mjtCamera.mjCAMERA_FIXED
            
            self.camera_renderer.update_scene(self.data, camera=self.camera_obj)
            camera_rgb = self.camera_renderer.render()
            camera_bgr = cv2.cvtColor(camera_rgb, cv2.COLOR_RGB2BGR)
            camera_bgr_rotated = cv2.rotate(camera_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            cv2.imshow('Robot Camera', camera_bgr_rotated)
            cv2.waitKey(1)
            
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_rgb'
            img_msg.height = self.camera_height
            img_msg.width = self.camera_width
            img_msg.encoding = 'rgb8'
            img_msg.is_bigendian = False
            img_msg.step = self.camera_width * 3
            img_msg.data = camera_rgb.tobytes()
            self.camera_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Camera rendering error: {e}')
    
    def run_simulation(self):
        """Run the MuJoCo simulation loop."""
        #with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
        with mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as viewer:
            viewer.cam.lookat[:] = [0, 3, 1]
            viewer.cam.distance = 5
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 90
            
            if self.camera_id is not None:
                try:
                    self.camera_renderer = mujoco.Renderer(self.model, 
                                                          height=self.camera_height, 
                                                          width=self.camera_width)
                    self.camera_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
                    self.camera_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = True
                    self.camera_obj = mujoco.MjvCamera()
                    self.camera_obj.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    self.camera_obj.fixedcamid = self.camera_id
                    cv2.namedWindow('Robot Camera', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('Robot Camera', self.camera_height, self.camera_width)
                    self.get_logger().info('Camera rendering initialized')
                except Exception as e:
                    self.get_logger().error(f'Failed to initialize camera rendering: {e}')
                    self.camera_id = None
            
            prev_render_time = prev_pub_time = prev_camera_time = time.time()
            
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
                
                if now - prev_camera_time > 1.0/30.0:
                    self._render_camera()
                    prev_camera_time = now
                
                if now - prev_render_time > 1.0/20.0:
                    viewer.sync()
                    prev_render_time = now
                
                elapsed = time.time() - step_start
                if elapsed < self.model.opt.timestep:
                    time.sleep(self.model.opt.timestep - elapsed)
            
            if self.camera_id is not None:
                cv2.destroyAllWindows()
    
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
