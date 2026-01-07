#!/usr/bin/env python3
"""Interactive command-line controller for Stretch 2 robot via ROS 2."""

import sys
import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray, Bool
from navigation import NavigationController
import time
import threading

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required. Install it with: pip install pyyaml")
    sys.exit(1)

# Try to import readline for command history (works on Linux/Mac)
READLINE_AVAILABLE = False
try:
    import readline
    READLINE_AVAILABLE = True
except ImportError:
    pass  # readline not available (e.g., on Windows)


class InteractiveController(Node):
    """Interactive command-line controller that sends ROS 2 commands."""
    
    # Parameter ranges for normalization (min, max)
    PARAM_RANGES = {
        'lift': (-0.5, 0.6),
        'arm_extend': (0.0, 0.52),
        'wrist_yaw': (-1.75, 4.0),
        'gripper': (-0.005, 0.04),
        'x': (-1.0, 2.0),  # Approximate workspace bounds
        'y': (1.0, 4.0),   # Approximate workspace bounds
        'direction': (0.0, 2.0 * math.pi),  # 0 to 2π radians
        'speed': (0.0, 100.0)  # Speed percentage (will be converted to 0-1 internally)
    }
    
    def __init__(self, actions_file='actions.yaml'):
        super().__init__('interactive_controller')
        
        self.actions_file = os.path.join(os.path.dirname(__file__), actions_file)
        self.micro_actions = {}
        self.macro_actions = {}
        self._load_actions()
        
        self._setup_publishers()
        self._setup_subscribers()
        self._init_joint_state()
        self._init_wait_state()
        self._init_readline()
        self._print_welcome()
    
    @staticmethod
    def _normalize_to_range(value, min_val, max_val):
        """Normalize a value from 0-1 range to actual range [min_val, max_val]."""
        # Clamp value to [0, 1]
        normalized = max(0.0, min(1.0, float(value)))
        # Map to actual range
        return min_val + normalized * (max_val - min_val)
    
    @staticmethod
    def _normalize_speed(value):
        """Normalize speed from 0-1 to 0-100 percentage."""
        # Clamp to [0, 1]
        normalized = max(0.0, min(1.0, float(value)))
        # Convert to percentage (0-100)
        return normalized * 100.0
    
    def _init_readline(self):
        """Initialize readline for command history."""
        if READLINE_AVAILABLE:
            # Set history file path
            histfile = os.path.join(os.path.expanduser("~"), ".stretch_controller_history")
            try:
                readline.read_history_file(histfile)
            except FileNotFoundError:
                pass
            
            # Set history length
            readline.set_history_length(1000)
            
            # Enable tab completion (optional enhancement)
            readline.parse_and_bind("tab: complete")
            
            # Set completer function
            def completer(text, state):
                options = [name for name in list(self.micro_actions.keys()) + list(self.macro_actions.keys()) 
                          if name.startswith(text)]
                if state < len(options):
                    return options[state]
                return None
            
            readline.set_completer(completer)
    
    def _load_actions(self):
        """Load micro and macro actions from YAML file."""
        try:
            with open(self.actions_file, 'r') as f:
                data = yaml.safe_load(f)
            
            # Load micro actions
            for action in data.get('micro_actions', []):
                self.micro_actions[action['name']] = action
            
            # Load macro actions
            for action in data.get('macro_actions', []):
                self.macro_actions[action['name']] = action
            
            self.get_logger().info(
                f'Loaded {len(self.micro_actions)} micro actions and '
                f'{len(self.macro_actions)} macro actions'
            )
        except FileNotFoundError:
            self.get_logger().error(f'Actions file not found: {self.actions_file}')
            sys.exit(1)
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Error loading actions: {e}')
            sys.exit(1)
    
    def _setup_publishers(self):
        """Initialize ROS 2 publishers."""
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.anchor_pub = self.create_publisher(String, '/stretch/navigate_to_anchor', 10)
        self.turn_towards_pub = self.create_publisher(String, '/stretch/turn_towards_anchor', 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/stretch/joint_commands', 10)
        self.reset_pub = self.create_publisher(String, '/stretch/reset_arm', 10)
        self.position_pub = self.create_publisher(Float64MultiArray, '/stretch/navigate_to_position', 10)
    
    def _setup_subscribers(self):
        """Initialize ROS 2 subscribers."""
        self.joint_state_sub = self.create_subscription(
            JointState, '/stretch/joint_states', self._joint_state_callback, 10
        )
        self.nav_status_sub = self.create_subscription(
            Bool, '/stretch/navigation_active', self._navigation_status_callback, 10
        )
        self.current_joint_states = {}
        self.navigation_active = False
    
    def _joint_state_callback(self, msg):
        """Update current joint states from robot."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def _navigation_status_callback(self, msg):
        """Update navigation status from robot."""
        self.navigation_active = msg.data
    
    def _init_joint_state(self):
        """Initialize joint state tracker."""
        # Order: lift, arm_extend, wrist_yaw, gripper, head_pan, head_tilt
        self.joint_state = {
            'lift': 0.0,
            'arm_extend': 0.0,
            'wrist_yaw': 0.0,
            'gripper': 0.0,
            'head_pan': 0.0,
            'head_tilt': 0.0
        }
        self.joint_order = ['lift', 'arm_extend', 'wrist_yaw', 'gripper', 'head_pan', 'head_tilt']
        self.target_joint_states = {}  # For wait_for_arm
    
    def _init_wait_state(self):
        """Initialize wait state tracking."""
        self.current_joint_states = {}
        self.wait_event = threading.Event()
    
    def _sync_joint_state_from_robot(self):
        """Update self.joint_state from actual robot joint states."""
        # Map robot joint names to controller joint_state keys
        joint_name_map = {
            'joint_lift': 'lift',
            'joint_wrist_yaw': 'wrist_yaw',
            'joint_head_pan': 'head_pan',
            'joint_head_tilt': 'head_tilt',
        }
        
        # Update from current joint states
        for robot_joint_name, controller_key in joint_name_map.items():
            if robot_joint_name in self.current_joint_states:
                self.joint_state[controller_key] = self.current_joint_states[robot_joint_name]
        
        # Handle arm_extend (sum of all arm joint segments)
        arm_joints = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
        arm_total = sum(self.current_joint_states.get(j, 0.0) for j in arm_joints)
        if any(j in self.current_joint_states for j in arm_joints):
            self.joint_state['arm_extend'] = arm_total
        
        # Handle gripper (joint_gripper_slide)
        if 'joint_gripper_slide' in self.current_joint_states:
            self.joint_state['gripper'] = self.current_joint_states['joint_gripper_slide']
    
    def _publish_joint_commands(self, speed_percent=50.0):
        """Publish current joint state as commands with speed control."""
        # Sync joint state from actual robot state before publishing
        # This ensures we don't accidentally move joints that weren't explicitly commanded
        self._sync_joint_state_from_robot()
        
        msg = Float64MultiArray()
        msg.data = [self.joint_state[key] for key in self.joint_order]
        msg.data.append(float(speed_percent))  # Append speed as last element
        self.joint_cmd_pub.publish(msg)
    
    def _print_welcome(self):
        """Print welcome message and available actions in table format."""
        print("\n" + "="*120)
        print("Stretch 2 Interactive Controller".center(120))
        print("="*120)
        
        # Extract usage from description (text in parentheses)
        def extract_usage(description):
            if '(' in description and 'Usage:' in description:
                start = description.find('Usage:') + 6
                end = description.find(')', start)
                if end > start:
                    return description[start:end].strip()
            return ""
        
        # Print Micro Actions table
        print("\n" + "MICRO ACTIONS (Primitive Commands)".center(120))
        print("=" * 120)
        print(f"{'Action Name':<18} {'Usage Example':<50} {'Description':<52}")
        print("=" * 120)
        for name, action in sorted(self.micro_actions.items()):
            desc = action.get('description', 'No description')
            # Remove usage from description for cleaner display
            desc_clean = desc.split('(Usage:')[0].strip() if '(Usage:' in desc else desc
            usage = extract_usage(desc)
            # Truncate if too long
            name_clean = name[:16] + "..." if len(name) > 18 else name
            usage = usage[:48] + "..." if len(usage) > 50 else usage
            desc_clean = desc_clean[:50] + "..." if len(desc_clean) > 52 else desc_clean
            print(f"{name_clean:<18} {usage:<50} {desc_clean:<52}")
        
        # Print Macro Actions table
        print("\n" + "MACRO ACTIONS (Complex Behaviors)".center(120))
        print("=" * 120)
        print(f"{'Action Name':<22} {'Description':<70} {'Status':<28}")
        print("=" * 120)
        for name, action in sorted(self.macro_actions.items()):
            desc = action.get('description', 'No description')
            sequence = action.get('sequence', [])
            status = "✓ Implemented" if sequence and len(sequence) > 0 else "○ Not implemented"
            desc_clean = desc[:68] + "..." if len(desc) > 70 else desc
            print(f"{name:<22} {desc_clean:<70} {status:<28}")
        
        # Print Commands table
        print("\n" + "CONTROLLER COMMANDS".center(120))
        print("=" * 120)
        print(f"{'Command':<35} {'Description':<85}")
        print("=" * 120)
        commands = [
            ("help", "Show this help message"),
            ("help <action_name>", "Show detailed help for a specific action"),
            ("list", "List all available actions"),
            ("exit / quit", "Exit the controller")
        ]
        for cmd, desc in commands:
            print(f"{cmd:<35} {desc:<85}")
        
        print("\n" + "="*120)
        print("Usage: Type an action name followed by parameters (if needed)")
        print("All parameters use normalized 0-1 range: 0=minimum, 0.5=middle/default, 1=maximum")
        print("Example: go_to_anchor anchor=A speed=0.5")
        print("="*120 + "\n")
    
    def _print_help(self, action_name=None):
        """Print help information in table format."""
        if action_name is None:
            self._print_welcome()
            return
        
        # Check micro actions
        if action_name in self.micro_actions:
            action = self.micro_actions[action_name]
            print("\n" + "="*80)
            print(f"Action: {action_name}".center(80))
            print("="*80)
            
            # Create table for action details
            print(f"\n{'Property':<20} {'Value':<60}")
            print("-" * 80)
            print(f"{'Type':<20} {action.get('type', 'unknown'):<60}")
            print(f"{'Description':<20} {action.get('description', 'No description'):<60}")
            
            params = action.get('parameters', {})
            if params:
                print(f"\n{'Parameter':<25} {'Type':<55}")
                print("-" * 80)
                for param, param_type in params.items():
                    print(f"{param:<25} {param_type:<55}")
            else:
                print(f"\n{'Parameter':<25} {'Type':<55}")
                print("-" * 80)
                print(f"{'None':<25} {'N/A':<55}")
            
            print("="*80 + "\n")
            return
        
        # Check macro actions
        if action_name in self.macro_actions:
            action = self.macro_actions[action_name]
            print("\n" + "="*80)
            print(f"Action: {action_name}".center(80))
            print("="*80)
            
            print(f"\n{'Property':<20} {'Value':<60}")
            print("-" * 80)
            print(f"{'Type':<20} {'Macro Action':<60}")
            print(f"{'Description':<20} {action.get('description', 'No description'):<60}")
            
            sequence = action.get('sequence', [])
            if sequence:
                print(f"\n{'Step':<5} {'Action':<25} {'Parameters':<50}")
                print("-" * 80)
                for i, step in enumerate(sequence, 1):
                    step_action = step.get('action', 'unknown')
                    step_params = step.get('parameters', {})
                    params_str = ", ".join([f"{k}={v}" for k, v in step_params.items()]) if step_params else "None"
                    params_str = params_str[:48] + "..." if len(params_str) > 50 else params_str
                    print(f"{i:<5} {step_action:<25} {params_str:<50}")
            else:
                print(f"\n{'Step':<5} {'Action':<25} {'Parameters':<50}")
                print("-" * 80)
                print(f"{'N/A':<5} {'Not yet implemented':<25} {'N/A':<50}")
            
            print("="*80 + "\n")
            return
        
        print(f"\nAction '{action_name}' not found.\n")
        print("Available actions:")
        print("  Micro:", ", ".join(sorted(self.micro_actions.keys())))
        print("  Macro:", ", ".join(sorted(self.macro_actions.keys())))
        print()
    
    def _list_actions(self):
        """List all available actions in table format."""
        print("\n" + "="*80)
        print("Available Actions".center(80))
        print("="*80)
        
        print(f"\n{'Micro Actions':<40} {'Macro Actions':<40}")
        print("-" * 80)
        micro_list = sorted(self.micro_actions.keys())
        macro_list = sorted(self.macro_actions.keys())
        max_len = max(len(micro_list), len(macro_list))
        
        for i in range(max_len):
            micro = micro_list[i] if i < len(micro_list) else ""
            macro = macro_list[i] if i < len(macro_list) else ""
            print(f"{micro:<40} {macro:<40}")
        
        print("="*80 + "\n")
    
    def _parse_command(self, command_line):
        """Parse command line input."""
        parts = command_line.strip().split()
        if not parts:
            return None, {}
        
        action_name = parts[0]
        params = {}
        
        # Parse parameters (simple key=value format)
        for part in parts[1:]:
            if '=' in part:
                key, value = part.split('=', 1)
                # Try to convert to appropriate type
                try:
                    if '.' in value:
                        params[key] = float(value)
                    else:
                        params[key] = int(value)
                except ValueError:
                    params[key] = value
            else:
                # If no =, treat as positional parameter (for backward compatibility)
                if 'anchor' not in params:
                    params['anchor'] = part
                elif 'x' not in params:
                    params['x'] = float(part)
                elif 'y' not in params:
                    params['y'] = float(part)
        
        return action_name, params
    
    def _execute_micro_action(self, action_name, params):
        """Execute a micro action."""
        if action_name not in self.micro_actions:
            print(f"Error: Micro action '{action_name}' not found")
            return False
        
        action = self.micro_actions[action_name]
        action_type = action.get('type', '')
        
        try:
            if action_type == 'navigation':
                speed_normalized = params.get('speed', 0.5)  # Default 0.5 (50%)
                speed_normalized = max(0.0, min(1.0, speed_normalized))  # Clamp to 0-1
                speed_percent = self._normalize_speed(speed_normalized)
                
                if action_name == 'go_to_anchor':
                    anchor = params.get('anchor', params.get('anchor', ''))
                    if not anchor:
                        print("Error: 'anchor' parameter required (e.g., go_to_anchor anchor=A)")
                        return False
                    self._go_to_anchor(anchor.upper(), speed_percent)
                
                elif action_name == 'turn_towards':
                    anchor = params.get('anchor', params.get('anchor', ''))
                    if not anchor:
                        print("Error: 'anchor' parameter required (e.g., turn_towards anchor=ORIGIN)")
                        return False
                    self._turn_towards(anchor.upper(), speed_percent)
                
                elif action_name == 'go_to_position':
                    x_normalized = params.get('x')
                    y_normalized = params.get('y')
                    if x_normalized is None or y_normalized is None:
                        print("Error: 'x' and 'y' parameters required (0-1 range, e.g., go_to_position x=0.5 y=0.5)")
                        return False
                    # Map normalized 0-1 to actual x, y ranges
                    x = self._normalize_to_range(x_normalized, *self.PARAM_RANGES['x'])
                    y = self._normalize_to_range(y_normalized, *self.PARAM_RANGES['y'])
                    direction_normalized = params.get('direction')
                    direction = None
                    if direction_normalized is not None:
                        direction = self._normalize_to_range(direction_normalized, *self.PARAM_RANGES['direction'])
                    self._go_to_position(x, y, direction, speed_percent)
            
            elif action_type == 'arm_control':
                speed_normalized = params.get('speed', 0.5)  # Default 0.5 (50%)
                speed_normalized = max(0.0, min(1.0, speed_normalized))  # Clamp to 0-1
                speed_percent = self._normalize_speed(speed_normalized)
                
                if action_name == 'reset_arm':
                    self._reset_arm(speed_percent)
                
                elif action_name == 'elevate_arm':
                    height_normalized = params.get('height')
                    if height_normalized is None:
                        print("Error: 'height' parameter required (0-1 range, e.g., elevate_arm height=0.5)")
                        return False
                    # Map normalized 0-1 to actual lift range
                    height = self._normalize_to_range(height_normalized, *self.PARAM_RANGES['lift'])
                    self._elevate_arm(height, speed_percent)
                
                elif action_name == 'extend_arm':
                    length_normalized = params.get('length')
                    if length_normalized is None:
                        print("Error: 'length' parameter required (0-1 range, e.g., extend_arm length=0.5)")
                        return False
                    # Map normalized 0-1 to actual arm_extend range
                    length = self._normalize_to_range(length_normalized, *self.PARAM_RANGES['arm_extend'])
                    self._extend_arm(length, speed_percent)
                
                elif action_name == 'rotate_wrist':
                    angle_normalized = params.get('angle')
                    if angle_normalized is None:
                        print("Error: 'angle' parameter required (0-1 range, e.g., rotate_wrist angle=0.5)")
                        return False
                    # Map normalized 0-1 to actual wrist_yaw range
                    angle = self._normalize_to_range(angle_normalized, *self.PARAM_RANGES['wrist_yaw'])
                    self._rotate_wrist(angle, speed_percent)
                
                elif action_name == 'open_gripper':
                    # Open = 1.0 normalized = max gripper value
                    self._set_gripper(self.PARAM_RANGES['gripper'][1], speed_percent)
                
                elif action_name == 'close_gripper':
                    # Close = 0.0 normalized = min gripper value
                    self._set_gripper(self.PARAM_RANGES['gripper'][0], speed_percent)
                
                elif action_name == 'set_gripper':
                    width_normalized = params.get('width')
                    if width_normalized is None:
                        print("Error: 'width' parameter required (0-1 range, e.g., set_gripper width=0.5)")
                        return False
                    # Map normalized 0-1 to actual gripper range
                    width = self._normalize_to_range(width_normalized, *self.PARAM_RANGES['gripper'])
                    self._set_gripper(width, speed_percent)
            
            elif action_type == 'utility':
                if action_name == 'wait':
                    duration = params.get('duration', 1.0)
                    self._wait(duration)
                
                elif action_name == 'wait_for_arm':
                    timeout = params.get('timeout', 10.0)
                    return self._wait_for_arm(timeout)
            
            else:
                print(f"Error: Unknown action type '{action_type}'")
                return False
            
            return True
        
        except Exception as e:
            print(f"Error executing action: {e}")
            return False
    
    def _execute_macro_action(self, action_name, params):
        """Execute a macro action."""
        if action_name not in self.macro_actions:
            print(f"Error: Macro action '{action_name}' not found")
            return False
        
        action = self.macro_actions[action_name]
        sequence = action.get('sequence', [])
        
        if not sequence or len(sequence) == 0:
            print(f"Error: Macro action '{action_name}' is not yet implemented")
            return False
        
        print(f"Executing macro action: {action_name}")
        
        for i, step in enumerate(sequence, 1):
            step_action = step.get('action')
            step_params = step.get('parameters', {})
            
            # Merge macro params with step params (macro params override)
            step_params.update(params)
            
            print(f"  Step {i}: {step_action}")
            
            if step_action in self.micro_actions:
                if not self._execute_micro_action(step_action, step_params):
                    print(f"Error: Failed at step {i}")
                    return False
            elif step_action in self.macro_actions:
                if not self._execute_macro_action(step_action, step_params):
                    print(f"Error: Failed at step {i}")
                    return False
            else:
                print(f"Error: Unknown action '{step_action}' in sequence")
                return False
        
        print(f"✓ Macro action '{action_name}' completed\n")
        return True
    
    # Micro action implementations
    def _go_to_anchor(self, anchor, speed_percent=50.0):
        """Navigate to an anchor point."""
        msg = String()
        msg.data = anchor
        self.anchor_pub.publish(msg)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Navigating to anchor {anchor}{speed_str}")
        # Wait for navigation to complete
        self._wait_for_navigation(timeout=30.0)
    
    def _turn_towards(self, anchor, speed_percent=50.0):
        """Turn robot towards an anchor point without moving."""
        msg = String()
        msg.data = anchor
        self.turn_towards_pub.publish(msg)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Turning towards anchor {anchor}{speed_str}")
        # Wait for navigation to complete
        self._wait_for_navigation(timeout=30.0)
    
    def _wait_for_navigation(self, timeout=30.0):
        """Wait until navigation completes (navigation_active becomes False)."""
        start_time = time.time()
        check_interval = 0.1  # Check every 100ms
        
        # Wait a bit for navigation to start
        time.sleep(0.2)
        
        while time.time() - start_time < timeout:
            if not self.navigation_active:
                # Navigation completed
                return True
            time.sleep(check_interval)
        
        # Timeout reached
        print(f"  ⚠ Navigation timeout after {timeout}s")
        return False
    
    def _go_to_position(self, x, y, direction=None, speed_percent=50.0):
        """Navigate to a specific position."""
        msg = Float64MultiArray()
        msg.data = [float(x), float(y)]
        if direction is not None:
            msg.data.append(float(direction))
        msg.data.append(float(speed_percent))  # Append speed percentage as 4th element
        self.position_pub.publish(msg)
        direction_str = f", direction={direction:.2f}" if direction is not None else ""
        speed_str = f", speed={speed_percent:.0f}%" if speed_percent != 50.0 else ""
        print(f"→ Navigating to position ({x:.2f}, {y:.2f}{direction_str}{speed_str})")
    
    def _reset_arm(self, speed_percent=50.0):
        """Reset arm to default position with speed percentage."""
        msg = String()
        # Format: "reset:SPEED_PERCENT" to pass speed to simulation
        msg.data = f'reset:{speed_percent}'
        self.reset_pub.publish(msg)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Resetting arm to default position{speed_str}")
    
    def _elevate_arm(self, height, speed_percent=50.0):
        """Set lift height with speed control."""
        self.joint_state['lift'] = height
        self._publish_joint_commands(speed_percent)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Elevating arm to height {height:.3f}{speed_str}")
    
    def _extend_arm(self, length, speed_percent=50.0):
        """Set arm extension with speed control."""
        self.joint_state['arm_extend'] = length
        self._publish_joint_commands(speed_percent)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Extending arm to length {length:.3f}{speed_str}")
    
    def _rotate_wrist(self, angle, speed_percent=50.0):
        """Set wrist yaw angle with speed control."""
        self.joint_state['wrist_yaw'] = angle
        self._publish_joint_commands(speed_percent)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Rotating wrist to angle {angle:.3f}{speed_str}")
    
    def _set_gripper(self, width, speed_percent=50.0):
        """Set gripper opening width with speed control."""
        self.joint_state['gripper'] = width
        self._publish_joint_commands(speed_percent)
        speed_str = f" (speed: {speed_percent:.0f}%)" if speed_percent != 50.0 else ""
        print(f"→ Setting gripper to width {width:.3f}{speed_str}")
    
    def _wait(self, duration):
        """Wait for specified duration."""
        print(f"→ Waiting {duration} seconds...")
        time.sleep(duration)
    
    def _wait_for_arm(self, timeout):
        """Wait until arm reaches target positions."""
        print(f"→ Waiting for arm to reach target positions (timeout: {timeout}s)...")
        
        # Get target positions from joint_state
        targets = {
            'joint_lift': self.joint_state.get('lift'),
            'joint_arm_l0': self.joint_state.get('arm_extend') / 4.0,  # Approximate mapping
            'joint_wrist_yaw': self.joint_state.get('wrist_yaw'),
        }
        
        tolerance = 0.05  # Position tolerance
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            all_reached = True
            
            for joint_name, target in targets.items():
                if target is None:
                    continue
                
                current = self.current_joint_states.get(joint_name)
                if current is None:
                    all_reached = False
                    break
                
                if abs(current - target) > tolerance:
                    all_reached = False
                    break
            
            if all_reached:
                print("  ✓ Arm reached target positions")
                return True
            
            # Spin once to get latest joint states
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        print(f"  ⚠ Timeout reached ({timeout}s)")
        return False
    
    def run(self):
        """Run the interactive command loop."""
        print("Type 'help' for available commands, 'exit' to quit")
        if READLINE_AVAILABLE:
            print("Use ↑/↓ arrow keys to navigate command history\n")
        else:
            print("(Command history not available on this system)\n")
        
        try:
            while rclpy.ok():
                try:
                    command = input("stretch> ").strip()
                    
                    if not command:
                        continue
                    
                    # Save command to history (readline does this automatically, but we can also track it)
                    if READLINE_AVAILABLE and command:
                        # readline automatically adds to history, but we ensure it's saved
                        pass
                    
                    if command.lower() in ['exit', 'quit']:
                        print("Exiting...")
                        break
                    
                    if command.lower() == 'help':
                        self._print_help()
                        continue
                    
                    if command.lower().startswith('help '):
                        action_name = command[5:].strip()
                        self._print_help(action_name)
                        continue
                    
                    if command.lower() == 'list':
                        self._list_actions()
                        continue
                    
                    # Parse and execute command
                    action_name, params = self._parse_command(command)
                    
                    if action_name in self.micro_actions:
                        self._execute_micro_action(action_name, params)
                    elif action_name in self.macro_actions:
                        self._execute_macro_action(action_name, params)
                    else:
                        print(f"Error: Unknown action '{action_name}'")
                        print("Type 'list' to see available actions or 'help' for more information")
                
                except EOFError:
                    print("\nExiting...")
                    break
                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                except Exception as e:
                    print(f"Error: {e}")
        
        finally:
            # Save history before exiting
            if READLINE_AVAILABLE:
                histfile = os.path.join(os.path.expanduser("~"), ".stretch_controller_history")
                try:
                    readline.write_history_file(histfile)
                except Exception:
                    pass  # Ignore errors when saving history
            
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = InteractiveController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

