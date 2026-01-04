"""Navigation controller for moving robot to target positions."""

import math
import numpy as np

# Navigation parameters
POSITION_TOLERANCE = 0.15  # meters
ALIGNMENT_THRESHOLD = math.radians(15)  # degrees
MAX_ANGLE_FOR_MOVEMENT = math.radians(45)  # degrees
DIRECTION_TOLERANCE = math.radians(5)  # degrees
MAX_LINEAR_VEL = 2.5  # m/s
MAX_ANGULAR_VEL = 2.0  # rad/s
K_P_ANGULAR = 2.0


class NavigationController:
    """Proportional controller for navigating to target positions."""
    
    def __init__(self):
        self.target_pos = None
        self.target_direction = None
        self.active = False
        self.reached = False
        self._was_aligned = False
        self._position_reached = False
    
    def set_target(self, target_pos, target_direction=None):
        """Set target position and optional direction."""
        self.target_pos = np.array(target_pos[:2])
        self.target_direction = target_direction
        self.active = True
        self.reached = False
        self._was_aligned = False
        self._position_reached = False
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-π, π] range."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    @staticmethod
    def _quaternion_to_yaw(quat):
        """Extract yaw angle from quaternion [w, x, y, z]."""
        w, x, y, z = quat if len(quat) == 4 else (1.0, 0.0, 0.0, 0.0)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def _calculate_angle_error(self, desired_angle, current_yaw):
        """Calculate angle error, choosing shortest rotation path."""
        return self._normalize_angle(desired_angle - current_yaw)
    
    def get_control(self, current_pos, current_quat):
        """
        Get control velocities to reach target.
        
        Returns:
            (linear_vel, angular_vel) tuple in (m/s, rad/s)
        """
        if not self.active or self.target_pos is None:
            return (0.0, 0.0)
        
        diff = self.target_pos - np.array(current_pos[:2])
        distance = np.linalg.norm(diff)
        current_yaw = self._quaternion_to_yaw(current_quat)
        
        if distance > 100.0:
            self.active = False
            return (0.0, 0.0)
        
        # Mark position as reached when within tolerance
        if not self._position_reached and distance < POSITION_TOLERANCE:
            self._position_reached = True
        
        # Phase 1: Navigate to position
        if not self._position_reached:
            desired_angle = math.atan2(diff[1], diff[0])
            angle_error = self._calculate_angle_error(desired_angle, current_yaw)
            angle_error_abs = abs(angle_error)
            
            angular_vel = np.clip(-K_P_ANGULAR * angle_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
            
            if angle_error_abs <= MAX_ANGLE_FOR_MOVEMENT:
                self._was_aligned = True
                if angle_error_abs <= ALIGNMENT_THRESHOLD:
                    linear_vel = MAX_LINEAR_VEL
                else:
                    speed_factor = 1.0 - (angle_error_abs / MAX_ANGLE_FOR_MOVEMENT) * 0.3
                    linear_vel = MAX_LINEAR_VEL * speed_factor
                linear_vel = -linear_vel  # Negate for MuJoCo convention
            else:
                self._was_aligned = False
                linear_vel = 0.0
        
        # Phase 2: Rotate to target direction
        elif self.target_direction is not None:
            angle_error = self._calculate_angle_error(self.target_direction, current_yaw)
            angle_error_abs = abs(angle_error)
            
            angular_vel = np.clip(-K_P_ANGULAR * angle_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
            linear_vel = 0.0
            
            if angle_error_abs <= DIRECTION_TOLERANCE:
                self.reached = True
                self.active = False
                return (0.0, 0.0)
        
        # Phase 3: Done
        else:
            self.reached = True
            self.active = False
            return (0.0, 0.0)
        
        return (linear_vel, angular_vel)
    
    def cancel(self):
        """Cancel current navigation."""
        self.active = False
        self.target_pos = None
        self.target_direction = None
        self.reached = False
        self._was_aligned = False
        self._position_reached = False
    
    def is_active(self):
        """Check if navigation is active."""
        return self.active
    
    def has_reached(self):
        """Check if target has been reached."""
        return self.reached
