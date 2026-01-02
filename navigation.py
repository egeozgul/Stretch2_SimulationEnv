"""Navigation controller for moving robot to target positions."""

import math
import numpy as np

# Navigation parameters
POSITION_TOLERANCE = 0.15
ALIGNMENT_THRESHOLD = math.radians(10)
ALIGNMENT_HYSTERESIS = math.radians(2.0)
DISTANCE_THRESHOLD = 0.7
CLOSE_SPEED_FACTOR = 0.7
HYSTERESIS_SPEED_FACTOR = 0.95
HANDLING_180_SPEED_FACTOR = 0.8
MAX_LINEAR_VEL = 2.5
MAX_ANGULAR_VEL = 2.0
K_P_LINEAR = 4.0
K_P_ANGULAR = 2.0
_180_DEG_THRESHOLD = math.radians(178.0)


class NavigationController:
    """Simple proportional controller for navigating to target positions."""
    
    def __init__(self):
        self.target_pos = None
        self.active = False
        self.reached = False
    
    def set_target(self, target_pos):
        """Set target position [x, y, z]."""
        self.target_pos = np.array(target_pos[:2])  # Only use x, y
        self.active = True
        self.reached = False
        self._was_aligned = False  # Reset hysteresis state for new target
    
    def get_control(self, current_pos, current_quat):
        """
        Get control velocities to reach target.
        
        Args:
            current_pos: [x, y, z] current position
            current_quat: [w, x, y, z] current orientation quaternion
        
        Returns:
            (linear_vel, angular_vel) tuple
        """
        if not self.active or self.target_pos is None:
            return (0.0, 0.0)
        
        # Extract 2D position (x, y)
        current_2d = np.array(current_pos[:2])
        
        diff = self.target_pos - current_2d
        distance = np.linalg.norm(diff)
        
        if distance < POSITION_TOLERANCE:
            self.reached = True
            self.active = False
            return (0.0, 0.0)
        
        if distance > 100.0:
            self.active = False
            return (0.0, 0.0)
        
        desired_angle = math.atan2(diff[1], diff[0])
        w, x, y, z = current_quat if len(current_quat) == 4 else (1.0, 0.0, 0.0, 0.0)
        current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        angle_error = math.atan2(math.sin(desired_angle - current_yaw),
                                math.cos(desired_angle - current_yaw))
        
        # Handle 180° ambiguity to prevent oscillation
        self._at_180 = abs(angle_error) >= _180_DEG_THRESHOLD
        if self._at_180:
            angle_error = math.pi
        
        angle_error_abs = abs(angle_error)
        angular_vel = np.clip(K_P_ANGULAR * angle_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        # Turn-then-move strategy with hysteresis
        self._was_aligned = getattr(self, '_was_aligned', False)
        is_aligned = angle_error_abs <= ALIGNMENT_THRESHOLD
        is_aligned_with_hyst = angle_error_abs <= (ALIGNMENT_THRESHOLD + ALIGNMENT_HYSTERESIS)
        can_move = is_aligned or (self._was_aligned and is_aligned_with_hyst) or self._at_180
        
        if can_move:
            self._was_aligned = True
            if distance >= DISTANCE_THRESHOLD:
                linear_vel = MAX_LINEAR_VEL
            else:
                angle_factor = (HANDLING_180_SPEED_FACTOR if self._at_180 else
                              HYSTERESIS_SPEED_FACTOR if angle_error_abs > ALIGNMENT_THRESHOLD else 1.0)
                linear_vel = min(K_P_LINEAR * distance * CLOSE_SPEED_FACTOR * angle_factor, MAX_LINEAR_VEL)
        else:
            self._was_aligned = False
            linear_vel = 0.0
        
        return (linear_vel, angular_vel)
    
    def cancel(self):
        """Cancel current navigation."""
        self.active = False
        self.target_pos = None
        self.reached = False
    
    def is_active(self):
        """Check if navigation is active."""
        return self.active
    
    def has_reached(self):
        """Check if target has been reached."""
        return self.reached

