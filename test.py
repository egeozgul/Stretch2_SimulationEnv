from stretch_mujoco import StretchMujocoSimulator

if __name__ == "__main__":
    sim = StretchMujocoSimulator()
    sim.start(headless=False) # This will open a Mujoco-Viewer window
    
    # Poses
    sim.stow()
    sim.home()
    
    # Position Control 
    sim.move_to('lift', 1.0)
    sim.move_by('head_pan', -1.1)
    sim.move_by('base_translate', 0.1)

    sim.wait_until_at_setpoint('lift')
    sim.wait_while_is_moving('base_translate')
    
    # Base Velocity control
    sim.set_base_velocity(0.3, -0.1)
    
    # Get Joint Status
    from pprint import pprint
    pprint(sim.pull_status())
    """
    Output:
    {'time': 6.421999999999515,
     'base': {'x_vel': -3.293721562016785e-07,'theta_vel': -3.061556698064456e-05},
     'lift': {'pos': 0.5889703729548038, 'vel': 1.3548342274419937e-08},
     'arm': {'pos': 0.09806380391427844, 'vel': -0.0001650879063921366},
     'head_pan': {'pos': -4.968686850480367e-06, 'vel': 3.987855066304579e-08},
     'head_tilt': {'pos': -0.00451929555883404, 'vel': -2.2404905787897265e-09},
     'wrist_yaw': {'pos': 0.004738908190630005, 'vel': -5.8446467640096307e-05},
     'wrist_pitch': {'pos': -0.0033446975569971366,'vel': -4.3182498418896415e-06},
     'wrist_roll': {'pos': 0.0049449466225058416, 'vel': 1.27366845279872e-08},
     'gripper': {'pos': -0.00044654737698173895, 'vel': -8.808287459130369e-07}}
    """
    
    # Get Camera Frames
    camera_data = sim.pull_camera_data()
    pprint(camera_data)
    """
    Output:
    {'time': 80.89999999999286,
     'cam_d405_rgb': array([[...]]),
     'cam_d405_depth': array([[...]]),
     'cam_d435i_rgb': array([[...]]),
     'cam_d435i_depth': array([[...]]),
     'cam_nav_rgb': array([[...]]),
     'cam_d405_K': array([[...]]),
     'cam_d435i_K': array([[...]])}
    """
    
    # Kills simulation process
    sim.stop()
