from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    robot = sl.declare_arg('robot', default_value='turret', description='Robot model (turret, kr16, ur10, rrrp, dh, mdh)')
    
    urdf = robot+'.urdf'
    config = robot+'.rviz'
    
    sl.robot_state_publisher('ecn_manip', urdf, 'robots')
    
    sl.rviz(sl.find('ecn_manip', config, 'launch'))
    
    sl.node('ecn_manip', 'arm_bridge.py', 'simulation')
    
    with sl.group(ns='gui'):
        sl.node('ecn_manip', 'manip_control.py', 'gui')
    
    return sl.launch_description()
