from simple_launch import SimpleLauncher
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    sl = SimpleLauncher()
    
    manip_root = get_package_share_directory('ecn_manip')
    
    robots = os.listdir(manip_root + '/robots')
    robots = [f.split('.')[0] for f in robots if f.endswith('urdf')]
    
    sl.declare_arg('robot', default_value='turret', description=f'Robot model ({", ".join(robots)})')
    robot = sl.arg('robot')
    
    urdf = sl.name_join(robot, '.urdf')
    config = sl.name_join(robot, '.rviz')
    
    sl.robot_state_publisher('ecn_manip', urdf, 'robots')
    
    sl.node('rviz2', arguments=['-d', sl.find('ecn_manip', config, 'launch')])
    
    sl.node('ecn_manip', 'arm_bridge.py', 'simulation')
    
    with sl.group(ns='gui'):
        sl.node('ecn_manip', 'manip_control.py', 'gui')
    
    return sl.launch_description() 
