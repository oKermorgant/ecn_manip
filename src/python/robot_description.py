#!/usr/bin/env python3

class Arm:
    def __init__(self, node, T=1.):
        # get model, parse joints
        from rcl_interfaces.srv import GetParameters
        from urdf_parser_py.urdf import URDF
        import rclpy
        client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        client.wait_for_service()
        req = GetParameters.Request()
        req.names = ['robot_description']
        res = client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(node)
            if res.done():
                urdf = URDF.from_xml_string(res.result().values[0].string_value)
                break
            
        joints = [j for j in urdf.joints if j.type not in ('mimic', 'fixed')]
        
        self.dof = len(joints)
        self.names = [j.name for j in joints]
        self.lower = [j.limit.lower for j in joints]
        self.upper = [j.limit.upper for j in joints]
        self.vel_max = [T*j.limit.velocity for j in joints]
