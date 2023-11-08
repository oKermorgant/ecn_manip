from simple_launch import SimpleLauncher
import yaml
import sympy as sp
from sympy.parsing.sympy_parser import parse_expr
import numpy as np
from urdf_parser_py import urdf
from scipy.spatial.transform import Rotation
import os


robot = urdf.URDF()
robot.name = 'test'


def add_link(name):
    link = urdf.Link()
    link.name = name
    robot.add_link(link)


def add_joint(name, jtype, parent, child, axis=None, xyz=None, rpy=None):
    joint = urdf.Joint()
    joint.name = name
    joint.type = jtype
    joint.parent = parent
    joint.child = child
    joint.axis = axis
    if jtype != 'fixed':
        lim = urdf.JointLimit()
        lim.effort = lim.velocity = 0
        if jtype == 'prismatic':
            lim.lower = -0.1
            lim.upper = 0.1
        else:
            lim.lower = -np.pi
            lim.upper = np.pi
        joint.limit = lim
    if xyz or rpy:
        joint.origin = urdf.Pose(xyz, rpy)

    robot.add_joint(joint)


def Homogeneous(t, tu):
    Rx = Rotation.from_rotvec(tu)
    Rt = np.hstack((Rx.as_matrix(),np.array(t).reshape(3,1)))
    return np.vstack((Rt, [0,0,0,1]))


class Joint:
    def __init__(self, line, notation, constants):

        self.dir = 0
        self.p = False

        for key in notation:

            val = parse_expr(str(line[notation[key]]), constants)

            try:
                val = float(val)
            except:
                # joint value
                q = val.free_symbols.pop()

                self.dir = sp.diff(val,q)
                val = val.subs(q, 0.)
                self.p = key == 'r'

            setattr(self, key, val)

    def update(self, j, prev):

        add_link(f'link{j}')

        # build full offset matrix
        Mx = Homogeneous([self.a, 0, 0], [self.alpha, 0, 0])
        Mz = Homogeneous([0, 0, self.r], [0, 0, self.theta])
        M = np.dot(Mx,Mz)
        R = Rotation.from_matrix(M[:3,:3])

        jtype = 'fixed'
        if self.dir:
            jtype = 'prismatic' if self.p else 'revolute'

        add_joint(f'joint{j}', jtype, prev, f'link{j}',
                  axis=[0,0,self.dir],
                  xyz=M[:3,3].tolist(),
                  rpy=R.as_euler('xyz'))

        return f'link{j}'


def load_yaml(mdh):
    with open(mdh) as f:
        robot = yaml.safe_load(f)

    notation = {'alpha': 0, 'a': 1, 'theta': 2, 'r': 3}

    # get ordering
    if 'notation' in robot:
        for key in notation:
            notation[key] = robot['notation'].index(key)

    # get constant
    constants = {'pi': np.pi}
    if 'constants' in robot:
        constants.update(robot['constants'])

    return dict([(key, Joint(line, notation, constants)) for key,line in robot['joint'].items()])


def build_robot(mdh):
    joints = load_yaml(mdh)

    prev = 'link0'
    add_link(prev)

    for j,joint in joints.items():
        prev = joint.update(j, prev)

    return robot


sl = SimpleLauncher()
sl.declare_arg('mdh', '',
               description = 'Path to a MDH parameter file including numerical values')


def launch_setup():

    mdh = sl.arg('mdh')

    if not os.path.exists(mdh):
        print(f'Could not find file "{mdh}"')
        return sl.launch_description()

    xml = build_robot(mdh).to_xml_string()

    with sl.group(ns = 'test'):

        sl.node('robot_state_publisher', parameters = {'robot_description': xml})

        sl.rviz(sl.find('ecn_manip', 'test.rviz'))

        sl.joint_state_publisher()

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
