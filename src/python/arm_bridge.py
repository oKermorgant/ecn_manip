#!/usr/bin/env python3

'''
Created on: 12 Sep 2012
Author: Olivier Kermorgant

Bridge to control arm position and velocity 

Subscribes:
- robot position or velocity command on topic /main_control/command

Publishes:
- robot joint positions commands on topic /joint_control/command (position control)
'''

# modules
# ROS stuff and multithreading
import rclpy
from rclpy.node import Node
import time,threading
from sensor_msgs.msg import JointState
# math
from pylab import arange, sign
# system
import sys,os

verbose = True

# low-level sampling time
T = 1./50

# init this node
rclpy.init(args=None)
node = Node('joint_control')

def now():
    s,ns = node.get_clock().now().seconds_nanoseconds()
    return s + ns*1e-9

# get joint properties
from robot_description import Arm
arm = Arm(node, T)
N = arm.dof
        
print(f"Initializing bridge with {N} joints")

def inJointLimits(q):
    '''
    Returns the position q projected inside the joint limits
    '''
    return [min(max(q[i],arm.lower[i]),arm.upper[i]) for i in range(N)]
    
    
class State:
    def __init__(self):
          self.qSet = [0.]*N
          self.cmdCount = 0
          self.t0 = now()

    def followPosition(self, qDes, cmdCountCur):
        '''
        Follows the joint position setpoint as long as :
        - the corresponding position lies inside the joint limits
        - no other command has been received
        '''
        # build trajectory to go from qSet to qDes with max velocity
        qTraj = []
        for i in range(N):
            if self.qSet[i] == qDes[i]:
                qTraj.append([])
            else:
                qTraj.append(list(arange(self.qSet[i],qDes[i],sign(qDes[i]-self.qSet[i])*arm.vel_max[i])[1:]))
        for i in range(N):
            if len(qTraj[i]) == 0:
                qTraj[i].append(qDes[i])
            elif qTraj[i][-1] != qDes[i]:
                qTraj[i].append(qDes[i])
        steps = max([len(t) for t in qTraj])
        for i in range(N):
            qTraj[i] += [qDes[i]] * (steps - len(qTraj[i]))

        # follow trajectory from qSet to qDes
        k = 0
        while (self.cmdCount == cmdCountCur) and rclpy.ok() and k < len(qTraj[0]):
            # running setpoint
            self.qSet = [qTraj[i][k] for i in range(N)]
            k = k+1
            time.sleep(T)

    def followVelocity(self, qDot, cmdCountCur):
        '''
        Follows the joint velocity setpoint as long as :
        - the corresponding position lies inside the joint limits
        - no other command has been received
        '''
        
        # ensures max velocity
        for i in range(N):
            if qDot[i] > arm.vel_max[i]:
                qDot[i] = arm.vel_max[i]
            elif -qDot[i] > arm.vel_max[i]:
                qDot[i] = -arm.vel_max[i]
        k = 0
        q0 = self.qSet
        while (self.cmdCount == cmdCountCur) and rclpy.ok():
            k = k+1
            # running setpoint
            self.qSet = inJointLimits([q0[i]+k*qDot[i] for i in range(N)])
            time.sleep(T)
    
  
    def readBridgeCommand(self, data):
        '''
        Execute command received on /robot/command topic
        '''
        if self.cmdCount == 0:
                    print('Switching to control mode')
        
        
        self.cmdCount += 1
        
        self.t0 = now()

        if len(data.velocity) == N:
            # read velocities
            thread=threading.Thread(group=None,target=self.followVelocity, name=None, args=([v*T for v in data.velocity], self.cmdCount), kwargs={})
            thread.start()
        elif len(data.position) == N:
                # read positions
                thread=threading.Thread(group=None,target=self.followPosition, name=None, args=(inJointLimits(data.position), self.cmdCount), kwargs={})
                thread.start()
        
    def readManualCommand(self, data):
        # erase autom setpoint only if not received for some time (2 s)
        if now() - self.t0 > .5:        
            if self.cmdCount != 0:
                print('Switching to manual mode')
            
            self.qSet = data.position
            self.cmdCount = 0
                    
state = State()

# subscribe to position and velocity command from main code
cmd_sub = node.create_subscription(JointState, '/main_control/command', state.readBridgeCommand, 10)
    
# subscribe to manual position setpoints
gui_sub = node.create_subscription(JointState, '/gui/position_manual', state.readManualCommand, 10)
            
# publish position command depending on the simulator type
cmdPub = node.create_publisher(JointState, '/joint_states', 10)

# create JointState object - used in rviz / ViSP
jointState = JointState()
jointState.position = [0.]*N
jointState.name = arm.names

def timer_update():
    jointState.position = [float(v) for v in state.qSet]
    jointState.header.stamp = node.get_clock().now().to_msg()
    cmdPub.publish(jointState)

timer = node.create_timer(T, timer_update)

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
