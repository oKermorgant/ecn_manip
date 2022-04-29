import matplotlib
#matplotlib.use('TkAgg')

import pylab as pl
from matplotlib import pyplot as pp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
#from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvasRenderer
import rclpy
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class AxisFig:
    
    def __init__(self, fig, sp, linear = True):
        self.ax = fig.add_subplot(sp)
        self.line = []
        self.t0 = 0
        
        legend = ('x','y','z')
        if not linear:
            self.ax.set_ylabel('orientation [rad]')
            legend = ('\\theta u_x', '\\theta u_y', '\\theta u_z')
        else:
            self.ax.set_ylabel('position [m]')
        c = ('r','g','b')
        for i,l in enumerate(legend):
            self.line.append(self.ax.plot([], [], c[i], lw=2, label='${}$'.format(l))[0])
        for i,l in enumerate(legend):
            self.line.append(self.ax.plot([], [], c[i] + '--d', lw=1, label= '${}^*$'.format(l), markevery=10)[0])
        self.ax.legend(loc='center left')
            
        self.t = []
        self.data = [[] for i in range(6)]
        
    def update(self, t, cur, des = None):
        if self.t0 == 0:
            self.t0 = t
        self.t.append(t - self.t0)
        
        # add measurement
        for i in range(3):
            self.data[i].append(cur[i])
        
        # dipslay or clean reference if not in control mode
        if des:
            for i in range(3,6):
                self.data[i].append(des[i-3])
        else:
            for i in range(3,6):
                self.data[i] = []
                
        # clean past measurements
        for idx in range(len(self.t)):
            if self.t[-1] - self.t[idx] < 10:
                break

        sync = len(self.data[0]) == len(self.data[3])
        self.t = self.t[idx:]
        
        for i in range(3):
            self.data[i] = self.data[i][idx:]
            self.line[i].set_data(self.t, self.data[i])
            if sync:
                self.data[i+3] = self.data[i+3][idx:]
                self.line[i+3].set_data(self.t, self.data[i+3])
            
        # update limits
        if len(self.t) > 10:
            self.ax.set_xlim(self.t[0], self.t[-1])
            m = min(min(v) for v in self.data if len(v))
            M = max(max(v) for v in self.data if len(v))
            self.ax.set_ylim(m - 0.05*(M-m), M + 0.05*(M-m))
            return True
        return False


class Plotter:
    
    def __init__(self, node):            
        
        self.fig = pp.figure()
        
        self.lin_ax = AxisFig(self.fig, 211)
        self.ang_ax = AxisFig(self.fig, 212, False)
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        
        self.buff = Buffer()
        self.tl = TransformListener(self.buff, node)
        self.des_sub = node.create_subscription(Float32MultiArray, '/desired_pose', self.des_pose_cb, 3)
        self.des_pose = []
        
    def des_pose_cb(self, msg):
        self.des_pose = msg.data
        
    def publish(self, now):
                        
        # update TF
        if self.buff.can_transform('base_link', 'tool0', rclpy.time.Time()):
            tr = self.buff.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            
            d = False
            t = tr.transform.translation
            q = tr.transform.rotation
            
            # angle-axis from quaternion
            s2 = pl.sqrt(q.x**2 + q.y**2 + q.z**2)
            tu = [0,0,0]
            if s2 > 1e-6:
                theta = pl.arctan2(s2, q.w)*2
                if theta > pl.pi:
                    theta -= 2*pl.pi
                tu = [theta*qi/s2 for qi in (q.x,q.y,q.z)]
                                        
            if len(self.des_pose):
                d = self.lin_ax.update(now, [t.x,t.y,t.z], self.des_pose[:3])
                self.ang_ax.update(now, tu, self.des_pose[3:])
            else:
                d = self.lin_ax.update(now, [t.x,t.y,t.z])
                self.ang_ax.update(now, tu)
            if d:
                self.canvas.draw()
                
class JointPlotter:
    def __init__(self, node):
        
        self.node = node
        
        # init figure
        self.fig = pp.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_ylim(-0.05, 1.05)
        self.ax.set_yticks([0,1])
        self.ax.set_yticklabels(['lower limit','upper limit'])        
        self.t0 = 0  
        self.tjs = 0
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        
        # get joint names and limits
        self.n = self.init_urdf()

        # subscriber and message history
        self.js_sub = node.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.t = []
        self.data = [[] for i in range(self.n)]                
        
    def now(self):
        s,ns = self.node.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9
        
    def init_urdf(self):
        
        from robot_description import Arm
        robot = Arm(self.node)
        self.names = robot.names
        self.qmin = robot.lower
        self.qmax = robot.upper
        
        self.line = [self.ax.plot([], [], lw=2, label=name)[0] for name in self.names]
        self.line.append(self.ax.plot([], [], 'k--', lw=2)[0])
        self.line.append(self.ax.plot([], [], 'k--', lw=2)[0])
        self.ax.legend(loc='center left')
        
        return len(self.names)
                        
    def joint_callback(self, msg):
        
        t = self.now()
        if self.t0 == 0:
            self.t0 = t
        
        if t - self.tjs > 0.1:
            # perform update
            self.tjs = t
                     
            # append
            self.t.append(t-self.t0)
            for i,name in enumerate(msg.name):
                if name in self.names:
                    j = self.names.index(name)
                    self.data[j].append((msg.position[i] - self.qmin[j])/(self.qmax[j] - self.qmin[j]))
            
            # clean
            for idx in range(len(self.t)):
                if self.t[-1] - self.t[idx] < 10:
                    break
            self.t = self.t[idx:]
                        
            for i in range(self.n):
                self.data[i] = self.data[i][idx:]
                self.line[i].set_data(self.t, self.data[i])
            self.line[i+1].set_data([self.t[0],self.t[-1]], [0,0])
            self.line[i+2].set_data([self.t[0],self.t[-1]], [1,1])
                        
            # update time scrolling
            if len(self.t) > 10:                
                self.ax.set_xlim(self.t[0], self.t[-1])                
            
            self.canvas.draw()
                
