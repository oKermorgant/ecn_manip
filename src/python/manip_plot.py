import matplotlib
#matplotlib.use('TkAgg')

import pylab as pl
from matplotlib import pyplot as pp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
#from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvasRenderer
import rospy
from tf import TransformListener
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import xml.dom.minidom

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
        
    def update(self, cur, des = None):
        if self.t0 == 0:
            self.t0 = rospy.Time.now().to_sec()
        self.t.append(rospy.Time.now().to_sec() - self.t0)
        
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
    
    def __init__(self):            
        
        self.fig = pp.figure()
        
        self.lin_ax = AxisFig(self.fig, 211)
        self.ang_ax = AxisFig(self.fig, 212, False)
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        
        self.tl = TransformListener()
        self.des_sub = rospy.Subscriber('/desired_pose', Float32MultiArray, self.des_pose_cb)
        self.des_pose = []
        
    def des_pose_cb(self, msg):
        self.des_pose = msg.data
        
    def loop(self):
        
        while not rospy.is_shutdown():
                        
            # update TF
            if self.tl.canTransform('base_link', 'tool0', rospy.Time(0)):
                tr = self.tl.lookupTransform('base_link', 'tool0', rospy.Time(0))                
                d = False
                
                # angle-axis from quaternion
                s2 = pl.sqrt(tr[1][0]**2 + tr[1][1]**2 + tr[1][2]**2)
                tu = [0,0,0]
                if s2 > 1e-6:
                    t = pl.arctan2(s2, tr[1][3])*2
                    if t > pl.pi:
                        t -= 2*pl.pi
                    tu = [t*tr[1][i]/s2 for i in range(3)]
                
                
                if len(self.des_pose):
                    d = self.lin_ax.update(tr[0], self.des_pose[:3])
                    self.ang_ax.update(tu, self.des_pose[3:])
                else:
                    d = self.lin_ax.update(tr[0])
                    self.ang_ax.update(tu)
                if d:
                    self.canvas.draw()
                
            rospy.sleep(0.1)
                
class JointPlotter:
    def __init__(self, pub = False):
        
        # init figure
        self.fig = pp.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_ylim(-0.05, 1.05)
        self.ax.set_yticks([0,1])
        self.ax.set_yticklabels(['lower limit','upper limit'])
        self.line = []
        self.t0 = 0  
        self.tjs = 0
        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        
        # get joint names and limits
        self.n = self.init_urdf()

        # subscriber and message history
        self.js_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.t = []
        self.data = [[] for i in range(self.n)]
        
        self.pub = None
        if pub:
            self.pub = rospy.Publisher('joints', Image, queue_size=10)
        
    def init_urdf(self):
 
        robot = xml.dom.minidom.parseString(rospy.get_param('/robot_description'))        
        robot = robot.getElementsByTagName('robot')[0]
        self.names = []
        self.qmin = []
        self.qmax = []
                
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                limit = child.getElementsByTagName('limit')[0]
                self.names.append(name)
                self.qmin.append(float(limit.getAttribute('lower')))
                self.qmax.append(float(limit.getAttribute('upper')))                
                self.line.append(self.ax.plot([], [], lw=2, label=name)[0])
                
        self.line.append(self.ax.plot([], [], 'k--', lw=2)[0])
        self.line.append(self.ax.plot([], [], 'k--', lw=2)[0])
        self.ax.legend(loc='center left')
        
        return len(self.names)
                        
    def joint_callback(self, msg):
        
        t = rospy.Time.now().to_sec()
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
            
            if self.pub:
                # publish plot as an image - used to do videos
                w,h = self.canvas.get_width_height()
                im = pl.fromstring(self.canvas.tostring_rgb(), dtype='uint8').reshape(h,w,3)
                im_msg = CvBridge().cv2_to_imgmsg(im)
                self.pub.publish(im_msg)
                
