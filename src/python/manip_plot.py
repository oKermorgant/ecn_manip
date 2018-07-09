import pylab as pl
from matplotlib import pyplot as pp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import rospy
from tf import TransformListener
from std_msgs.msg import Float32MultiArray

class AxisFig:
    
    def __init__(self, fig, sp, linear = True):
        self.ax = fig.add_subplot(sp)
        self.line = []
        self.t0 = 0
        
        legend = ('x','y','z')
        if not linear:
            self.ax.set_ylabel('orientation [rad]')
            legend = ('$\\theta u_x$', '$\\theta u_y$', '$\\theta u_z$')
        else:
            self.ax.set_ylabel('position [m]')
        c = ('r','g','b')
        for i,l in enumerate(legend):
            self.line.append(self.ax.plot([], [], c[i], lw=2, label=l)[0])
        for i,l in enumerate(legend):
            self.line.append(self.ax.plot([], [], c[i] + '--d', lw=1, label=l + '${}^*$', markevery=10)[0])
        self.ax.legend(loc='center left')
            
        self.t = []
        self.data = [[] for i in xrange(6)]
        
    def update(self, cur, des = None):
        if self.t0 == 0:
            self.t0 = rospy.Time.now().to_sec()
        self.t.append(rospy.Time.now().to_sec() - self.t0)
        
        # add measurement
        for i in range(3):
            self.data[i].append(cur[i])
        
        # clean reference if not in control mode
        if des == None:
            for i in range(3,6):
                self.data[i] = []
        else:
            for i in range(3,6):
                self.data[i].append(des[i-3])
                
        # clean past measurements
        for idx in xrange(len(self.t)):
            if self.t[-1] - self.t[idx] < 10:
                break

        sync = len(self.data[0]) == len(self.data[3])
        self.t = self.t[idx:]
        
        for i in xrange(3):
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
        self.des_sub = rospy.Subscriber('desired_pose', Float32MultiArray, self.des_pose_cb)
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
                    tu = [t*tr[1][i]/s2 for i in xrange(3)]
                
                
                if len(self.des_pose):
                    d = self.lin_ax.update(tr[0], self.des_pose[:3])
                    self.ang_ax.update(tu, self.des_pose[3:])
                else:
                    d = self.lin_ax.update(tr[0])
                    self.ang_ax.update(tu)
                if d:
                    self.canvas.draw()
                
            rospy.sleep(0.1)
                
