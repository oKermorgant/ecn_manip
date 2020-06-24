#!/usr/bin/env python

from PyQt5 import QtWidgets
import sys
from threading import Thread
import manip_gui, joint_state_publisher, manip_plot
import rospy
import signal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

rb_order = ('manual', 'twist', 'p2p_direct','p2p_interp','p2p_line','p2p_vel')

class manipControl(QtWidgets.QMainWindow):
    
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.ui = manip_gui.Ui_main_ui()
        self.ui.setupUi(self)
        
        # spawn jsp on vertical layout
        self.jsp = joint_state_publisher.JointStatePublisher(self.ui.joint_manual)
        Thread(target=self.jsp.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        # publisher for mode / switch time / lambda-gain
        self.config_pub = rospy.Publisher('config', JointState, queue_size=5)
        self.config = {'mode': 0, 'switch_time': 2, 'lambda': 1}
                        
        # radio buttons
        current = 0
        for i,button in enumerate(rb_order):
            getattr(self.ui, 'rb_'+ button).clicked.connect(self.mode_update)
            if i == current:
                getattr(self.ui, 'rb_'+ button).setChecked(True)
        self.mode_update()
    
        # switching time slider
        ts_default = 2.
        self.ui.ts_slider.valueChanged.connect(self.ts_update)
        self.ui.ts_slider.setValue(int((ts_default-1)*100.))
        self.ts_update()
        
        # lambda slider
        lambda_default = 2.
        self.ui.gain_slider.valueChanged.connect(self.gain_update)
        self.ui.gain_slider.setValue(int((lambda_default-1)*100.))
        self.gain_update()
        
        # twist slider
        self.twist_pub = rospy.Publisher('twist_manual', Twist, queue_size=5)
        self.twist_msg = Twist()
        for la in ('v','w'):
            for ax in ('x','y','z'):
                getattr(self.ui, '{}{}_slider'.format(la, ax)).valueChanged.connect(self.vel_update)
                getattr(self.ui, '{}{}_slider'.format(la, ax)).setValue(50)
        self.vel_update()
        
        self.ui.twistCenter.clicked.connect(self.twist_center)
                
        # plotter
        self.plot = manip_plot.Plotter()
        self.ui.plot.addWidget(self.plot.canvas)
        Thread(target=self.plot.loop).start()
        
        # joint plotter
        self.plotJoints = manip_plot.JointPlotter()
        self.ui.plotJoints.addWidget(self.plotJoints.canvas)
        
        # publish slider values
        Thread(target=self.loop).start()
               
               
    def mode_update(self):
        # get clicked button
        for i,button in enumerate(rb_order):
            if getattr(self.ui, 'rb_'+ button).isChecked():
                self.config['mode'] = i
                if i in (0,1):
                    self.ui.spaceTabs.setCurrentIndex(i)
                break
            
    def ts_update(self):        
        sp = self.ui.ts_slider.value()/100.+1
        self.config['switch_time'] = sp
        self.ui.ts_display.setText(str(sp) + ' s')

    def gain_update(self):        
        sp = self.ui.gain_slider.value()/100.+1
        self.config['lambda'] = sp
        self.ui.gain_display.setText(str(sp))   
        
    def twist_center(self):
        for la in ('v','w'):
            for ax in ('x','y','z'):
                getattr(self.ui, '{}{}_slider'.format(la, ax)).setValue(50.)
        self.vel_update()
        
        
    def vel_update(self):
        for la in ('v','w'):
            for ax in ('x','y','z'):
                v = la+ax
                sp = (getattr(self.ui, v+'_slider').value()-50.)/500.
                if la == 'v':
                    u = 'm/s'
                    setattr(self.twist_msg.linear, ax, sp)
                else:
                    u = 'rad/s'
                    sp *= 5
                    setattr(self.twist_msg.angular, ax, sp)
                getattr(self.ui, 'twist_'+v).setText(v+': {} {}'.format(sp, u))
        
            
    def loop(self):
        
        while not rospy.is_shutdown():
            
            config_msg = JointState()
            config_msg.name = list(self.config.keys())
            config_msg.position = list(self.config.values())
            
            self.config_pub.publish(config_msg)
            self.twist_pub.publish(self.twist_msg)
            rospy.sleep(0.1)
        
            
             

if __name__ == '__main__':
    
    rospy.init_node('manip_control_gui')
    app = QtWidgets.QApplication(sys.argv)
    window = manipControl()
    window.show()
    sys.exit(app.exec_())
    
