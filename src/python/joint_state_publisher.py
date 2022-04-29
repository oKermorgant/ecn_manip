#!/usr/bin/env python

import rclpy
import random

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget

import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread
import sys
import signal
import math

RANGE = 10000

class JointStatePublisher:

    def __init__(self, joint_manual, node):
        
        self.node = node
        from robot_description import Arm
        self.free_joints = {}
        robot = Arm(node)
        for i in range(robot.dof):
            self.free_joints[robot.names[i]] = {'min': robot.lower[i],
                                                'max': robot.upper[i]}
            for key in ('zero', 'position'):
                self.free_joints[robot.names[i]][key] = 0.5*(robot.lower[i]+robot.upper[i])

        num_rows = 0
        self.gui = JointStatePublisherGui("Joint State Publisher", self, num_rows)
        joint_manual.addWidget(self.gui)

        self.msg = JointState()
        self.msg.name = robot.names
        self.msg.position = [0. for _ in range(robot.dof)]
        self.pub = node.create_publisher(JointState, 'position_manual', 5)
        
    def publish(self):
        self.msg.header.stamp  = self.node.get_clock().now().to_msg()
        
        for i,name in enumerate(self.msg.name):
            self.msg.position[i] = self.free_joints[name]['position']
        
        self.pub.publish(self.msg)

class JointStatePublisherGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self, title, jsp, num_rows=0):
        super(JointStatePublisherGui, self).__init__()
        self.jsp = jsp
        self.joint_map = {}
        self.vlayout = QVBoxLayout(self)
        self.scrollable = QWidget()
        self.gridlayout = QGridLayout()
        self.scroll = QScrollArea()
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)

        font = QFont("Helvetica", 9, QFont.Bold)

        ### Generate sliders ###
        sliders = []
        for name in self.jsp.free_joints:
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            joint_layout = QVBoxLayout()
            row_layout = QHBoxLayout()

            label = QLabel(name)
            label.setFont(font)
            row_layout.addWidget(label)
            display = QLineEdit("0.00")
            display.setAlignment(Qt.AlignRight)
            display.setFont(font)
            display.setReadOnly(True)
            row_layout.addWidget(display)

            joint_layout.addLayout(row_layout)

            slider = QSlider(Qt.Horizontal)

            slider.setFont(font)
            slider.setRange(0, RANGE)
            slider.setValue(RANGE/2)

            joint_layout.addWidget(slider)

            self.joint_map[name] = {'slidervalue': 0, 'display': display,
                                    'slider': slider, 'joint': joint}
            # Connect to the signal provided by QSignal
            slider.valueChanged.connect(self.onValueChanged)
            sliders.append(joint_layout)

        # Determine number of rows to be used in grid
        self.num_rows = num_rows
        # if desired num of rows wasn't set, default behaviour is a vertical layout
        if self.num_rows == 0:
            self.num_rows = len(sliders)  # equals VBoxLayout
        # Generate positions in grid and place sliders there
        self.positions = self.generate_grid_positions(len(sliders), self.num_rows)
        for item, pos in zip(sliders, self.positions):
            self.gridlayout.addLayout(item, *pos)

        # Set zero positions read from parameters
        self.center()

        # Synchronize slider and displayed value
        self.sliderUpdate(None)

        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        self.scrollable.setLayout(self.gridlayout)
        self.scroll.setWidget(self.scrollable)
        self.vlayout.addWidget(self.scroll)

        # Buttons for randomizing and centering sliders and
        # Spinbox for on-the-fly selecting number of rows
        self.ctrbutton = QPushButton('Center', self)
        self.ctrbutton.clicked.connect(self.center_event)
        self.vlayout.addWidget(self.ctrbutton)
        self.setLayout(self.vlayout)

    @pyqtSlot(int)
    def onValueChanged(self, event):
        # A slider value was changed, but we need to change the joint_info metadata.
        for name, joint_info in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].value()
            joint = joint_info['joint']
            joint['position'] = self.sliderToValue(joint_info['slidervalue'], joint)
            joint_info['display'].setText("%.2f" % joint['position'])

    @pyqtSlot()
    def updateSliders(self):
        self.update_sliders()

    def update_sliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].setValue(joint_info['slidervalue'])

    def center_event(self, event):
        self.center()

    def center(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(self.valueToSlider(joint['zero'], joint))

    def generate_grid_positions(self, num_items, num_rows):
        if num_rows==0:
          return []
        positions = [(y, x) for x in range(int((math.ceil(float(num_items) / num_rows)))) for y in range(num_rows)]
        positions = positions[:num_items]
        return positions

    def sliderUpdate(self, event):
        for name, joint_info in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].value()
        self.update_sliders()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue

