#!/usr/bin/env python
import roslib; roslib.load_manifest('roverstation')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import time
import sys
import os

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal,QTimer, Qt
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QIntValidator, QDoubleValidator

from roverbridge.srv import move


DEFAULT_CONTROL_KEYS = [Qt.Key_W, Qt.Key_S, Qt.Key_A,Qt.Key_D ]

class KeyboardWidget(QWidget):
    keyPressed = pyqtSignal(int)
    keyReleased = pyqtSignal(int)

    def __init__(self):
        super(KeyboardWidget,self).__init__()    

        abspath=os.path.dirname(os.path.abspath(__file__))    
        uic.loadUi(abspath+'/rovercmd.ui', self) 

    def keyPressEvent(self, keyEvent):
        self.keyPressed.emit(keyEvent.key())

    def keyReleaseEvent(self, keyEvent):
        self.keyReleased.emit(keyEvent.key())

class RoverCommanderApplication(QApplication):
    def __init__(self, args,control_keys=DEFAULT_CONTROL_KEYS, vehicle_id="rover"):
        super(RoverCommanderApplication,self).__init__(args)        
        self.control_keys=control_keys
        rospy.init_node('rovercmd')

        rospy.wait_for_service('move')
        self.move_srv = rospy.ServiceProxy('move', move)
    
        self.keyboardWidget = KeyboardWidget()
        self.keyboardWidget.keyPressed.connect(self.on_key_pressed)

        self.keyboardWidget.interval_phase0.setValidator(QDoubleValidator())
        self.keyboardWidget.interval_phase0.textChanged.connect(self.update_params)
        self.keyboardWidget.motor_a_phase0.setValidator(QIntValidator())
        self.keyboardWidget.motor_a_phase0.textChanged.connect(self.update_params)
        self.keyboardWidget.motor_b_phase0.setValidator(QIntValidator())
        self.keyboardWidget.motor_b_phase0.textChanged.connect(self.update_params)
        self.keyboardWidget.interval_phase1.setValidator(QDoubleValidator())
        self.keyboardWidget.interval_phase1.textChanged.connect(self.update_params)
        self.keyboardWidget.motor_a_phase1.setValidator(QIntValidator())
        self.keyboardWidget.motor_a_phase1.textChanged.connect(self.update_params)
        self.keyboardWidget.motor_b_phase1.setValidator(QIntValidator())
        self.keyboardWidget.motor_b_phase1.textChanged.connect(self.update_params)

        self.keyboardWidget.show()

        self.interval_phase_0 = 0.5
        self.motor_a_phase_0 = 100
        self.motor_b_phase_0 = 100
        self.interval_phase_1 = 5
        self.motor_a_phase_1 = 80
        self.motor_b_phase_1 = 0

    def update_params(self,value):
        self.interval_phase_0 = float(self.keyboardWidget.interval_phase0.text())                
        self.motor_a_phase_0 = float(self.keyboardWidget.motor_a_phase0.text())                
        self.motor_b_phase_0 = float(self.keyboardWidget.motor_b_phase0.text())                
        self.interval_phase_1 = float(self.keyboardWidget.interval_phase1.text())                
        self.motor_a_phase_1 = float(self.keyboardWidget.motor_a_phase1.text())                
        self.motor_b_phase_1 = float(self.keyboardWidget.motor_b_phase1.text())                
        
    def on_key_pressed(self, key):        

        if key in self.control_keys:
            try:
                if key == self.control_keys[0]:                    
                    motor_a_phase_0_value = self.motor_a_phase_0
                    motor_b_phase_0_value = self.motor_b_phase_0
                    motor_a_phase_1_value = self.motor_a_phase_1
                    motor_b_phase_1_value = self.motor_b_phase_1
                elif key == self.control_keys[1]:
                    motor_a_phase_0_value = -self.motor_a_phase_0
                    motor_b_phase_0_value = -self.motor_b_phase_0
                    motor_a_phase_1_value = -self.motor_a_phase_1
                    motor_b_phase_1_value = -self.motor_b_phase_1
                elif key == self.control_keys[2]:
                    motor_a_phase_0_value = self.motor_a_phase_0
                    motor_b_phase_0_value = -self.motor_b_phase_0
                    motor_a_phase_1_value = self.motor_a_phase_1
                    motor_b_phase_1_value = -self.motor_b_phase_1
                elif key == self.control_keys[3]:
                    motor_a_phase_0_value = -self.motor_a_phase_0
                    motor_b_phase_0_value = self.motor_b_phase_0
                    motor_a_phase_1_value = -self.motor_a_phase_1
                    motor_b_phase_1_value = self.motor_b_phase_1
                
                ret = self.move_srv(
                    self.interval_phase_0, 
                    motor_a_phase_0_value,
                    motor_b_phase_0_value,
                    self.interval_phase_1,
                    motor_a_phase_1_value, 
                    motor_b_phase_1_value
                )
                return ret
            except rospy.ServiceException, e:
                rospy.logerror("Service call failed: %s"%e)
                    
if __name__ == '__main__':
    app = RoverCommanderApplication(sys.argv)
    sys.exit(app.exec_())
