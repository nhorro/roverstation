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

"""


"""




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

        self.controls = [ 
            # Forward
            [
                self.keyboardWidget.fwd_interval_phase0,
                self.keyboardWidget.fwd_motor_a_phase0,
                self.keyboardWidget.fwd_motor_b_phase0,
                self.keyboardWidget.fwd_interval_phase1,
                self.keyboardWidget.fwd_motor_a_phase1,
                self.keyboardWidget.fwd_motor_b_phase1
            ],

            # Left
            [
                self.keyboardWidget.lft_interval_phase0,
                self.keyboardWidget.lft_motor_a_phase0,
                self.keyboardWidget.lft_motor_b_phase0,
                self.keyboardWidget.lft_interval_phase1,
                self.keyboardWidget.lft_motor_a_phase1,
                self.keyboardWidget.lft_motor_b_phase1
            ],

            # Right
            [
                self.keyboardWidget.rgt_interval_phase0,
                self.keyboardWidget.rgt_motor_a_phase0,
                self.keyboardWidget.rgt_motor_b_phase0,
                self.keyboardWidget.rgt_interval_phase1,
                self.keyboardWidget.rgt_motor_a_phase1,
                self.keyboardWidget.rgt_motor_b_phase1
            ]
        ]

        self.keyboardWidget.show()
        self.init_controls()


    def init_controls(self):
        for tab in self.controls:            
            tab[0].setValidator(QDoubleValidator())
            tab[0].textChanged.connect(self.update_params)
            tab[1].setValidator(QIntValidator())
            tab[1].textChanged.connect(self.update_params)
            tab[2].setValidator(QIntValidator())
            tab[2].textChanged.connect(self.update_params)
            tab[3].setValidator(QDoubleValidator())
            tab[3].textChanged.connect(self.update_params)
            tab[4].setValidator(QIntValidator())
            tab[4].textChanged.connect(self.update_params)
            tab[5].setValidator(QIntValidator())
            tab[5].textChanged.connect(self.update_params)
        self.update_params(0)


    def update_params(self,ignored):    
        self.interval_phase_0 = [0,0,0]
        self.motor_a_phase_0 = [0,0,0]
        self.motor_b_phase_0 = [0,0,0]
        self.interval_phase_1 = [0,0,0]
        self.motor_a_phase_1 = [0,0,0]
        self.motor_b_phase_1 = [0,0,0]
        for idx in range(len(self.controls)):
            self.interval_phase_0[idx] = float(self.controls[idx][0].text()) 
            self.motor_a_phase_0[idx] = float(self.controls[idx][1].text()) 
            self.motor_b_phase_0[idx] = float(self.controls[idx][2].text()) 
            self.interval_phase_1[idx] = float(self.controls[idx][3].text()) 
            self.motor_a_phase_1[idx] = float(self.controls[idx][4].text()) 
            self.motor_b_phase_1[idx] = float(self.controls[idx][5].text()) 
        
    def on_key_pressed(self, key):        

        if key in self.control_keys:
            try:
                if key == self.control_keys[0]:    
                    interval_phase_0 = self.interval_phase_0[0]
                    interval_phase_1 = self.interval_phase_1[0]
                    motor_a_phase_0_value = self.motor_a_phase_0[0]
                    motor_b_phase_0_value = self.motor_b_phase_0[0]
                    motor_a_phase_1_value = self.motor_a_phase_1[0]
                    motor_b_phase_1_value = self.motor_b_phase_1[0]
                elif key == self.control_keys[1]:
                    interval_phase_0 = self.interval_phase_0[0]
                    interval_phase_1 = self.interval_phase_1[0]
                    motor_a_phase_0_value = -self.motor_a_phase_0[0]
                    motor_b_phase_0_value = -self.motor_b_phase_0[0]
                    motor_a_phase_1_value = -self.motor_a_phase_1[0]
                    motor_b_phase_1_value = -self.motor_b_phase_1[0]
                elif key == self.control_keys[3]:
                    interval_phase_0 = self.interval_phase_0[1]
                    interval_phase_1 = self.interval_phase_1[1]
                    motor_a_phase_0_value = self.motor_a_phase_0[1]
                    motor_b_phase_0_value = -self.motor_b_phase_0[1]
                    motor_a_phase_1_value = self.motor_a_phase_1[1]
                    motor_b_phase_1_value = -self.motor_b_phase_1[1]
                elif key == self.control_keys[2]:
                    interval_phase_0 = self.interval_phase_0[2]
                    interval_phase_1 = self.interval_phase_1[2]
                    motor_a_phase_0_value = -self.motor_a_phase_0[2]
                    motor_b_phase_0_value = self.motor_b_phase_0[2]
                    motor_a_phase_1_value = -self.motor_a_phase_1[2]
                    motor_b_phase_1_value = self.motor_b_phase_1[2]
                
                ret = self.move_srv(
                    interval_phase_0, 
                    motor_a_phase_0_value,
                    motor_b_phase_0_value,
                    interval_phase_1,
                    motor_a_phase_1_value, 
                    motor_b_phase_1_value
                )
                return ret
            except rospy.ServiceException, e:
                rospy.logerror("Service call failed: %s"%e)
                    
if __name__ == '__main__':
    app = RoverCommanderApplication(sys.argv)
    sys.exit(app.exec_())
