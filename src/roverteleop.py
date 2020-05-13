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

MIN_THROTTLE = -1.0
MAX_THROTTLE =  1.0
IDLE_THROTTLE = (MIN_THROTTLE + MAX_THROTTLE)/2.0
THROTTLE_ACCELERATION = 0.1
THROTTLE_TOLERANCE = THROTTLE_ACCELERATION/2.0 

MIN_ANGLE = -3.0
MAX_ANGLE =  3.0
IDLE_ANGLE = (MIN_ANGLE + MAX_ANGLE)/2.0
ANGLE_ACCELERATION = 0.1
ANGLE_TOLERANCE = ANGLE_ACCELERATION/2.0

REFRESH_INTERVAL_IN_MS = 100

MODE_THROTTLE_ENABLED = 0x01
MODE_ROTATING_ENABLED = 0x02

DEFAULT_CONTROL_KEYS = [Qt.Key_W, Qt.Key_S, Qt.Key_A,Qt.Key_D ]

class KeyboardWidget(QWidget):
    keyPressed = pyqtSignal(int)
    keyReleased = pyqtSignal(int)

    def __init__(self):
        super(KeyboardWidget,self).__init__()    

        abspath=os.path.dirname(os.path.abspath(__file__))    
        uic.loadUi(abspath+'/roverteleop.ui', self) # Load the .ui file

        # linear
        self.v_linear_min.valueChanged.connect(self.linear_min_update)
        self.v_linear_idle.valueChanged.connect(self.linear_idle_update)
        self.v_linear_acc.valueChanged.connect(self.linear_acc_update)

        # angular
        self.v_angular_max.valueChanged.connect(self.angular_max_update)
        self.v_linear_acc.valueChanged.connect(self.angular_acc_update)

    def keyPressEvent(self, keyEvent):
        self.keyPressed.emit(keyEvent.key())

    def keyReleaseEvent(self, keyEvent):
        self.keyReleased.emit(keyEvent.key())

    def update(self, v_linear, v_angular):
        self.v_linear.setValue(v_linear*1000.0)
        self.v_angular.setValue(v_angular*1000.0)
        self.v_linear_lcd.display(v_linear)
        self.v_angular_lcd.display(v_angular)

    def linear_min_update(self, value):
        global MIN_THROTTLE, MAX_THROTTLE, IDLE_THROTTLE
        MIN_THROTTLE = -value/1000.0
        MAX_THROTTLE = value/1000.0
        IDLE_THROTTLE = (MIN_THROTTLE + MAX_THROTTLE)/2.0
        self.v_linear_lcd.display(value)

    def linear_idle_update(self, value):
        global IDLE_THROTTLE
        IDLE_THROTTLE = value/1000.0
        self.v_linear_lcd.display(value)

    def linear_acc_update(self, value):
        global THROTTLE_ACCELERATION,THROTTLE_TOLERANCE
        THROTTLE_ACCELERATION = value/1000.0
        THROTTLE_TOLERANCE = THROTTLE_ACCELERATION/2.0 
        self.v_linear_lcd.display(value)

    def angular_max_update(self, value):
        global MIN_ANGLE, MAX_ANGLE
        MIN_ANGLE = -value/1000.0
        MAX_ANGLE = value/1000.0
        self.v_angular_lcd.display(value)

    def angular_acc_update(self, value):
        global ANGLE_ACCELERATION,ANGLE_TOLERANCE
        ANGLE_ACCELERATION = value/1000.0
        ANGLE_TOLERANCE = ANGLE_ACCELERATION/2.0
        self.v_angular_lcd.display(value)

class SteeringControllerApplication(QApplication):
    def __init__(self, args,control_keys=DEFAULT_CONTROL_KEYS, vehicle_id="rover"):
        super(SteeringControllerApplication,self).__init__(args)        

        self.control_keys=control_keys

        rospy.init_node('roverteleop')
        self.cmd_vel_topic = vehicle_id+'/cmd_vel'
        rospy.loginfo("Publishing to %s" % self.cmd_vel_topic)
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0  
        self.msg.angular.y = 0  
        self.msg.angular.z = 0

        self.timer = QTimer()
        self.timer.setInterval(REFRESH_INTERVAL_IN_MS)        
        self.timer.timeout.connect(self.on_timeout_accel)
        self.mode = 0 # 0 idle

        self.keyboardWidget = KeyboardWidget()
        self.keyboardWidget.keyPressed.connect(self.on_key_pressed)
        self.keyboardWidget.keyReleased.connect(self.on_key_released)
        self.keyboardWidget.cb_enabled.stateChanged.connect(self.enable_state_changed)

        self.keyboardWidget.show()
    
        self.throttle = 0.0        
        self.angle = 0.0 
        self.acc_linear = 0.0 
        self.acc_angular = 0.0
        self.enable_cmd_vel_pub = False

        self.timer.start()

    def on_timeout_accel(self):        
        if self.mode & MODE_THROTTLE_ENABLED:
            self.throttle += self.acc_linear
        else:
            if self.throttle > IDLE_THROTTLE:
                self.throttle-= THROTTLE_ACCELERATION
            elif self.throttle < IDLE_THROTTLE:
                self.throttle+= THROTTLE_ACCELERATION
        
        if self.mode & MODE_ROTATING_ENABLED:
            self.angle+=self.acc_angular
            self.throttle += self.acc_linear
        else:
            if self.angle > IDLE_ANGLE:
                self.angle -= ANGLE_ACCELERATION
            elif self.angle < IDLE_ANGLE:
                self.angle += ANGLE_ACCELERATION              
        
        if self.throttle < MIN_THROTTLE:
            self.throttle = MIN_THROTTLE
        elif self.throttle > MAX_THROTTLE:
            self.throttle = MAX_THROTTLE            

        if self.angle <  MIN_ANGLE:
            self.angle = MIN_ANGLE 
        elif self.angle >  MAX_ANGLE:
            self.angle = MAX_ANGLE        

        if (self.throttle > (IDLE_THROTTLE-THROTTLE_TOLERANCE)) and (self.throttle < (IDLE_THROTTLE+THROTTLE_TOLERANCE)):
            self.throttle = IDLE_THROTTLE

        if (self.angle > (IDLE_ANGLE-ANGLE_TOLERANCE)) and (self.angle < (IDLE_ANGLE+ANGLE_TOLERANCE)):
            self.angle = IDLE_ANGLE

        self.msg.linear.x = self.throttle
        self.msg.angular.z = self.angle

        self.keyboardWidget.update(self.throttle, self.angle)

        if self.enable_cmd_vel_pub:
            self.pub.publish(self.msg)
        else:
            rospy.logwarn("Publishing is disabled.")

    def on_key_pressed(self, key):        
        if key == self.control_keys[0]:
            self.mode|= MODE_THROTTLE_ENABLED
            self.acc_linear= THROTTLE_ACCELERATION
        elif key == self.control_keys[1]:
            self.mode|= MODE_THROTTLE_ENABLED
            self.acc_linear= -THROTTLE_ACCELERATION
        
        if key == self.control_keys[3]:
            self.mode |= MODE_ROTATING_ENABLED
            self.acc_angular = -ANGLE_ACCELERATION            
        elif key == self.control_keys[2]:
            self.mode |= MODE_ROTATING_ENABLED
            self.acc_angular = ANGLE_ACCELERATION
            self.acc_linear = THROTTLE_ACCELERATION
        
        if key == Qt.Key_Space:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.pub.publish(self.msg)
            print("Brake!")
       
    def on_key_released(self, key):        
        if (key == self.control_keys[0]) or (key == self.control_keys[1]):
            self.mode &= ~MODE_THROTTLE_ENABLED
            #self.acc_linear = 0
        
        if (key == self.control_keys[2]) or (key == self.control_keys[3]):
            self.mode &= ~MODE_ROTATING_ENABLED
            #self.acc_angular = 0

    def enable_state_changed(self, state):
        self.enable_cmd_vel_pub = state
        if state:
            rospy.loginfo("Control is enabled")
        else:
            rospy.loginfo("Control is disabled")


if __name__ == '__main__':
    app = SteeringControllerApplication(sys.argv)
    sys.exit(app.exec_())
