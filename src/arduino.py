#!/usr/bin/env python 

import rospy
from std_msgs.msg import UInt16

class ArduinoControl():
    def __init__(self): 
        self.pub = rospy.Publisher("/set_relay", UInt16, queue_size=0.5)
        self.pub_msg = UInt16()

    def open(self):
        self.pub_msg.data = 1
        self.pub.publish(self.pub_msg)

    def close(self): 
        self.pub_msg.data = 0
        self.pub.publish(self.pub_msg)  
               
