#! /usr/bin/env python

import rospy
from kinova.srv import sequence, sequenceRequest 
import sys
from enum import Enum

class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 
    RIGHT_AND_LEFT = 3 

class KinovaServiceClient():
    def __init__(self):
        rospy.init_node('service_client')
        rospy.wait_for_service('/kinova_service_client')        
        self.service = rospy.ServiceProxy('/kinova_service_server', sequence)        
        self.object = sequenceRequest()

    def send_goal(self, vagon_type, isLast, index, side):
        self.object.vagon_type = vagon_type #int 
        self.object.isLast = isLast #bool
        self.object.index = index #int
        self.object.side = side #int
    
        result = self.service(self.object)

if __name__ == "__main__":
    kinovaserviceclient = KinovaServiceClient()
    kinovaserviceclient.send_goal(1, False, 4, Side.LEFT.value)
