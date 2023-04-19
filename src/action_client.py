#! /usr/bin/env python

import rospy 
import actionlib 
from kinova.msg import SequenceAction, SequenceGoal, SequenceResult, SequenceFeedback
import sys 
from enum import Enum

class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 
    RIGHT_AND_LEFT = 3 

class KinovaActionClient():
    def __init__(self):
        rospy.init_node('kinova_pkg_action_nodes')
        self.client = actionlib.SimpleActionClient('/kinova_pkg_action',SequenceAction)
        client.wait_for_server()
        self.goal = SequenceGoal()


    def send_goals(self, vagon_type, isLast, index, side): 
        goal.vagon_type = vagon_type
        goal.isLast = isLast 
        goal.index = index
        goal.side = side

        client.send_goal(goal)
        client.wait_for_result() 

        return client.get_result() 

if __name__ == '__main__': 
    kinova_action_client = KinovaActionClient()
    kinova_action_client.send_goals(1, True, 4, Side.RIGHT_AND_LEFT.value )

