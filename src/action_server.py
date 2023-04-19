#! /usr/bin/env python

import rospy 
import actionlib
from enum import Enum
from kinova.msg import SequenceAction, SequenceResult, SequenceFeedback
from control import KinovaControl
from arduino import ArduinoControl

class VagonType(Enum): 
    AMPEER = 1 
    BMPEER = 2
    BDMPEER = 3 
    BMZ = 4

class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 
    RIGHT_AND_LEFT = 3 

    
class KinovaActionServer(object):
    _feedback = SequenceFeedback()
    _result =  SequenceResult()

    def __init__(self): 
        self._as = actionlib.SimpleActionServer("kinova_pkg_action", SequenceAction ,self.execute_cb, False)
        self._as.start()
        self.kinova = KinovaControl()
        self.arduino_rele = ArduinoControl()

    def execute_cb(self, goal):
        success = True

        self.vagon_type = goal.vagon_type #int
        self.isLast = goal.isLast #string
        self.index = goal.index #int
        self.side = goal.side #bool

        if self.vagon_type == VagonType.AMPEER.value:
            ampeer_list = self.kinova.get_ampeer_list(self.seat_sequence_type)
            for i in ampeer_list: 
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(ampeer_list) - 1):
                    self.arduino_rele.close()
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)

        if self.vagon_type == VagonType.BMPEER.value: 
            bmpeer_list = self.kinova.get_bmpeer_list(self.seat_sequence_type)
            for i in bmpeer_list:
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(bmpeer_list) - 1):
                    self.arduino_rele.close()
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)

        if self.vagon_type == VagonType.BDMPEER.value: 
            bdmpeer_list = self.kinova.get_bdmpeer_list(self.seat_sequence_type)
            for i in bdmpeer_list:
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(bdmpeer_list) - 1):
                    self.arduino_rele.close()
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)
     
        if self.vagon_type == VagonType.BMZ.value: 
            bmz_list = self.kinova.get_bmz_list(self.seat_sequence_type)
            for i in bmz_list:
                if i == 1:
                    self.arduino_rele.open()
                if i == (len(ampeer_list) - 1):
                    self.arduino_rele.close()
                self.kinova.realize_arm_move(i)
                self._feedback.joint_info = self.kinova.get_current_joints()
                self._as.publish_feedback(self._feedback)

        if success: 
            self._result.status = "Kinova Action successful"
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node("kinova_action_server")
    server = KinovaActionServer()
    rospy.spin()
