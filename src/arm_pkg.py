#!/usr/bin/env python

import rospy
from kinova.srv import sequence, sequenceRequest 
from std_srvs.srv import Empty, EmptyRequest
from std_srvs.srv import SetBool, SetBoolRequest
from kinova_control import KinovaControl
from arduino import ArduinoControl
from enum import Enum
from kinova.srv import sequence, sequenceResponse


class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 
    CYCLE_START = 3
    CYCLE_STOP = 4

class Kinova_Final():
    def __init__(self):
        self.kinova_control = KinovaControl()
        self.arduino_control = ArduinoControl()
        self.start_kinova_service = rospy.Service('start_kinova_service', Empty, self.cb_start)
        self.stop_kinova_service = rospy.Service('stop_kinova_service', Empty, self.cb_stop)
        self.kinova_service_no_cycle = rospy.Service('sequence_kinova_service', sequence, self.cb_sequence)
        self.run = None

    def cb_start(self, req):
        self.run = Side.CYCLE_START.value
        return []
    
    def cb_stop(self, req): 
        self.run = Side.CYCLE_STOP.value
        return []
    
    def cb_sequence(self, req): 
        self.run = req.side
        return sequenceResponse()
    
    def doSomeWork(self): 
        if self.run == Side.CYCLE_START.value: 
            self.arduino_control.open()
            list = self.kinova_control.cykle_movement()
            for x in list: 
                self.kinova_control.realize_arm_move(x)
                
        if self.run == Side.CYCLE_STOP.value: 
            self.arduino_control.close()
            home = self.kinova_control.home_pose()
            self.kinova_control.realize_arm_move(home)
            rospy.loginfo("Kinova is in STOP state")
              
        if self.run == Side.RIGHT.value: 
            self.arduino_control.open()
            list = self.kinova_control.get_sequence_right()
            for x in list: 
                self.kinova_control.realize_arm_move(x)              
            self.arduino_control.close()
            home = self.kinova_control.home_pose()
            self.kinova_control.realize_arm_move(home)
            self.run = Side.CYCLE_STOP
            
        if self.run == Side.LEFT.value: 
            self.arduino_control.open()
            list = self.kinova_control.get_sequence_right()
            for x in list: 
                self.kinova_control.realize_arm_move(x)              
            self.arduino_control.close()
            self.run = Side.CYCLE_STOP
                
        
    def grab_gun(self):
        self.kinova_control.reach_gripper_position(0.5)
    
def main(): 
    kinova = Kinova_Final()
    kinova.grab_gun() 
    rospy.init_node('service_server_kinova_package')
    r = rospy.Rate(15.0)
    
    while not rospy.is_shutdown():
        kinova.doSomeWork()
        r.sleep() 
         
         
if __name__ == '__main__': 
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
        
        
        
        