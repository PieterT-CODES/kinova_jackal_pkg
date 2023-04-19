#! /usr/bin/env python

import rospy 
from kortex_driver.srv import Base_ClearFaults, Base_ClearFaultsRequest 

class ClearError(): 
    def __init__(self):
        rospy.init_node("clear_fault_client")
        rospy.wait_for_service("/my_gen3_lite/base/clear_faults") 
        self.service = rospy.ServiceProxy("/my_gen3_lite/base/clear_faults", Base_ClearFaults())
        self.object = Base_ClearFaultsRequest()
        
    def send_goal(self, input): 
        self.object.input = input 
        result = self.service(self.object)
        
if __name__ == "__main__": 
    clearerror = ClearError()
    clearerror.send_goal(input)            