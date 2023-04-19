#! /usr/bin/env python

import rospy 
from kortex_driver.srv import GetTrajectoryErrorReport 

class ErrorReportClient(): 

    def __init__(): 
        rospy.init_node("just_test")
        rospy.wait_for_service('/gen3_lite/base/get_trajectory_error_report')
        self.service = rospy.ServiceProxy(name, service_class)
        self.object = Base_ClearFaultsRequest()

    def send_goal(self, input): 
        self.object.input = input
        result = self.service(self.object)
        
        
if __name__ == "__main__": 
    kinova_handle_error = HandleErrorClient()
    kinova_handle_error.send_goal(input)