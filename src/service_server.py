#! /usr/bin/env python


import rospy
from kinova.srv import sequence, sequenceResponse
from enum import Enum
from control import KinovaControl
from arduino import ArduinoControl
# from kortex_driver.srv import GetTrajectoryErrorReport 
# from kortex_driver.srv import Base_ClearFaults 

class VagonType(Enum): 
    AMPEER = 1 
    BMPEER = 2
    BDMPEER = 3
    BMZ = 4
    UNIVERSAL = 5
    
class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 
    

class KinovaService():
    def __init__(self):
        self.kinova = KinovaControl()
        self.my_service = rospy.Service('/kinova_service_server', sequence , self.my_callback) 
        self.arduino_rele = ArduinoControl()


    def my_callback(self, request):
        rospy.loginfo("Kinova service action has been called with:")
        # self.vagon_type = request.vagon_type #int
        # self.isLast = request.isLast #string
        # self.index = request.index #int
        self.side = request.side
        response = ''

        sequence_list = self.kinova.get_sequence(self.side)
        for i in sequence_list: 
            self.arduino_rele.open()
            self.kinova.realize_arm_move(i)
        response = "Bmpeer kinova sequence done careffuly"

        return sequenceResponse(response) # EmptyResponse


if __name__ == "__main__":
    rospy.init_node('kinova_service_server_node') 
    kinova_service = KinovaService()
    rospy.spin()