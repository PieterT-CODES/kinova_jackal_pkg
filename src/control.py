#!/usr/bin/env python

import sys
import copy
from syslog import LOG_INFO
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class KinovaControl():
	def __init__(self): 
		self.moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander("robot_description")
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "arm"
		gripper_group_name = "gripper"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
		display_trajectory_publisher = rospy.Publisher('/gen3_lite/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

		#GRIPER CONTROL
		gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
		self.gripper_joint_name = gripper_joint_names[0]

	def reach_gripper_position(self, relative_position):
		gripper_joint = self.robot.get_joint(gripper_joint_name)
		gripper_joint.move(relative_position, True)

	# def do_sequence(self):
	# 	joint_goal = self.move_group.get_current_joint_values()
	# 	joint_goal[0] = 1.4717693448213902
	# 	joint_goal[1] = 0.1715070415688849
	# 	joint_goal[2] = -0.16365870583642206
	# 	joint_goal[3] = -1.5849952364627624
	# 	joint_goal[4] = -0.18462737099526993
	# 	joint_goal[5] = 1.4980281131695714

	# 	self.move_group.go(joint_goal, wait=True)
	# 	self.move_group.stop()
	# 	rospy.loginfo("Reached and stop position 1")

	def set_arm_position(self, joint0, joint1, joint2, joint3, joint4, joint5):
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = joint0
		joint_goal[1] = joint1
		joint_goal[2] = joint2
		joint_goal[3] = joint3
		joint_goal[4] = joint4 
		joint_goal[5] = joint5    
		return joint_goal

	# def get_current_joints(self):    
    # 	return self.move_group.get_current_joint_values()	

	def realize_arm_move(self, joint_goal):         
		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop()

	def home_pose(self): 
		joint_goal = self.set_arm_position(0.030855917021876067, -0.6764423842479088, -2.5571678113263023, -1.675087312979053, -2.081171442316516, 1.5458318547364935)
		return joint_goal

	def get_ampeer_list(self, isLast): 
		joint_goals = []
		if isLast == True:
			joint_goals.append(self.home_pose())
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))

		if isLast == False:
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals


	def get_bmpeer_list(self, isLast): 
		joint_goals = []
		if isLast == True:
			joint_goals.append(self.home_pose())
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))

		if isLast == False:
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals


	def get_bdmpeer_list(self, isLast):
		joint_goals = [] 
		if isLast == True:
			joint_goals.append(self.home_pose())
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))

		if isLast == False:	
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals


	def get_bmz_list(self, isLast): 
		joint_goals = []
		if isLast == True:
			joint_goals.append(self.home_pose())
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))

		if isLast == False:
			joint_goals.append(self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4718764038972112, 0.17158161007940714, -1.9348275446233538, -1.5231209495925162, -0.1839839512759065, 1.4980281131695714))
			joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals


	def get_universal_list(self): 
		joint_goals = []
		joint_goal_0 = self.home_pose()
		joint_goal_left = self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206, -1.5849952364627624, -0.18462737099526993, 1.4980281131695714)
		joint_goal_right = self.set_arm_position(1.4717693448213902, 0.1715070415688849, -0.16365870583642206,  2.583668927333681, -0.18462737099526993, 1.4980281131695714)

		joint_goals.append(joint_goal_left)
		joint_goals.append(joint_goal_right)
		joint_goals.append(joint_goal_left)
		joint_goals.append(joint_goal_right)
		return joint_goals

	def cykle_movement(self): 
		joint_goals = []
		joint_goal_left = self.set_arm_position(0.00606854517596356, 0.006065882014873481, -0.04563912423290528, -2.5255973681839583, -1.5704698232452525, -1.6174485175064683)
		joint_goal_right = self.set_arm_position(0.00606854517596356, 0.006061354641020346, -0.04627189130790921, 2.5225792077205713, -1.5721119283733955, -1.6174410606554162)
		
		joint_goals.append(joint_goal_left)
		joint_goals.append(joint_goal_right)
		return joint_goals
		
		

if __name__ == "__main__":
	kinova = KinovaControl()
	stop = False
	while stop == False: 
		bmz_list = kinova.cykle_movement()
		for i in bmz_list:
			kinova.realize_arm_move(i)


	

