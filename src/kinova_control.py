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
from arduino import ArduinoControl
from enum import Enum

class Side(Enum): 
    RIGHT = 1 
    LEFT = 2 

class KinovaControl():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
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
		self.stop = False

	def reach_gripper_position(self, relative_position):
		gripper_joint = self.robot.get_joint(self.gripper_joint_name)
		gripper_joint.move(relative_position, True)

	def set_arm_position(self, joint0, joint1, joint2, joint3, joint4, joint5):
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = joint0
		joint_goal[1] = joint1
		joint_goal[2] = joint2
		joint_goal[3] = joint3
		joint_goal[4] = joint4 
		joint_goal[5] = joint5    
		return joint_goal

	def get_current_joints(self):    
		return self.move_group.get_current_joint_values()	

	def realize_arm_move(self, joint_goal):    
		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop()

	def home_pose(self): 
		joint_goal = self.set_arm_position(0.031143272103495618, -2.1303216781305823, -2.6214698358462654, -1.6230741789931518, -0.7582536276707064, 1.5455836481228984)
		return joint_goal

	def cykle_pose_third_joint(self): 
		joint_goals = []
		joint_goal_left = self.set_arm_position(0.00606854517596356, 0.006065882014873481, -0.04563912423290528, -2.5255973681839583, -1.5704698232452525, -1.6174485175064683)
		joint_goal_right = self.set_arm_position(0.00606854517596356, 0.006061354641020346, -0.04627189130790921, 2.5225792077205713, -1.5721119283733955, -1.6174410606554162)
		
		joint_goals.append(joint_goal_left)
		joint_goals.append(joint_goal_right)
		return joint_goals
		
	def cykle_pose_last_joint(self): 
		joint_goals = []
		joint_goal_left = self.set_arm_position(0.00602220617299618, 0.00604031566840872, -0.046274021836780754, 1.131639997776394, -0.02404248568901668, 2.5875566098929785)
		joint_goal_right = self.set_arm_position(0.006024603017977253, 0.00604031566840872, -0.04627189130790921, 1.1316474546274464, -0.024035028837965378, -2.5086346299368074)
		
		joint_goals.append(joint_goal_left)
		joint_goals.append(joint_goal_right)
		return joint_goals

	def cykle_movement(self): 
		joint_goals = []
		joint_goal_1 = self.set_arm_position(0.030315561636698993, -0.6772898021067713, -0.9067690669167199, -1.5526671239902914, -0.09137944858723479, 2.533427860737118)
		joint_goal_2 = self.set_arm_position(0.030315561636698993, -0.6772898021067713, -0.9067690669167199, -1.5526671239902914, -0.09137944858723479, -2.599267061838275)
	
		joint_goals.append(joint_goal_1)
		joint_goals.append(joint_goal_2)
		return joint_goals
		
	def get_sequence_right(self):
		joint_goals = []
		
		joint_goals.append(self.set_arm_position(1.4590847085493428, -0.0891008479585631, -0.01631878589556912, -1.4297771533852384, -1.3379055526285573, 1.5177019494064228))
		joint_goals.append(self.set_arm_position(2.247980667722501, -0.08970272236492072, 0.382914627853773, -1.5700884585771533, -0.92546339250464, 1.5449807084521043))
		joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals

	def get_sequence_left(self):
		joint_goals = []
		
		joint_goals.append(self.set_arm_position(1.4590847085493428, -0.0891008479585631, -0.01631878589556912, -1.4297771533852384, -1.3379055526285573, 1.5177019494064228))
		joint_goals.append(self.set_arm_position(2.247980667722501, -0.08970272236492072, 0.382914627853773, -1.5700884585771533, -0.92546339250464, 1.5449807084521043))
		joint_goals.append(self.set_arm_position(1.4717523005904136, 0.17061142049429126, 1.9479723751317668, -1.5211310356260093, -0.18631794565525173, 1.4980020141908887))
		return joint_goals

if __name__ == "__main__":
	rospy.init_node('kinova_arduino')
	kinova = KinovaControl()
	arduino = ArduinoControl()
	while not rospy.is_shutdown():
		list = kinova.cykle_movement()
		for i in list:
			kinova.realize_arm_move(i)
	

