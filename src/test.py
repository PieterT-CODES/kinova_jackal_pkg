#!/usr/bin/env python
# coding=utf-8

import rospy
from moveit_msgs.msg import RobotTrajectory, JointTrajectory, JointTrajectoryPoint

# Inicializácia ROS uzla
rospy.init_node('example_node')

# Vytvorenie JointTrajectory správy
joint_traj = JointTrajectory()
joint_traj.header.stamp = rospy.Time.now()
joint_traj.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

# Vytvorenie JointTrajectoryPoint správy pre počiatočnú polohu
start_point = JointTrajectoryPoint()
start_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
start_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
start_point.time_from_start = rospy.Duration(0.0)
joint_traj.points.append(start_point)

# Vytvorenie JointTrajectoryPoint správy pre koncovú polohu
end_point = JointTrajectoryPoint()
end_point.positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
end_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_point.time_from_start = rospy.Duration(5.0)
joint_traj.points.append(end_point)

# Vytvorenie RobotTrajectory správy
robot_traj = RobotTrajectory()
robot_traj.joint_trajectory = joint_traj

# Vytvorenie správy pre pre-computed joint trajectory
pre_traj = rospy.wait_for_message('/robot/limb/right/follow_joint_trajectory/precomputed_joint_trajectory', RobotTrajectory)
pre_traj.joint_trajectory = robot_traj

# Publikovanie pre-computed joint trajectory správy
pub = rospy.Publisher('/robot/limb/right/follow_joint_trajectory/precomputed_joint_trajectory', RobotTrajectory, queue_size=10)
pub.publish(pre_traj)