#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
robot = moveit_commander.RobotCommander()
## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
scene = moveit_commander.PlanningSceneInterface()
## Instantiate a `MoveGroupCommander`_ object.
group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)
## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

## Planning to a Joint Goal
joint_goal = group.get_current_joint_values()
joint_goal[0] = (1/180)*pi
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0
joint_goal[6] = 0

group.go(joint_goal, wait=True)
group.stop()

## Wait_for_scene_update
## Ensuring Collision Updates Are Receieved
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():

# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	
	
	
	seconds = rospy.get_time()
i=0
while (i<3) and not rospy.is_shutdown():
	## Planning to a Joint Goal
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = (1/180)*pi
	joint_goal[1] = (56/180)*pi
	joint_goal[2] = (68/180)*pi
	joint_goal[3] = (-58/180)*pi
	joint_goal[4] = (-121/180)*pi
	joint_goal[5] = (-28/180)*pi
	joint_goal[6] = (59/180)*pi
	joint_goal[7] = (0/180)*pi
	joint_goal[8] = (-8/180)*pi
	joint_goal[9] = (88/180)*pi
	joint_goal[10] = (-6/180)*pi
	joint_goal[11] = (-52/180)*pi
	joint_goal[12] = (13/180)*pi
	joint_goal[13] = (48/180)*pi
	joint_goal[14] = (0/180)*pi

	group.go(joint_goal, wait=True)
	group.stop()

	## Wait_for_scene_update
	## Ensuring Collision Updates Are Receieved
	start = rospy.get_time()
	seconds = rospy.get_time()
	timeout = 4
	while (seconds - start < timeout) and not rospy.is_shutdown():

	# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		
		
		
		seconds = rospy.get_time()

	## Planning to a Joint Goal
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = (1/180)*pi
	joint_goal[1] = (-2/180)*pi
	joint_goal[2] = (100/180)*pi
	joint_goal[3] = (-5/180)*pi
	joint_goal[4] = (-5/180)*pi
	joint_goal[5] = (170/180)*pi
	joint_goal[6] = (18/180)*pi
	joint_goal[7] = (0/180)*pi
	joint_goal[8] = (-10/180)*pi
	joint_goal[9] = (68/180)*pi
	joint_goal[10] = (-19/180)*pi
	joint_goal[11] = (-92/180)*pi
	joint_goal[12] = (-152/180)*pi
	joint_goal[13] = (48/180)*pi
	joint_goal[14] = (0/180)*pi

	group.go(joint_goal, wait=True)
	group.stop()

	## Wait_for_scene_update
	## Ensuring Collision Updates Are Receieved
	start = rospy.get_time()
	seconds = rospy.get_time()
	timeout = 4
	while (seconds - start < timeout) and not rospy.is_shutdown():

	# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		
		
		
		seconds = rospy.get_time()
	i=i+1

