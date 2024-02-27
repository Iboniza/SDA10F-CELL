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


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_left"
group = moveit_commander.MoveGroupCommander(group_name)


def ini_state():
	global STATE
	print("estado inicial")
	
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = (0/180)*pi
	joint_goal[1] = (0/180)*pi
	joint_goal[2] = (0/180)*pi
	joint_goal[3] = (0/180)*pi
	joint_goal[4] = (0/180)*pi
	joint_goal[5] = (0/180)*pi
	joint_goal[6] = (0/180)*pi
	group.go(joint_goal, wait=True)
	group.stop()
	
	STATE = "miling_state"
	
def miling_state():
	global STATE
	print("milling state")
	
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = (0/180)*pi
	joint_goal[1] = (0/180)*pi
	joint_goal[2] = (0/180)*pi
	joint_goal[3] = (0/180)*pi
	joint_goal[4] = (0/180)*pi
	joint_goal[5] = (30/180)*pi
	joint_goal[6] = (0/180)*pi
	group.go(joint_goal, wait=True)
	group.stop()
	
	STATE = "hola"
	

STATE = "ini_state"

while True:
	if STATE == "ini_state":
		ini_state()
	elif STATE == "miling_state":
		miling_state()
	else:
		print("finished")

	
