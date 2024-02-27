#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    


p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.35 +0.25	  ##Distancia profundidad al robot + mitad prof. Controladora
p.pose.position.y = 0  	  ##Distancia al borde paralelo + mitad del tamaño de la Controlador
p.pose.position.z = 0.7 + (0.15/2)
controladora = "Part"
scene.add_box(controladora, p, (0.15, 0.15, 0.15)) #size of the box
##                               prof. anch.  alt.


timeout = 10
start = rospy.get_time()
seconds = rospy.get_time()
box_is_attached = False
box_is_known = True

while (seconds - start < timeout) and not rospy.is_shutdown():
	# Test if the box is in attached objects
	attached_objects = scene.get_attached_objects([controladora])
	is_attached = len(attached_objects.keys()) > 0
	# Test if the box is in the scene.
	# Note that attaching the box will remove it from known_objects
	is_known = controladora in scene.get_known_object_names()
	# Test if we are in the expected state
	if (box_is_attached == is_attached) and (box_is_known == is_known):
		break
	# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()
	# If we exited the while loop without returning then we timed out

print(scene.get_known_object_names())

moveit_commander.roscpp_shutdown()

print("--------------Part has been added--------------")
