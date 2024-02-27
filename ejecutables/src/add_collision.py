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
p.pose.position.x = -1
p.pose.position.y = -1
p.pose.position.z = 0.43
path = roslib.packages.get_pkg_dir('motoman_sda10f_support')+'/meshes/sda10f/collision/base.stl'
scene.add_mesh("mesh_example",p, path)

p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = -1.
p.pose.position.y = -1.
p.pose.position.z = 0.43
scene.add_box("box_example", p, (1, 1, 0.85)) #size of the box

rospy.sleep(2)

print(scene.get_known_object_names())

moveit_commander.roscpp_shutdown()
