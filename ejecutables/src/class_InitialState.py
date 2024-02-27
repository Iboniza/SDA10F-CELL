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
rospy.init_node('move_group_python_interface_tutorial',
            anonymous=True)

## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
## the robot:
robot = moveit_commander.RobotCommander()

## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
## to the world surrounding the robot:
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to one group of joints.  In this case the group is the joints in the Panda
## arm so we set ``group_name = panda_arm``. If you are using a different robot,
## you should change this value to the name of your robot arm planning group.
## This interface can be used to plan and execute motions on the Panda:
group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)

## We create a `DisplayTrajectory`_ publisher which is used later to publish
## trajectories for RViz to visualize:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

## Getting Basic Information
## ^^^^^^^^^^^^^^^^^^^^^^^^^
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print ("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

## Initial position right arm
## Planning to a Joint Goal
joint_goal = group.get_current_joint_values()
joint_goal[0] = (0/180)*pi
joint_goal[1] = (90/180)*pi
joint_goal[2] = (-90/180)*pi
joint_goal[3] = (-90/180)*pi
joint_goal[4] = (-135/180)*pi
joint_goal[5] = (0/180)*pi
joint_goal[6] = (-45/180)*pi
joint_goal[7] = (0/180)*pi

joint_goal[8] = (90/180)*pi
joint_goal[9] = (-90/180)*pi
joint_goal[10] = (-90/180)*pi
joint_goal[11] = (-135/180)*pi
joint_goal[12] = (0/180)*pi
joint_goal[13] = (-45/180)*pi
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

