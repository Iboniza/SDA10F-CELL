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
## Instantiate a `PlanningSceneInterface`_ object. This object is an interface to the world surroundign the robot:
scene = moveit_commander.PlanningSceneInterface()
## Instantiate a `MoveGroupCommander`_ object.
group_name = "arm_right"
group = moveit_commander.MoveGroupCommander(group_name)
# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)


# Copy class variables to local variables to make the web tutorials more clear.
# In practice, you should use the class variables directly unless you have a good
# reason not to.

timeout=5

## First, we will create a box in the planning scene at the location of the left finger:
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "arm_right_link_tool0"
box_pose.pose.orientation.w = 1.0
box_pose.pose.orientation.x = 0
box_pose.pose.orientation.y = 0
box_pose.pose.orientation.z = -0.45
box_name = "sensor"
scene.add_box(box_name, box_pose, size=(0.125, 0.10, 0.15))
##                                      anch. alt.  prof.
start = rospy.get_time()
seconds = rospy.get_time()
box_is_attached = False
box_is_known = True

while (seconds - start < timeout) and not rospy.is_shutdown():
	# Test if the box is in attached objects
	attached_objects = scene.get_attached_objects([box_name])
	is_attached = len(attached_objects.keys()) > 0
	# Test if the box is in the scene.
	# Note that attaching the box will remove it from known_objects
	is_known = box_name in scene.get_known_object_names()
	# Test if we are in the expected state
	if (box_is_attached == is_attached) and (box_is_known == is_known):
		break
	# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()
	# If we exited the while loop without returning then we timed out

"""
rospy.sleep(2)
moveit_commander.roscpp_shutdown()
"""
## Attaching Box to the Robot
##eef_link = "arm_right_link_7_t"
grasping_group = 'arm_right'
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)

"""
rospy.sleep(2)
moveit_commander.roscpp_shutdown()
"""

start = rospy.get_time()
seconds = rospy.get_time()
box_is_attached = True
box_is_known = False

while (seconds - start < timeout) and not rospy.is_shutdown():
	# Test if the box is in attached objects
	attached_objects = scene.get_attached_objects([box_name])
	is_attached = len(attached_objects.keys()) > 0
	# Test if the box is in the scene.
	# Note that attaching the box will remove it from known_objects
	is_known = box_name in scene.get_known_object_names()
	# Test if we are in the expected state
	if (box_is_attached == is_attached) and (box_is_known == is_known):
		break
	# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()
	# If we exited the while loop without returning then we timed out
	
print("--------------Sensor has been added--------------")
"""
planning_scene.robot_state.attached_collision_objects.push_back(box_name);
planning_scene.robot_state.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene);
"""

