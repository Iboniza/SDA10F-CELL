#!/usr/bin/env python3

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


## Configuracion
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

group_name = "arm_left"
group = moveit_commander.MoveGroupCommander(group_name)
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

## Cartesian Paths
waypoints = []
scale = 1
wpose = group.get_current_pose().pose

wpose.position.x += scale * 0.25  # Move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))
wpose.position.y -= scale * 0.26  # Move left/right in (y)
waypoints.append(copy.deepcopy(wpose))
wpose.position.z -= scale * 0.07  # Move up/down in (z)
waypoints.append(copy.deepcopy(wpose))


wpose.position.x += scale * 0.05  # Move left/right in (y)
waypoints.append(copy.deepcopy(wpose))

r = 0.05
prevy = 0
prevx = 0.05
for theta in range(0, 360):
	wpose.position.y -= prevy - r*math.sin(math.radians(theta))
	wpose.position.x -= prevx - r*math.cos(math.radians(theta))
	prevy = r*math.sin(math.radians(theta)) 
	prevx = r*math.cos(math.radians(theta))
	waypoints.append(copy.deepcopy(wpose))

wpose.position.z+= scale * 0.07  # Move up/down in (z)
waypoints.append(copy.deepcopy(wpose))
wpose.position.y += scale * 0.26  # Move left/right in (y)
waypoints.append(copy.deepcopy(wpose))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)     # jump_threshold

## Displaying a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);

## Executing a Plan
group.execute(plan, wait=True)

group_name = "arm_left"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()
joint_goal[0] = (90/180)*pi
joint_goal[1] = (-90/180)*pi
joint_goal[2] = (-90/180)*pi
joint_goal[3] = (-135/180)*pi
joint_goal[4] = (0/180)*pi
joint_goal[5] = (-45/180)*pi
joint_goal[6] = (0/180)*pi

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
	
