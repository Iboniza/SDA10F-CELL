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
joint_goal[0] = (1/180)*pi
joint_goal[1] = (1/180)*pi
joint_goal[2] = (1/180)*pi
joint_goal[3] = (1/180)*pi
joint_goal[4] = (1/180)*pi
joint_goal[5] = (1/180)*pi
joint_goal[6] = (1/180)*pi
joint_goal[7] = (1/180)*pi

joint_goal[8] = (90/180)*pi
joint_goal[9] = (-90/180)*pi
joint_goal[10] = (-90/180)*pi
joint_goal[11] = (-135/180)*pi
joint_goal[12] = (-180/180)*pi
joint_goal[13] = (48/180)*pi
joint_goal[14] = (0/180)*pi
joint_goal[15] = (1/180)*pi

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
#Hasta aquí bien-------------------------------------------------------------------


group_name = "arm_right"
group = moveit_commander.MoveGroupCommander(group_name)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)


## Cartesian Paths
waypoints = []
scale = 1
wpose = group.get_current_pose().pose

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.z -= scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
wpose.position.y -= scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
wpose.position.y += scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += scale * 0.07  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x -= scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.z += scale * 0.08  # Second move forward/backwards in (x)
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

## Wait_for_scene_update
## Ensuring Collision Updates Are Receieved
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():

	rospy.sleep(0.1)
	seconds = rospy.get_time()
	
# Hasta aquí funciona bien---------------------------------------------------------------------

#Primera iteración en coordenadas cartesianas
#Ahora movemos el otro brazo y el torso con el grupo sda10f

group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()
joint_goal[0] = (30/180)*pi
joint_goal[1] = (60/180)*pi
joint_goal[2] = (29/180)*pi
joint_goal[3] = (33/180)*pi
joint_goal[4] = (-64/180)*pi
joint_goal[5] = (0/180)*pi
joint_goal[6] = (-93/180)*pi
joint_goal[7] = (0/180)*pi
joint_goal[15] = (30/180)*pi

group.go(joint_goal, wait=True)
group.stop()

start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():
	rospy.sleep(0.1)
	seconds = rospy.get_time()
	
#Hasta aquí funciona bien------------------------------------------------------

#Cogemos pieza imaginaria

group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()
joint_goal[4] = (-46/180)*pi
joint_goal[6] = (-67/180)*pi

group.go(joint_goal, wait=True)
group.stop()

start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():
	rospy.sleep(0.1)
	seconds = rospy.get_time()
#Hasta aquí funciona bien----------------------------------------------------------

#La colocamos donde queremos

group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()

joint_goal[1] = (-68/180)*pi
joint_goal[2] = (80/180)*pi
joint_goal[3] = (107/180)*pi
joint_goal[4] = (-90/180)*pi
joint_goal[5] = (11/180)*pi
joint_goal[6] = (-44/180)*pi

group.go(joint_goal, wait=True)
group.stop()

start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():
	rospy.sleep(0.1)
	seconds = rospy.get_time()
#Hasta aquí funciona bien--------------------------------------------------------------

#La colocamos donde queremos

group_name = "sda10f"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()

joint_goal[0] = (1/180)*pi
joint_goal[15] = (1/180)*pi

group.go(joint_goal, wait=True)
group.stop()

start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():
	rospy.sleep(0.1)
	seconds = rospy.get_time()

#La colocamos donde queremos

group_name = "arm_left"
group = moveit_commander.MoveGroupCommander(group_name)

waypoints = []
scale = 1
wpose = group.get_current_pose().pose

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
wpose.position.z -= scale * 0.05  # Second move forward/backwards in (x)
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

## Wait_for_scene_update
## Ensuring Collision Updates Are Receieved
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():

# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	
	
	
	seconds = rospy.get_time()

#Volvemos a trabajar con el bazo derecho con el torso en su posición

group_name = "arm_right"
group = moveit_commander.MoveGroupCommander(group_name)

waypoints = []
scale = 1
wpose = group.get_current_pose().pose

wpose.position.y -= scale * 0.07  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.17  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.z -= scale * 0.08  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
wpose.position.y -= scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x -= scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
wpose.position.y += scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += scale * 0.07  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x -= scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.z += scale * 0.08  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x -= scale * 0.10  # Second move forward/backwards in (x)
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

## Wait_for_scene_update
## Ensuring Collision Updates Are Receieved
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():

# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()
	
