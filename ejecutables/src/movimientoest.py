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


"""
def all_close(goal, actual, tolerance):
  
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  MoveGroupPythonIntefaceTutorial
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
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
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL
"""

## BEGIN_SUB_TUTORIAL setup
    ##
"""
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
"""
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:

## BEGIN_SUB_TUTORIAL setup
##
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
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                           moveit_msgs.msg.DisplayTrajectory,
                                           queue_size=20)

## END_SUB_TUTORIAL



eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

## BEGIN_SUB_TUTORIAL plan_to_joint_state
##
## Planning to a Joint Goal
## ^^^^^^^^^^^^^^^^^^^^^^^^
## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
## thing we want to do is move it to a slightly better configuration.
# We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
joint_goal[0] = -pi/2
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()

## END_SUB_TUTORIAL

"""
## BEGIN_SUB_TUTORIAL plan_to_pose
##
## Planning to a Pose Goal
## ^^^^^^^^^^^^^^^^^^^^^^^
## We can plan a motion for this group to a desired pose for the
## end-effector:
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4
group.set_pose_target(pose_goal)

## Now, we call the planner to compute the plan and execute it.
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

## END_SUB_TUTORIAL

"""
"""
## BEGIN_SUB_TUTORIAL display_trajectory
##
## Displaying a Trajectory
## ^^^^^^^^^^^^^^^^^^^^^^^
## You can ask RViz to visualize a plan (aka trajectory) for you. But the
## group.plan() method does this automatically so this is not that useful
## here (it just displays the same trajectory again):
##
## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
## We populate the trajectory_start with our current robot state to copy over
## any AttachedCollisionObjects and add our plan to the trajectory.
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);

## END_SUB_TUTORIAL
"""
"""
## BEGIN_SUB_TUTORIAL execute_plan
##
## Executing a Plan
## ^^^^^^^^^^^^^^^^
## Use execute if you would like the robot to follow
## the plan that has already been computed:
group.execute(plan, wait=True)

## **Note:** The robot's current joint state must be within some tolerance of the
## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
## END_SUB_TUTORIAL
"""

## BEGIN_SUB_TUTORIAL wait_for_scene_update
##
## Ensuring Collision Updates Are Receieved
## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
## If the Python node dies before publishing a collision object update message, the message
## could get lost and the box will not appear. To ensure that the updates are
## made, we wait until we see the changes reflected in the
## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
## For the purpose of this tutorial, we call this function after adding,
## removing, attaching or detaching an object in the planning scene. We then wait
## until the updates have been made or ``timeout`` seconds have passed
start = rospy.get_time()
seconds = rospy.get_time()
timeout = 4
while (seconds - start < timeout) and not rospy.is_shutdown():

# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()

# If we exited the while loop without returning then we timed out
## END_SUB_TUTORIAL
