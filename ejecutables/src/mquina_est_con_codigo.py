#! /usr/bin/env python3

import time
import threading
import keyboard
import roslaunch
import os
import time
import math
import subprocess
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class State:
    def __init__(self, machine):
        self.machine = machine

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_input(self, user_input):
        pass


class OFF(State):
    print("Machine is OFF")
    print("Press 'o' to turn on the machine")

    def on_enter(self):
        print("Machine OFF")
        print("Press 'o' to turn on the machine")

    def handle_input(self, user_input):
        if user_input == 'o':
            self.machine.set_state(Powering)
        else:
            print("That's not an option, please press 'o'")


class Powering(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.keyboard_thread = None
        self.off = 0

    def on_enter(self):
        print("Powering up the machine...")
        # Start the thread to monitor for 'q' key press
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()
        
        if __name__ == '__main__':
        	launch_process = subprocess.Popen(['gnome-terminal', '--','roslaunch','paquete_de_prueba2', 'moveit_planning_execution.launch','controller:=fs100', 'robot_ip:=192.168.100.56', 'sim:=false'])
        	time.sleep(5)
        	subprocess.Popen(['gnome-terminal', '--', 'rosservice', 'call', '/robot_enable'])
        	time.sleep(5)
        self.execute()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(StandStill)

    def keyboard_listener(self):
        keyboard.add_hotkey('q', self.turn_off)

    def turn_off(self):
        self.off = 1
        self.machine.set_state(OFF)
        keyboard.unhook_all()


class StandStill(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.state_map = {
            'a': AutoState,
            'm': ManualState,
            'f': MaintenanceState
        }

    def on_enter(self):
        print("Machine ON, waiting entrance: ")
        print("-'a' for Automatic")
        print("-'m' for Manual")
        print("-'f' for Maintenance")

    def handle_input(self, user_input):
        global previous_state
        if user_input in self.state_map:
            state_class = self.state_map[user_input]
            previous_state = self.__class__.__name__
            print(state_class.__name__ + " mode entered: (Process starting in 2 secs)")
            self.machine.set_state(state_class)
        else:
            print("Invalid input for machine mode")


class AutoState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        if self.previous_state.__class__.__name__ == "StandStill" or \
                self.previous_state.__class__.__name__ == "AutoState":
            #print("Automatic mode entered: (Process starting in 2 secs)")
            time.sleep(2)
            self.machine.set_state(InitialState)
        elif self.previous_state.__class__.__name__ == "InitialState":
            self.machine.set_state(MillingState)
        elif self.previous_state.__class__.__name__ == "MillingState":
            self.machine.set_state(PolishingState)
        elif self.previous_state.__class__.__name__ == "PolishingState":
            time.sleep(4)
            self.machine.set_state(MillingState)

    def handle_input(self, user_input):
        #if user_input in self.state_map:
            #state_class = self.state_map[user_input]
            #self.machine.set_state(state_class)
        #else:
        print("Invalid input in automatic mode")


class ManualState(State):
    def on_enter(self):
        print("Entering manual mode")

    def handle_input(self, user_input):
        if user_input == 's':
            self.machine.set_state(StandStill)
        else:
            print("Invalid input in manual mode")


class MaintenanceState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.state_map = {
            's': InitialState,
            'm': MillingState,
            'p': PolishingState,
            'pause': PauseState
        }

    def on_enter(self):
        print("Maintenance mode entered: (Press 's' to Start)")
        print("-Press 's' to Start")
        print("-Press 'm' to ManualMode")
        print("-Press 'f' to MaintenanceMode")

    def handle_input(self, user_input):
        if user_input in self.state_map:
            state_class = self.state_map[user_input]
            self.machine.set_state(state_class)
        else:
            print("Invalid input in maintenance mode")


class InitialState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.keyboard_thread = None
        self.off = 0
        self.previous_state = previous_state

    def on_enter(self):
        print("Setting starting position")
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "sda10f"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 		moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)
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
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 4
        while (seconds - start < timeout) and not rospy.is_shutdown():
                rospy.sleep(0.1)
                seconds = rospy.get_time()

        time.sleep(10)
        self.execute()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

    def keyboard_listener(self):
        keyboard.add_hotkey('p', self.turn_off)

    def turn_off(self):
        self.off = 1
        self.machine.set_state_sin_guardar_state(PauseState)
        keyboard.unhook_all()


class MillingState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        print("Machine is milling")
        
        ## Configuracion
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 		moveit_msgs.msg.DisplayTrajectory, queue_size=20)

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
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0) 
        ## Displaying a Trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## Executing a Plan
        group.execute(plan, wait=True)
        
        time.sleep(3)
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
        
        time.sleep(10)
        self.machine.set_state(type(self.previous_state))

    def handle_input(self, user_input):
        if user_input == 'p':
            self.machine.set_state(PauseState)
        else:
            print("Invalid input in milling state")


class PolishingState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        print("Machine is polishing")
        
        ## Configuracion
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        group_name = "arm_right"
        group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        ## Cartesian Paths
        waypoints = []
        scale = 1
        wpose = group.get_current_pose().pose

        wpose.position.x += scale * 0.25  # Move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.26  # Move left/right in (y)
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
        wpose.position.y -= scale * 0.26  # Move left/right in (y)
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
        
        time.sleep(3)
        group_name = "arm_right"
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
                
        time.sleep(10)
        self.machine.set_state(type(self.previous_state))

    def handle_input(self, user_input):
        if user_input == 'p':
            self.machine.set_state(PauseState)
        elif user_input == 's':
            self.machine.set_state(StandStill)
        elif user_input == 'h':
            print("Press 'p' for Pause or 's' to Stop")
        else:
            print("Invalid input in polishing state: (press 'h' for help)")


class PauseState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        print("Entering pause state")

    def on_exit(self):
        print("Exiting pause state")

    def handle_input(self, user_input):
        if user_input == 'p':
            self.machine.set_state_sin_guardar_state(AutoState)
        elif user_input == 's':
            self.machine.set_state(StandStill)
        else:
            print("Invalid input in pause state")


previous_state = None


class Machine:
    def __init__(self):
        self.current_state = OFF(self)

    def set_state(self, state_class):
        global previous_state
        if self.current_state:
            self.current_state.on_exit()
        previous_state = self.current_state
        self.current_state = state_class(self)
        self.current_state.on_enter()

    def set_state_sin_guardar_state(self, state_class):
        global previous_state
        if self.current_state:
            self.current_state.on_exit()
        self.current_state = state_class(self)
        self.current_state.on_enter()

    def handle_input(self, user_input):
        self.current_state.handle_input(user_input)

    def run(self):
        while True:
            user_input = input("> ")
            if isinstance(self.current_state, OFF):
                self.current_state.handle_input(user_input)
            else:
                self.handle_input(user_input)


machine = Machine()
machine.run()
