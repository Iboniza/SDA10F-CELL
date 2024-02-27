#! /usr/bin/env python3

import time
import threading
import paho.mqtt.client as mqtt
import json
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


class MQTTClient:
    def __init__(self, machine):
        self.machine = machine
        mqtt_client.on_message = self.on_message
        mqtt_client.connect("localhost", 1883, 60)
        mqtt_client.subscribe("machine/input")
        mqtt_client.loop_start()
        
    def on_message(self, client, userdata, msg):
        user_input = msg.payload.decode()
        self.machine.handle_input(user_input)

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
        # Asignar mqtt_client.on_message al método on_message de la clase MQTTClient
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("Machine is OFF")
        print("Press 'o' to turn on the machine")      

    def handle_input(self, user_input):
        if user_input == 'o':
            self.machine.set_state(Powering)
        else:
            print("That's not an option, please press 'o'")


class Powering(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.off = 0
        self.interrupt_event = threading.Event()
        
    def on_enter(self):
        print("Powering up the machine...")
        
        # Función que se ejecutará en segundo plano
        def wait_and_execute():
            self.powering_code()
            if not self.interrupt_event.is_set():
                self.execute()

        def handle_message(client, userdata, msg):
            user_input = msg.payload.decode()
            if user_input == 'o':
                self.interrupt_event.set()
                thread.join()  #Para esperar a que el hilo secundario termine su ejecución antes de pasar al estado OFF, puede ser interesante para otras funciones o evitar errores con el inicio del programa move_it
                self.machine.set_state(OFF)
            else:
                self.execute()

        # Suscribirse al tema MQTT correspondiente para recibir el mensaje de entrada
        mqtt_client.subscribe("machine/input")
        mqtt_client.on_message = handle_message

        # Crear y ejecutar el hilo secundario
        thread = threading.Thread(target=wait_and_execute)
        thread.start()
        
    def execute(self):
        if self.off == 0:
            self.machine.set_state(StandStill)

    def powering_code(self):
        if __name__ == '__main__':
        	launch_process = subprocess.Popen(['gnome-terminal', '--','roslaunch','paquete_de_prueba2', 'moveit_planning_execution.launch','controller:=fs100', 'robot_ip:=192.168.100.56', 'sim:=false'])
        	time.sleep(20)
        	subprocess.Popen(['gnome-terminal', '--', 'rosservice', 'call', '/robot_enable'])
        	time.sleep(5)
             
    def handle_input(self, user_input):
        if user_input == 'o':
            self.off = 1
            self.machine.set_state(OFF)
        else:
            print("Invalid input in Powering state")


class StandStill(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.state_map = {
            'a': AutoState,
            'm': ManualState,
            'f': MaintenanceState,
            'o': OFF
        }

    def on_enter(self):
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message 
        print("Machine ON, waiting entrance:")
        print("- 'a' for Automatic")
        print("- 'm' for Manual")
        print("- 'f' for Maintenance")
        print("- 'o' for Power OFF")

    def handle_input(self, user_input):
        global previous_state
        if user_input in self.state_map:
            state_class = self.state_map[user_input]
            previous_state = self.__class__.__name__
            print(state_class.__name__ + " mode entered:")
            self.machine.set_state(state_class)
        else:
            print("Invalid input for machine mode")


class AutoState(State):
    def __init__(self, machine):
        super().__init__(machine)
        global machine_mode
        machine_mode = "auto"
        self.previous_state = previous_state

    def on_enter(self):
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        if self.previous_state.__class__.__name__ == "StandStill" or \
                self.previous_state.__class__.__name__ == "AutoState":
            print("(Process starting in 2 secs)")
            print("(To exit this mode first pause pressing 'p')")
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
        print("Invalid input in automatic mode")


class ManualState(State):
    def __init__(self, machine):
        super().__init__(machine)
        global machine_mode
        machine_mode = "manual"
        self.previous_state = previous_state

    def on_enter(self):
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("- Press 'n' to step in the process")
        print("- Press 's' to enter Mode Selection")
        print("- Press 'o' to power OFF the machine")

    def handle_input(self, user_input):
        global previous_state
        if user_input == 'n':
            if self.previous_state.__class__.__name__ == "StandStill" or \
                    self.previous_state.__class__.__name__ == "AutoState":
                self.machine.set_state(InitialState)
            elif self.previous_state.__class__.__name__ == "InitialState":
                self.machine.set_state(MillingState)
            elif self.previous_state.__class__.__name__ == "MillingState":
                self.machine.set_state(PolishingState)
            elif self.previous_state.__class__.__name__ == "PolishingState":
                time.sleep(4)
                self.machine.set_state(MillingState)
        elif user_input == 'o':
            self.machine.set_state(OFF)
        elif user_input == 's':
            self.machine.set_state(StandStill)
        else:
            print("Invalid input for machine mode")


class MaintenanceState(State):
    def __init__(self, machine):
        super().__init__(machine)
        global machine_mode
        machine_mode = "maintenance"
        self.state_map = {
            'i': InitialState,
            'm': MillingState,
            'p': PolishingState,
            's': StandStill,
            'o': OFF,
        }

    def on_enter(self):
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("Maintenance mode options:")
        print("- Press 'i' to Initial Position")
        print("- Press 'm' to Milling Path")
        print("- Press 'p' to Polishing path")
        print("- Press 's' to enter Mode Selection")
        print("- Press 'o' to power OFF the machine")

    def handle_input(self, user_input):
        if user_input in self.state_map:
            state_class = self.state_map[user_input]
            self.machine.set_state(state_class)
        else:
            print("Invalid input in maintenance mode")


class InitialState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.off = 0
        self.previous_state = previous_state
        self.interrupt_event = threading.Event()

    def on_enter(self):
        print("Setting starting position")
        if machine_mode == "auto":
            def wait_and_execute():
                self.initial_pos_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            def handle_message(client, userdata, msg):
                user_input = msg.payload.decode()
                if user_input == 'p':
                    self.interrupt_event.set()
                    thread.join()  
                    self.machine.set_state(PauseState)
                else:
                    self.execute()
            mqtt_client.subscribe("machine/input")
            mqtt_client.on_message = handle_message

            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()
        else:
            self.initial_pos_code()
            self.execute()
         
    def initial_pos_code(self):
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
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
        time.sleep(5) 

    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

    def handle_input(self, user_input):
        if user_input == 'p':
            self.off = 1
            self.machine.set_state_sin_guardar_state(PauseState)
        else:
            print("Invalid input in InitialState")


class MillingState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.off = 0
        self.previous_state = previous_state
        self.interrupt_event = threading.Event()

    def on_enter(self):
        print("Machine is Milling...")
        if machine_mode == "auto":
            def wait_and_execute():
                self.milling_path_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            def handle_message(client, userdata, msg):
                user_input = msg.payload.decode()
                if user_input == 'p':
                    self.interrupt_event.set()
                    thread.join()  
                    self.machine.set_state(PauseState)
                else:
                    self.execute()
            mqtt_client.subscribe("machine/input")
            mqtt_client.on_message = handle_message

            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()
        else:
            self.milling_path_code()
            self.execute()

    def milling_path_code(self):
        ## Configuracion
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
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
        time.sleep(5)
    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

    def handle_input(self, user_input):
        if user_input == 'p':
            self.off = 1
            self.machine.set_state_sin_guardar_state(PauseState)
        else:
            print("Invalid input in MillingState")


class PolishingState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.off = 0
        self.previous_state = previous_state
        self.interrupt_event = threading.Event()

    def on_enter(self):
        print("Machine is Polishing...")
        if machine_mode == "auto":
            def wait_and_execute():
                self.polishing_path_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            def handle_message(client, userdata, msg):
                user_input = msg.payload.decode()
                if user_input == 'p':
                    self.interrupt_event.set()
                    thread.join()  
                    self.machine.set_state(PauseState)
                else:
                    self.execute()
            mqtt_client.subscribe("machine/input")
            mqtt_client.on_message = handle_message

            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()
        else:
            self.polishing_path_code()
            self.execute()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

    def polishing_path_code(self):
        ## Configuracion
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
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
        timeout = 1
        while (seconds - start < timeout) and not rospy.is_shutdown():
                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time() 
        time.sleep(5)
        
    def handle_input(self, user_input):
        if user_input == 'p':
            self.off = 1
            self.machine.set_state_sin_guardar_state(PauseState)
        else:
            print("Invalid input in PolishingState")


class PauseState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("Entering pause state:")
        print("- Press 'p' to exit Pause state")
        print("- Press 's' to enter Mode Selection")

    def on_exit(self):
        print("Exiting pause state")

    def handle_input(self, user_input):
        if user_input == 'p':
            self.machine.set_state_sin_guardar_state(AutoState)
        elif user_input == 's':
            self.machine.set_state(StandStill)
        else:
            print("Invalid input in PauseState")


previous_state = None
machine_mode = None
mqtt_client = mqtt.Client()

class Machine:
    def __init__(self):
        self.current_state = OFF(self)
        self.mqtt_client = MQTTClient(self)
        
    def on_message(self, client, userdata, msg):             
        user_input = msg.payload.decode()
        self.handle_input(user_input) 
        #if self.current_state.__class__.__name__ == "StandStill":
            #moveit_commander.roscpp_initialize(sys.argv)
            #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
             
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
        self.current_state.user_input = user_input
        self.current_state.handle_input(user_input)

    def run(self):
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        while True:
            # Esperar por mensajes MQTT en el hilo de eventos MQTT
            #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
            time.sleep(0.5)
           

machine = Machine()
machine.run()
