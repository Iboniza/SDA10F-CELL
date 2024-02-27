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
        global instructions
        self.machine = machine
        mqtt_client.on_message = self.on_message
        mqtt_client.connect("localhost", 1883, 60)
        mqtt_client.subscribe("machine/input")
        if machine.current_state.__class__.__name__ == "OFF":
            mqtt_client.publish("machine/state","Machine is OFF")
            mqtt_client.publish("machine/estado","iniciamos")
            instructions = "-Press POWER button to turn ON the machine"
        mqtt_client.publish("machine/instructions",instructions)
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
        global instructions
        # Asignar mqtt_client.on_message al método on_message de la clase MQTTClient
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        mqtt_client.publish("machine/state","Machine is OFF")
        mqtt_client.publish("machine/estado","OFF")
        print("Machine is OFF")
        print("Press 'o' to turn on the machine")  
        instructions = "-Press POWER button to turn ON the machine"
        mqtt_client.publish("machine/instructions",instructions)   

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
        global instructions
        print("Powering up the machine...")
        mqtt_client.publish("machine/estado","POWERING")
        mqtt_client.publish("machine/state","Powering up the machine...")
        instructions = "-Press POWER button to turn OFF the machine"
        mqtt_client.publish("machine/instructions",instructions)
        
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
        global instructions
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("Machine ON, waiting input:")
        mqtt_client.publish("machine/state","Machine ON, waiting input:")
        mqtt_client.publish("machine/estado","STANDSTILL")
        instructions = "-Press AUTO for automatic mode <br> -Press MANUAL for manual mode <br> -Press MAINTANANCE for maintenance mode <br> -Press POWER button to turn OFF the machine"
        mqtt_client.publish("machine/instructions",instructions)
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
        global instructions
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        mqtt_client.publish("machine/estado","AUTO")
        if self.previous_state.__class__.__name__ == "StandStill" or \
                self.previous_state.__class__.__name__ == "AutoState":
            mqtt_client.publish("machine/state","Auto mode entered:")
            instructions = "(Process starting in 2 secs) <br> (To exit this mode first pause pressing PAUSE)"
            mqtt_client.publish("machine/instructions",instructions)
            print("(Process starting in 2 secs)")
            print("(To exit this mode first pause pressing 'p')")
            def wait_and_execute():
                time.sleep(2)
                self.machine.set_state(InitialState)
            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()
            
            #self.machine.set_state(InitialState)
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
        global instructions
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        mqtt_client.publish("machine/state","Manual mode:")
        mqtt_client.publish("machine/estado","MANUAL")
        instructions = "-Press STEP to step in the process <br> -Press MODE SELECTION to select mode <br> -Press POWER button to power OFF the machine"
        mqtt_client.publish("machine/instructions",instructions)
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
            'l': PolishingState,
            's': StandStill,
            'o': OFF,
        }

    def on_enter(self):
        global instructions
        self.machine.mqtt_client.on_message = MQTTClient(self.machine).on_message
        print("Maintenance mode entered:")
        mqtt_client.publish("machine/state","Maintenance mode:")
        mqtt_client.publish("machine/estado","MAINTENANCE")
        instructions = "-Press INIT to Initial Position <br> -Press MILLING/POLISH to Milling/Polishing path <br> -Press MODE SELECTION to select mode <br> -Press POWER button to turn OFF the machine"
        mqtt_client.publish("machine/instructions",instructions)
        print("- Press 'i' to Initial Position")
        print("- Press 'm' to Milling Path")
        print("- Press 'l' to Polishing path")
        print("- Press 's' to enter Mode Selection")
        print("- Press 'o' to power OFF the machine")

    def handle_input(self, user_input):
        if user_input in self.state_map:
            time.sleep(2)
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
        mqtt_client.publish("machine/state","Setting starting position")
        
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
            #He añadido un hilo secundario en el else para hacer la conexion mqtt y que en el HMI nos aparezca la instrucción setting starting position
            def wait_and_execute():
                self.initial_pos_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()
         
    def initial_pos_code(self):
        comando = ['rosrun', 'ejecutables', 'class_InitialState.py']
        proceso = subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proceso.wait()
        #time.sleep(15) 
        

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
        mqtt_client.publish("machine/state","Machine is Milling...")
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
            def wait_and_execute():
                self.milling_path_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()

    def milling_path_code(self):
        comando = ['rosrun', 'ejecutables', 'class_MillingState.py']
        proceso = subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proceso.wait()
        #time.sleep(22)
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
        mqtt_client.publish("machine/state","Machine is Polishing...")
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
            def wait_and_execute():
                self.polishing_path_code()
                if not self.interrupt_event.is_set():
                    self.execute()
            # Crear y ejecutar el hilo secundario
            thread = threading.Thread(target=wait_and_execute)
            thread.start()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

    def polishing_path_code(self):
        comando = ['rosrun', 'ejecutables', 'class_PolishingState.py']
        proceso = subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proceso.wait()
        #time.sleep(22)
        
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
        mqtt_client.publish("machine/state","Entering pause state:")
        mqtt_client.publish("machine/estado","PAUSESTATE")
        instructions = "-Press PAUSE to exit Pause state <br> -Press MODE SELECTION to select mode <br> -Press POWER button to turn OFF the machine"
        mqtt_client.publish("machine/instructions",instructions)
        print("- Press 'p' to exit Pause state")
        print("- Press 's' to enter Mode Selection")
        print("- Press 'o' to power OFF the machine")

    def on_exit(self):
        print("Exiting pause state")

    def handle_input(self, user_input):
        if user_input == 'p':
            self.machine.set_state_sin_guardar_state(AutoState)
        elif user_input == 's':
            self.machine.set_state(StandStill)
        elif user_input == 'o':
            self.machine.set_state(OFF)
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
        while True:
            # Esperar por mensajes MQTT en el hilo de eventos MQTT
            time.sleep(0.5)
           

machine = Machine()
machine.run()
