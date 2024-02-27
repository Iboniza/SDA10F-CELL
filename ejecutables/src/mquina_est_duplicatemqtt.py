#! /usr/bin/env python3

import time
import threading
import paho.mqtt.client as mqtt
import json

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
        
    def on_enter(self):
        print("Powering up the machine...")
        self.start_secondary_thread()
        time.sleep(5)
        self.execute()
        
    def start_secondary_thread(self):
        secondary_thread = threading.Thread(target=self.mqtt_listener) 
        secondary_thread.start() 
        
    def mqtt_listener(self):
        self.machine.cliente()
        while True:
            time.sleep(0.5)
        
    def execute(self):
        if self.off == 0:
            self.machine.set_state(StandStill)
            
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
        print("Maintenance mode entered:")
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

    def on_enter(self):
        print("Setting starting position")
        #if machine_mode == "auto":
            # Suscribirse al tema correspondiente en MQTT
            #self.client.subscribe("machine/position")
            # Enviar mensaje de posición inicial
            #self.client.publish("machine/position", "initial")
        time.sleep(5)
        self.execute()

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

    def on_enter(self):
        print("Machine is Milling...")
        #if machine_mode == "auto":
            # Suscribirse al tema correspondiente en MQTT
            #self.client.subscribe("machine/status")
            # Enviar mensaje de estado "milling"
            #self.client.publish("machine/status", "milling")
        time.sleep(5)
        self.execute()

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

    def on_enter(self):
        print("Machine is Polishing...")
        #if machine_mode == "auto":
            # Suscribirse al tema correspondiente en MQTT
            #self.client.subscribe("machine/status")
            # Enviar mensaje de estado "polishing"
            #self.client.publish("machine/status", "polishing")
        time.sleep(5)
        self.execute()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(type(self.previous_state))

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

class Machine:
    def __init__(self):
        self.current_state = OFF(self)
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        self.client.subscribe("machine/input")
        self.client.loop_start()
        
    def cliente(self):
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        self.client.subscribe("machine/input")
        self.client.loop_start()
        
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
            time.sleep(0.1)
           

machine = Machine()
machine.run()
