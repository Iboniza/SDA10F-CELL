#! /usr/bin/env python3

import time
import threading
import keyboard


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
        keyboard.unhook_all()
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
        self.button_thread = None
        self.off = 0

    def on_enter(self):
        print("Powering up the machine...")
        # Start the thread to monitor for 'q' key press
        self.button_thread = threading.Thread(target=self.button_listener)
        self.button_thread.start()
        time.sleep(2)
        self.execute()

    def execute(self):
        if self.off == 0:
            self.machine.set_state(StandStill)

    def button_listener(self):
        #logic for button press goes here

    def turn_off(self, channel):
        self.off = 1
        self.machine.set_state(OFF)


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
        keyboard.unhook_all()
        print("Machine ON, waiting entrance: ")
        print("-'a' for Automatic")
        print("-'m' for Manual")
        print("-'f' for Maintenance")
        print("-'o' for Power OFF")

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
        #if user_input in self.state_map:
            #state_class = self.state_map[user_input]
            #self.machine.set_state(state_class)
        #else:
        print("Invalid input in automatic mode")


class ManualState(State):
    def __init__(self, machine):
        super().__init__(machine)
        global machine_mode
        machine_mode = "manual"
        self.previous_state = previous_state

    def on_enter(self):
        print("-Press 'n' to step in the process")
        print("-Press 's' to enter Mode Selection")
        print("-Press 'o' to power OFF the machine")

    def handle_input(self, user_input):
        global previous_state
        if user_input == 'n':
            if self.previous_state.__class__.__name__ == "StandStill" or \
                    self.previous_state.__class__.__name__ == "AutoState":
                # print("Automatic mode entered: (Process starting in 2 secs)")
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
        keyboard.unhook_all()
        print("Maintenance mode entered:")
        print("-Press 'i' to Initial Position")
        print("-Press 'm' to Milling Path")
        print("-Press 'p' to Polishing path")
        print("-Press 's' to enter Mode Selection")
        print("-Press 'o' to power OFF the machine")

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
        keyboard.unhook_all()
        print("Setting starting position")
        if machine_mode == "auto":
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        time.sleep(5)
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
        self.keyboard_thread = None
        self.off = 0
        self.previous_state = previous_state

    def on_enter(self):
        keyboard.unhook_all()
        print("Machine is Milling...")
        if machine_mode == "auto":
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        time.sleep(5)
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


class PolishingState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.keyboard_thread = None
        self.off = 0
        self.previous_state = previous_state

    def on_enter(self):
        keyboard.unhook_all()
        print("Machine is Polishing...")
        if machine_mode == "auto":
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        time.sleep(5)
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


class PauseState(State):
    def __init__(self, machine):
        super().__init__(machine)
        self.previous_state = previous_state

    def on_enter(self):
        keyboard.unhook_all()
        print("Entering pause state:")
        print("-Press 'p' to exit Pause state")
        print("-Press 's' to enter Mode Selection")

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
machine_mode = None

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
            #logic gor button press goes here
            if GPIO.input(button_pin) == GPIO.HIGH:
                self.handle_input(button_name)
            user_input = input("> ")
            if isinstance(self.current_state, OFF):
                self.current_state.handle_input(user_input)
            else:
                self.handle_input(user_input)


machine = Machine()
machine.run()

