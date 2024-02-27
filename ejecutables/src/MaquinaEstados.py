#!/usr/bin/env python3

import paho.mqtt.client as mqtt
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
	print("Paso por aqui 1")
	pass
def next(self, input):
	print("Paso por aqui 2")
	pass
	
class StateInicio(State):
    def run(self):
        print("Comenzando proceso de limado y pulido")

    def next(self, input):
        return StateMilling()


class StateMilling(State):
    def run(self):
        print("Usando la fresadora para limar los bordes")

    def next(self, input):
        if input == "pausa":
            return StatePausa()
        else:
            return StatePolishing()


class StatePolishing(State):
    def run(self):
        print("Usando la pulidora para pulir la superficie")

    def next(self, input):
        if input == "pausa":
            return StatePausa()
        else:
            return StateFin()


class StatePausa(State):
    def run(self):
        print("El robot se encuentra en pausa")

    def next(self, input):
        if input == "reanudar":
            return StatePolishing()
        elif input == "parar":
            return StateParadaDeEmergencia()
        else:
            return self


class StateFin(State):
    def run(self):
        print("El proceso de limado y pulido ha finalizado")

    def next(self, input):
        return StateInicio()


class StateParadaDeEmergencia(State):
    def run(self):
        print("El robot se ha detenido por una emergencia")

    def next(self, input):
        if input == "automatico":
            return StateInicio()
        else:
            return self


class ModoAutomatico(State):
    def run(self):
        print("El robot está en modo automático")
        self.state = StateInicio()

    def next(self, input):
        if input == "manual":
            return ModoManual()
        elif input == "mantenimiento":
            return ModoMantenimiento()
        else:
            return self


class ModoManual(State):
    def run(self):
        print("El robot está en modo manual")
        self.state = StateInicio()

    def next(self, input):
        if input == "automatico":
            return ModoAutomatico()
        else:
            return self


class ModoMantenimiento(State):
    def run(self):
        print("El robot está en modo de mantenimiento")
        self.state = StateInicio()

    def next(self, input):
        if input == "automatico":
            return ModoAutomatico()
        else:
            return self


class StateMachine:
    print("Paso por aqui 3")
    def __init__(self):
        print("Paso por aqui 4")
        self.state = ModoAutomatico()

    def run(self):
        print("Paso por aqui 5")
        client = mqtt.Client()
        client.connect("localhost", 1883)
        client.subscribe("boton/#")
        client.on_message = on_message
        client.loop_start()

        while True:
            self.state.run()

        client.loop_stop()
        client.disconnect()


def on_message(client, userdata, message):
    topic = message.topic
    payload = message.payload.decode()
    print("Paso por aqui 6")

    if topic == "boton/error":
        stateMachine.state = StateParadaDeEmergencia()
    elif topic == "boton/iniciartrabajo":
        stateMachine.state = StateInicio()
    elif topic == "boton/automatico":
        stateMachine.state = ModoAutomatico()
    elif topic == "boton/manual":
        stateMachine.state = ModoManual()
    elif topic == "boton/mantenimiento":
        stateMachine.state = ModoMantenimiento()
    elif topic == "boton/reanudar":
    	if isinstance(stateMachine.state, StatePausa):
    		stateMachine.state = stateMachine.state.next("reanudar")
    	else:
    		print("El botón 'reanudar' no está disponible en el estado actual.")
