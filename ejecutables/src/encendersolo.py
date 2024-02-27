#!/usr/bin/env python3

import roslaunch
import os
import time
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

if __name__ == '__main__':
	# Ejecuta el comando "roslaunch" con los argumentos especificados
	launch_process = subprocess.Popen(['gnome-terminal', '--','roslaunch', 'paquete_de_prueba2', 'moveit_planning_execution.launch', 'controller:=fs100', 'robot_ip:=192.168.100.200', 'sim:=false'])
	#os.system('roslaunch paquete_de_prueba2 moveit_planning_execution.launch controller:=fs100 robot_ip:=192.168.100.200 sim:=false')
	time.sleep(5)
	subprocess.Popen(['gnome-terminal', '--', 'rosservice', 'call', '/robot_enable'])
	time.sleep(5)
	subprocess.Popen(['gnome-terminal', '--', 'rosrun', 'ejecutables', 'MovimientoCartesians2.py'])
	
	# result2 = subprocess.run(['roslaunch', 'motoman_sda10f_support', 'robot_interface_streaming_sda10f.launch', 'controller:=fs100', 'robot_ip:=192.168.100.200'], stdout=subprocess.PIPE)
	
	# Imprime el resultado
	# print(result.stdout)
