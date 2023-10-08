import rclpy
from rclpy.node import Node
import numpy as np

# from moonbot_custom_interfaces.msg 
from IK.InverseKinematics import InverseKinematics
from cmd_manager.hyperdog_variables import Body, Leg, Cmds

from std_msgs.msg import String
import threading
from threading import Thread
import logging
import time

class CmdManager_ROS():
	def __init__():