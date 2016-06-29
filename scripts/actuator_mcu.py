#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import serial
import threading

class MotorReader(threading.Thread):
	def __init__(self, serial_handle):
		super(MotorReader, self).__init__()
		self.serial_handle = serial_handle
		self.finalize = False





