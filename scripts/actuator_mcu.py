#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import serial
import threading
import struct

HEADER = 0xA5
REQ_JOY = 0x42
RES_JOY = 0xC2
REQ_MOTOR_ONOFF = 0x21
REQ_MOTOR_OPTION_ON = 0x01
REQ_MOTOR_OPTION_OFF = 0x00
REQ_MOTOR_SET_VEL = 0x22
REQ_MOTOR_ENC = 0x23
RES_MOTOR_ENC = 0x24

MAX_VEL = 300


g_request_queue = []

def doParse(serial_handle):
	#little endian '<'
	#big endian '>' 
	#print 'do parse' 
	data = serial_handle.read(1)
	if len(data) == 0:
		return "em", ""
	header = ord(data[0])
	#print "%x" % header    
	if header != HEADER:
		print 'header is unmatching'
		return "em", ""

	data = serial_handle.read(1)
	
	if len(data) == 0:
		print 'data length is 0'
		return "em", ""

	#length = struct.unpack('<B', data[0])
	length = ord(data[0])
    
	data_stripped = serial_handle.read(length + 1)
	#print len(data_stripped)
	command, = struct.unpack('<B', data_stripped[0])
	#print command
	#print data_stripped
	return command, data_stripped

class MCUReader(threading.Thread):
	def __init__(self, serial_handle):
		super(MCUReader, self).__init__()
		self.serial_handle = serial_handle
		self.finalize = False
		self.rate = rospy.Rate(100)
		self.pub_joy = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

	def run(self):
		joy_msg = Twist()
			while self.finalize is False:
				command, data_stripped = doParse(self.serial_handle)
				if command == "em":
					continue
				if command == RES_JOY:
					if data_stripped[1] == 0:
						joy_msg.linear.x = 0.0
						joy_msg.linear.z = 0.0
						joy_msg.angular.z = 0.0
					
						vel, rot, = struct.unpack('<bb', data_stripped[2:4])
							#print 'Joy: %d, %d' % (vel, rot)
						if (vel < 31 and vel > -31):
							vel = 0
						else:
							pass
							#print vel
						if (vel < 31 and vel > -31) or (rot < 31 and rot > -31):
							rot = 0
						else:
							pass
							#print vel

						vel = float(vel) / 100.0
						rot = float(rot) / 100.0

						joy_msg.linear.x = vel
						joy_msg.angular.z = rot
						#publish
						self.pub_joy.publish(joy_msg)
					else if data_stripped[1] == 1:
						joy_msg.linear.x = 0.0
						joy_msg.linear.z = 1.0
						joy_msg.angular.z = 0.0
					
						left, right, = struct.unpack('<bb', data_stripped[2:4])
						if (left < 10 and left > -10) or (right < 10 and right > -10):
							left = 0
							right = 0

						joy_msg.linear.x = float(left) / 100.0
						joy_msg.angular.z = float(right) / 100.0
						self.pub_joy.publish(joy_msg)

				self.rate.sleep()

def append_checksum(buff):
	checksum = 0
	for c in buff:
		checksum ^= ord(c)
	return checksum

def callback(data):
	#data enqueue [data.linear.x, data.angular.z]

	if data.linear.z == 0.0:
		if data.linear.x > 0:
			vel = -int(data.linear.x * MAX_VEL * 2/3 - data.angular.z * MAX_VEL * 1/3)
			rot = -int(data.linear.x * MAX_VEL * 2/3 + data.angular.z * MAX_VEL * 1/3)
		else:
			vel = -int(data.linear.x * MAX_VEL * 2/3 + data.angular.z * MAX_VEL * 1/3)
			rot = -int(data.linear.x * MAX_VEL * 2/3 - data.angular.z * MAX_VEL * 1/3)

		#print vel, rot

		req = struct.pack('<BBBhh', HEADER, 0x05, REQ_MOTOR_SET_VEL, vel, rot)
		req += chr(append_checksum(req))
		g_request_queue.append(req)

	elif data.linear.z == 1.0:
		left = int(data.linear.x * MAX_VEL)
		right = int(data.angular.z * MAX_VEL)
		req = struct.pack('<BBBhh', HEADER, 0x05, REQ_MOTOR_SET_VEL, left, right)
		req += chr(append_checksum(req))
		g_request_queue.append(req)

def actuator():

	serial_handle = serial.Serial("/dev/MotorMCU", 115200, timeout=1.0)
	rospy.init_node("actuator", anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback)
	print 'init complete'
	#motor enable
	#req = struct.pack('<BBBBB', HEADER, 0x02, REQ_MOTOR_ONOFF, REQ_MOTOR_OPTION_ON, HEADER^0x02^REQ_MOTOR_ONOFF^REQ_MOTOR_OPTION_ON)
	req = struct.pack('<BBBB', HEADER, 0x02, REQ_MOTOR_ONOFF, REQ_MOTOR_OPTION_ON)
	req += chr(append_checksum(req))

	serial_handle.write(req)
	print 'motor enable sent'
	reader = MCUReader(serial_handle)
	reader.start()
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		req = struct.pack('<BBB', HEADER, 0x01, REQ_JOY)
		req += chr(append_checksum(req))
		g_request_queue.append(req)
		if len(g_request_queue) > 3000:
			print '!!!!!!!!!request queue is too large.. Exitting'
			break

		while len(g_request_queue) != 0:
			req = g_request_queue.pop(0)
			serial_handle.write(req)

		rate.sleep()

	#motor disable
	req = struct.pack('<BBBB', HEADER, 0x02, REQ_MOTOR_ONOFF, REQ_MOTOR_OPTION_OFF)
	req += chr(append_checksum(req))
	serial_handle.write(req)
	reader.finalize = True
	reader.join()

if __name__ == "__main__":
	actuator()
