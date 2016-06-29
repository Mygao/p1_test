#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import serial
import threading

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
    
    data = serial_handle.read(1)
    
    header = ord(data[0])
    
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
    print command
    print data_stripped
    return command, data_stripped

class MCUReader(threading.Thread):
	def __init__(self, serial_handle):
		super(MCUReader, self).__init__()
		self.serial_handle = serial_handle
		self.finalize = False
		self.rate = rospy.Rate(10)
		self.pub_joy = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

	def run(self):
		joy_msg = Twist()
        while self.finalize is False:
        	req = struct.pack('<BBBB', HEADER, 0x05, REQ_MOTOR_SET_VEL, HEADER^0x02^REQ_MOTOR_SET_VEL^vel^rot)
			g_request_queue.append(req)
        	command, = doParse(serial_handle)
        	
        	if command == RES_JOY:
        		joy_msg.linear.x = 0.0
            	joy_msg.angular.z = 0.0

            	vel, rot, = struct.unpack('<bb', data_stripped[1:3])
            	print 'Joy: %d, %d' % (vel, rot)

            	vel = float(vel) / 100.0
            	rot = float(rot) / 100.0

            	joy_msg.linear.x = vel
            	joy_msg.angular.z = rot
            	#publish
            	self.pub_joy.publish(joy_msg)

            self.rate.sleep()

def callback(data):
	#data enqueue [data.linear.x, data.angular.z]
	vel = data.linear.x * MAX_VEL
	rot = data.angular.z * MAX_VEL
	req = struct.pack('<BBBhhB', HEADER, 0x05, REQ_MOTOR_SET_VEL, vel, rot, HEADER^0x02^REQ_MOTOR_SET_VEL^vel^rot)
	g_request_queue.append(req)

def actuator():

	serial_handle = serial.Serial("/dev/MotorMCU", 115200, timeout=1.0)
	rospy.init_node("actuator", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)

    #motor enable
    req = struct.pack('<BBBBB', HEADER, 0x02, REQ_MOTOR_ONOFF, REQ_MOTOR_OPTION_ON, HEADER^0x02^REQ_MOTOR_ONOFF^REQ_MOTOR_OPTION_ON)
    serial_handle.write(req)

	while not rospy.is_shutdown():
		if len(g_request_queue) > 3000:
			print '!!!!!!!!!request queue is too large.. Exitting'
			break

		if len(g_request_queue) != 0:
			 req = g_request_queue.pop(0)
			 serial_handle.write(req)

	#motor disable
    req = struct.pack('<BBBBB', HEADER, 0x02, REQ_MOTOR_ONOFF, REQ_MOTOR_OPTION_OFF, HEADER^0x02^REQ_MOTOR_ONOFF^REQ_MOTOR_OPTION_OFF)
    serial_handle.write(req)

if __name__ == "__main__":
    actuator()
