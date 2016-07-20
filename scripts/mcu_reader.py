#!/usr/bin/env python
import threading
import struct

import rospy
from geometry_msgs.msg import Twist

import mcu_parser
import mcu_protocol


class MCUReader(threading.Thread):
    def __init__(self, serial_handle):
        super(MCUReader, self).__init__()
        self.serial_handle = serial_handle
        self.finalize = False
        self.rate = rospy.Rate(100)
        self.pub_joy = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def run(self):
        joy_msg = Twist()
        while self.finalize is False:
            command, data_stripped = mcu_parser.doParse(self.serial_handle)

            if command == "em":
                self.rate.sleep()
                continue

            if command == mcu_protocol.RES_JOY:
                #print 'E %d' % (ord(data_stripped[1]))
                if ord(data_stripped[1]) == 0:
                    joy_msg.linear.x = 0.0
                    joy_msg.linear.y = 0.0
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

                    vel = -float(vel) / 100.0
                    rot = float(rot) / 100.0

                    joy_msg.linear.x = vel
                    joy_msg.angular.z = rot
                    #publish
                    self.pub_joy.publish(joy_msg)

                elif ord(data_stripped[1]) == 1:
                    joy_msg.linear.x = 0.0
                    joy_msg.linear.y = 0.0
                    joy_msg.linear.z = 1.0
                    joy_msg.angular.z = 0.0

                    left, right, = struct.unpack('<bb', data_stripped[2:4])
                    #print left, right
                    if (left < 20 and left > -10):
                        left = 0
                    if (right < 20 and right > -10):
                        right = 0

                    joy_msg.linear.x = float(right) / 100.0
                    joy_msg.angular.z = float(left) / 100.0
                    self.pub_joy.publish(joy_msg)
                elif ord(data_stripped[1]) == 2:
                    joy_msg.linear.x = 0.0
                    joy_msg.linear.y = 0.0
                    joy_msg.linear.z = 1.0
                    joy_msg.angular.z = 0.0

                    left, right, = struct.unpack('<bb', data_stripped[2:4])
                    #print left, right
                    #if (left < 5 and left > -5):
                    #   left = 0
                    #if (right < 5 and right > -5):
                    #   right = 0

                    joy_msg.linear.x = float(right) / 100.0
                    joy_msg.angular.z = float(left) / 100.0
                    self.pub_joy.publish(joy_msg)

            self.rate.sleep()#while
