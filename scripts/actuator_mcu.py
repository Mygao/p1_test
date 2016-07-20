#!/usr/bin/env python
import serial
import struct

import rospy
from geometry_msgs.msg import Twist


import mcu_protocol
import mcu_reader

MAX_VEL = 450

g_request_queue = []
g_mode = 1


def append_checksum(buff):
    checksum = 0
    for c in buff:
        checksum ^= ord(c)
    return checksum


def callback(data):
    #data enqueue [data.linear.x, data.angular.z]
    global g_mode
    if data.linear.y == 2:
        g_mode = 2
    elif data.linear.y == 1:
        g_mode = 1

    if g_mode == 2 and data.linear.y == 2:
        print 'joy'
        vel = int(data.linear.x * MAX_VEL * 2 / 3 / 2 -
                  data.angular.z * MAX_VEL * 1 / 3 / 2)
        rot = int(data.linear.x * MAX_VEL * 2 / 3 / 2 +
                  data.angular.z * MAX_VEL * 1 / 3 / 2)

        req = struct.pack('<BBBhh', mcu_protocol.HEADER, 0x05,
                          mcu_protocol.REQ_MOTOR_SET_VEL, vel, rot)
        req += chr(append_checksum(req))
        g_request_queue.append(req)

        return

    if g_mode != 1:
        return

    if data.linear.z == 0.0 and data.linear.y == 0.0:
        if data.linear.x > 0:
            vel = -int(data.linear.x * MAX_VEL * 2 / 3 -
                       data.angular.z * MAX_VEL * 1 / 3)
            rot = -int(data.linear.x * MAX_VEL * 2 / 3 +
                       data.angular.z * MAX_VEL * 1 / 3)
        else:
            vel = -int(data.linear.x * MAX_VEL * 2 / 3 +
                       data.angular.z * MAX_VEL * 1 / 3)
            rot = -int(data.linear.x * MAX_VEL * 2 / 3 -
                       data.angular.z * MAX_VEL * 1 / 3)

        #print vel, rot

        req = struct.pack('<BBBhh', mcu_protocol.HEADER, 0x05,
                          mcu_protocol.REQ_MOTOR_SET_VEL, vel, rot)
        req += chr(append_checksum(req))
        g_request_queue.append(req)

    elif data.linear.z == 1.0 and data.linear.y == 0.0:
        left = int(data.linear.x * MAX_VEL / 2)
        right = int(data.angular.z * MAX_VEL / 2)
        #print left, right
        req = struct.pack('<BBBhh', mcu_protocol.HEADER, 0x05,
                          mcu_protocol.REQ_MOTOR_SET_VEL, right, left)
        req += chr(append_checksum(req))
        g_request_queue.append(req)


def actuator():
    serial_handle = serial.Serial("/dev/MotorMCU", 115200, timeout=1.0)
    rospy.init_node("actuator", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    print 'init complete'
    #motor enable
    req = struct.pack('<BBBB', mcu_protocol.HEADER, 0x02,
                      mcu_protocol.REQ_MOTOR_ONOFF,
                      mcu_protocol.REQ_MOTOR_OPTION_ON)
    req += chr(append_checksum(req))

    serial_handle.write(req)
    print 'motor enable sent'
    reader = mcu_reader.MCUReader(serial_handle)
    reader.start()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        req = struct.pack('<BBB', mcu_protocol.HEADER, 0x01,
                          mcu_protocol.REQ_JOY)
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
    req = struct.pack('<BBBB', mcu_protocol.HEADER, 0x02,
                      mcu_protocol.REQ_MOTOR_ONOFF,
                      mcu_protocol.REQ_MOTOR_OPTION_OFF)
    req += chr(append_checksum(req))
    serial_handle.write(req)
    reader.finalize = True
    reader.join()

if __name__ == "__main__":
    actuator()
