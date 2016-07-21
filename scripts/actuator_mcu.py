#!/usr/bin/env python
import serial

import rospy
from geometry_msgs.msg import Twist


import mcu_protocol
import mcu_reader

MAX_VEL = 450
REMOTE_MODE = 2
DIRECT_MODE = 1

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
    if data.linear.y == 2.0:
        g_mode = REMOTE_MODE
    elif data.linear.y == 1.0:
        g_mode = DIRECT_MODE

    if g_mode == REMOTE_MODE and data.linear.y == 2.0:
        print 'joy'
        vel = int(data.linear.x * MAX_VEL * 2 / 3 / 2 -
                  data.angular.z * MAX_VEL * 1 / 3 / 2)
        rot = int(data.linear.x * MAX_VEL * 2 / 3 / 2 +
                  data.angular.z * MAX_VEL * 1 / 3 / 2)

        req = mcu_protocol.make_motor_set_vel(vel, rot)
        g_request_queue.append(req)

        return

    if g_mode != DIRECT_MODE:
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

        req = mcu_protocol.make_motor_set_vel(vel, rot)
        g_request_queue.append(req)

    elif data.linear.z == 1.0 and data.linear.y == 0.0:
        left = int(data.linear.x * MAX_VEL / 2)
        right = int(data.angular.z * MAX_VEL / 2)
        #print left, right
        req = mcu_protocol.make_motor_set_vel(right, left)
        g_request_queue.append(req)


def actuator():
    serial_handle = serial.Serial("/dev/MotorMCU", 115200, timeout=1.0)
    rospy.init_node("actuator", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    print 'init complete'
    #motor enable
    req_motor_on = mcu_protocol.make_motor_on()

    serial_handle.write(req_motor_on)
    print 'motor enable sent'
    reader = mcu_reader.MCUReader(serial_handle)
    reader.start()
    rate = rospy.Rate(100)

    req_joy = mcu_protocol.make_req_joy()

    while not rospy.is_shutdown():
        g_request_queue.append(req_joy)
        if len(g_request_queue) > 3000:
            print '!!!!!!!!!request queue is too large.. Exitting'
            break

        while len(g_request_queue) != 0:
            req = g_request_queue.pop(0)
            serial_handle.write(req)

        rate.sleep()

    #motor disable
    req_motor_off = mcu_protocol.make_motor_off()
    serial_handle.write(req_motor_off)
    reader.finalize = True
    reader.join()

if __name__ == "__main__":
    actuator()
