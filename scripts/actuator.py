#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import serial
import threading
import struct

VELOCITY_MAX = 120
VELOCITY_MIN = -120


class ELMOReader(threading.Thread):
    def __init__(self, serial_handle):
        super(ELMOReader, self).__init__()
        self.serial_handle = serial_handle
        self.finalize = False

    def run(self):
        while self.finalize is False:
            print self.readEncoderVelocity()

    def readEncoderVelocity(self):
        return self.serial_handle.read(100)


class ELMOWriter():
    def __init__(self, port_name):
        self.velocity = 0
        self.port = None
        self.port_name = port_name

        self.port = serial.Serial('/dev/' + port_name, 19200, timeout=0.1)
        self.reader = ELMOReader(self.port)
        self.reader.start()
        self.port.write('UM=2\r\n')
        self.port.write('MO=1\r\n')

        self.stop()

    def __del__(self):
        if self.port:
            self.stop()
            self.port.write('MO=0\r\n')
            self.port.close()
            self.reader.finalize = True
            self.reader.join()

    def sendVelocityCommand(self):
        self.port.write('JV=%d\r\n' % (self.velocity * 1.5))
        self.port.write('BG\r\n')
#       print 'ELMOWriter:sendVelocityCommand():' + self.port_name +
#            ' ' + str(self.velocity)

    def requestEncoderVelocity(self):
        self.port.write('PX\r\n')

    def setAbsoluteVelocity(self, vel):
        self.velocity = vel

        if self.velocity > VELOCITY_MAX:
            self.velocity = VELOCITY_MAX
        if self.velocity < VELOCITY_MIN:
            self.velocity = VELOCITY_MIN

        self.sendVelocityCommand()

    def stop(self):
        self.velocity = 0
        self.port.write('ST\r\n')
        self.port.write('JV=0\r\n')
        print 'ELMOWriter:stop()'


def callback(data):
    if (abs(data.linear.x) < 0.09 and abs(data.angular.z) < 0.09):
        wheel_L.stop()
        wheel_R.stop()
        return
#   rospy.loginfo(rospy.get_caller_id() + "Received cmd: %s %s",
#        data.linear, data.angular)
    wheel_L.setAbsoluteVelocity(data.linear.x * 80.0 - data.angular.z * 40)
    wheel_R.setAbsoluteVelocity(data.linear.x * 80.0 + data.angular.z * 40)


def actuator():
    global wheel_L, wheel_R
    wheel_L = ELMOWriter('LEFT')
    wheel_R = ELMOWriter('RIGHT')

    rospy.init_node("actuator", anonymous=True)
    rospy.Subscriber("cord/cmd_vel", Twist, callback)
#    rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wheel_L.requestEncoderVelocity()
        wheel_R.requestEncoderVelocity()
        rate.sleep()

    wheel_L.stop()
    wheel_R.stop()

if __name__ == "__main__":
    actuator()
