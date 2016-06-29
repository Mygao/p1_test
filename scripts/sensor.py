#!/usr/bin/env python

import rospy
import struct
import serial

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

sonar_topics = ["p1_sonar_1",
                "p1_sonar_2",
                "p1_sonar_3",
                "p1_sonar_4",
                "p1_sonar_5",
                "p1_sonar_6",
                "p1_sonar_7",
                "p1_sonar_8",
                "p1_sonar_9",
                "p1_sonar_10",
                "p1_sonar_11",
                "p1_sonar_12",
                "p1_sonar_13",
                "p1_sonar_14"]

HEADER = 0xA5
REQ_SONAR = 0x11
RES_SONAR = 0x91
REQ_JOY = 0x42
RES_JOY = 0xC2

def request(req_val, serial_handle):
     #little endian '<'
    #big endian '>' 
    req = struct.pack('<BBBB', HEADER, 0x01, req_val, HEADER^0x01^req_val)
    serial_handle.write(req)
    data = serial_handle.read(1)
    print len(data)
    print '%x' %(ord(data[0]))
    header = ord(data[0])
    #header = struct.unpack('<B', data)
    if header != HEADER:
        print 'header is unmatching'

    data = serial_handle.read(1)
    
    if len(data) == 0:
	print 'data length is 0'
	return "em"

    #length = struct.unpack('<B', data[0])
    length = ord(data[0])
    print length

    data_stripped = serial_handle.read(length + 1)
    print len(data_stripped)
    command, = struct.unpack('<B', data_stripped[0])
    print command
    print data_stripped
    return command, data_stripped

def sensor():
    serial_handle = serial.Serial("/dev/MotorMCU", 115200, timeout=1.0)
    pub_sonar = []

    for topic in sonar_topics:
        pub_sonar.append(rospy.Publisher(topic, Range, queue_size=10))

        rospy.init_node("sensor", anonymous=True)

    pub_joy = rospy.Publisher('joy', Twist, queue_size = 1)

    rate = rospy.Rate(10)
    range_msg = Range()
    range_msg.field_of_view = 0.05
    range_msg.min_range = 0.0
    range_msg.max_range = 5.0

    while not rospy.is_shutdown():

#        command, data_stripped = request(RES_SONAR, serial_handle)
	command = "em"
    	if command == RES_SONAR:
            channel_count = struct.unpack('<B', data_stripped[1])
            sonar_data = [] 
            for i in range(channel_count):
                sonar_data[i] = struct.unpack('<H', data_stripped[2+i:2+i+2])

            i = 0
            print("\033c")

            for elem in sonar_data:
                range_msg.header.frame_id = sonar_topics[i]
                range_msg.range = elem / 1000.0
                if elem == 0:
                    range_msg.range = 5.0

                pub_sonar[i].publish(range_msg)
    #                rospy.loginfo(range_msg.range)
                bar = ""
                if int(elem) == 0:
                    for k in range(1, 100):
                        bar += "*"

                for j in range(1, int(elem) / 20):
                    bar += "-"
                    if j > 100:
                        break

                rospy.loginfo(bar)
                i = i + 1	

        command, data_stripped = request(REQ_JOY, serial_handle)

        if command == RES_JOY:
            vel, rot, = struct.unpack('<bb', data_stripped[1:3])
            print 'Joy: %d, %d' % (vel, rot)

            vel = vel / 100
            rot = rot / 100

        rate.sleep()


if __name__ == "__main__":
    sensor()
