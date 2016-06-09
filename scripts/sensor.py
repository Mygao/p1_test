#!/usr/bin/env python

import rospy
import struct
import serial

from sensor_msgs.msg import Range
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


def sensor():
    serial_handle = serial.Serial("/dev/cp210x", 115200, timeout=0.1)
    pub_sonar = []

    for topic in sonar_topics:
        pub_sonar.append(rospy.Publisher(topic, Range, queue_size=10))

        rospy.init_node("sensor", anonymous=True)

    rate = rospy.Rate(10)
    range_msg = Range()
    range_msg.field_of_view = 0.05
    range_msg.min_range = 0.0
    range_msg.max_range = 5.0

    while not rospy.is_shutdown():
        serial_handle.write("\xfc")
        data = serial_handle.read(100)
        sonar_data_raw = data[2:18]
        if len(sonar_data_raw) < 16:
            continue

        sonar_data = struct.unpack('hhhhhhhh', data[2:18])

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

        rate.sleep()


if __name__ == "__main__":
    sensor()
