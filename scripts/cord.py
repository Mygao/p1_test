#!/usr/bin/env python

import rospy

import message_filters

from geometry_msgs.msg import Twist
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


class VelocityReducer():
    VEL_STOP_THRESHOLD = 0.2
    VEL_AVOID_THRESHOLD = 0.6

    sum_weight = [-0.1, -0.2, -0.3, -0.4, 0.4, 0.3, 0.2, 0.1]

    def __init__(self, pub):
        self.pub = pub
        self.subs = []
        self.sonar_range_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_mod = Twist()
        self.is_emergency = False
        self._velocity_floor = 1.0

    def addSubscriber(self, sub):
        self.subs.append(sub)

    def callbackVelocity(self, data):
        self.vel_mod.linear.x = 0.0
        self.vel_mod.angular.z = 0.0

        final_vel = Twist()

        if self.is_emergency is True:
            print "Emergency!!"
            final_vel.linear.x = data.linear.x
            final_vel.angular.z = data.angular.z

            """if forward velocity is set, ignore"""
            if data.linear.x > 0:
                final_vel.linear.x = 0

        else:
            """calculate mod vel"""
            self._calculateModVel()
            final_vel.linear.x = data.linear.x + self.vel_mod.linear.x
            final_vel.angular.z = data.angular.z + self.vel_mod.angular.z


        if final_vel.linear.x > 1:
            final_vel.linear.x = 1
        if final_vel.angular.z > 1:
            final_vel.angular.z = 1
        elif final_vel.angular.z < -1:
            final_vel.angular.z = -1

        self.pub.publish(final_vel)

    def callbackSonar(
            self, data1, data2, data3, data4, data5, data6, data7, data8):
        args = locals()
        self.is_emergency = False
        for i in xrange(len(self.sonar_range_data)):
            self.sonar_range_data[i] = 0.0

        for i, (k, v) in enumerate(args.items()):
            if isinstance(v, Range) is not True:
                continue

            self.sonar_range_data[sonar_topics.index(v.header.frame_id)] = v.range

            if v.range < self.VEL_STOP_THRESHOLD and v.range > 0.0:
                self.is_emergency = True

    def _calculateModVel(self):
        sum_of_data = 0
        for elem, w in zip(self.sonar_range_data, self.sum_weight):
            if elem < self.VEL_AVOID_THRESHOLD:
                sum_of_data += (-2.5 * elem + 1.5) * w

        self.vel_mod.angular.z = sum_of_data

        return self.vel_mod


def cord():
    rospy.init_node("cord", anonymous=True)
    pub = rospy.Publisher("/cord/cmd_vel", Twist, queue_size=10)
    reducer = VelocityReducer(pub)
    rospy.Subscriber("cmd_vel", Twist, reducer.callbackVelocity)

    for i in range(0, 8):
        reducer.addSubscriber(message_filters.Subscriber(
            sonar_topics[i], Range))

    ats = message_filters.ApproximateTimeSynchronizer(
        reducer.subs, 10, 0.1)
    ats.registerCallback(reducer.callbackSonar)

    rospy.spin()


if __name__ == "__main__":
    cord()
