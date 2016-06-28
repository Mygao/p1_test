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
                "p1_sonar_8"]


class VelocityReducer():
    VEL_STOP_THRESHOLD = 0.2
    VEL_AVOID_THRESHOLD = 0.6

    sum_weight = [-0.05, -0.1, -0.35, -0.4, 0.4, 0.35, 0.1, 0.05]

    def __init__(self, pub):
        self.pub = pub
        self.subs = []
        self.sonar_range_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_mod = Twist()
        self.vel_des = Twist()
        self.is_emergency = False
        self._velocity_floor = 1.0

    def addSubscriber(self, sub):
        self.subs.append(sub)

    def callbackVelocity(self, data):
        final_vel = Twist()
        
        final_vel.linear.x = data.linear.x
        final_vel.angular.z = data.angular.z
        if self.is_emergency is True:
            print "Emergency!!"

            """if forward velocity is set, ignore"""
            if data.linear.x > 0:
                final_vel.linear.x = 0

        #else:
        print data.linear.x
        print final_vel.linear.x
        self.vel_des.linear.x = final_vel.linear.x
        self.vel_des.angular.z = final_vel.angular.z
        

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
            if elem < self.VEL_AVOID_THRESHOLD and elem > 0.0:
                sum_of_data += (-2.5 * elem + 1.5) * w

        self.vel_mod.angular.z = 8 * sum_of_data
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

    #rospy.spin()
    rate = rospy.Rate(10)
    
    
    final_vel = Twist()
    while not rospy.is_shutdown():
        
        reducer.vel_mod.linear.x = 0.0
        reducer.vel_mod.angular.z = 0.0

        """calculate mod vel"""
        reducer._calculateModVel()
        final_vel.linear.x = reducer.vel_des.linear.x + reducer.vel_mod.linear.x
        final_vel.angular.z = reducer.vel_des.angular.z + reducer.vel_mod.angular.z


        if final_vel.linear.x > 1.0:
            final_vel.linear.x = 1.0

        if final_vel.angular.z > 1.0:
            final_vel.angular.z = 1.0
        elif final_vel.angular.z < -1.0:
            final_vel.angular.z = -1.0

        reducer.pub.publish(final_vel)
        
        rate.sleep()

if __name__ == "__main__":
    cord()
