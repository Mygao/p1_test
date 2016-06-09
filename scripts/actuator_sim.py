#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
#from gazebo_msgs.srv import *


def callback(data):
    rospy.loginfo(
        rospy.get_caller_id() + "\nReceived cmd: %s\n %s",
        data.linear, data.angular)
#    aje = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
#    st = rospy.Time()
#    st.secs = 0
#    st.nsecs = 0
#    dr = rospy.Duration()
#    dr.secs = 10
#    dr.nsecs = 0
#    res = aje("left_wheel_hinge", data.linear.x, st, dr)
#    res = aje("right_wheel_hinge", data.linear.x, st, dr)


def actuator():
#    rospy.wait_for_service('/gazebo/apply_joint_effort')
    rospy.init_node("actuator", anonymous=True)
    rospy.Subscriber("/cord/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    actuator()
