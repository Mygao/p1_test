#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose 

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s", data.position, data.orientation)

def listener():

	rospy.init_node('listenerrpi', anonymous=True)

	rospy.Subscriber("p1_pose", Pose, callback)

	rospy.spin()

if __name__ == "__main__":
	listener()
