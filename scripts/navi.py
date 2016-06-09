#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose

def navi():
	pub = rospy.Publisher("p1_pose", Pose, queue_size=50)
	rospy.init_node("navi", anonymous=True)
	rate = rospy.Rate(1)
    
	pose = Pose()
	i = 0.0

	while not rospy.is_shutdown():
		pose.position.x = i
		pose.position.y = i
		rospy.loginfo(pose)
		pub.publish(pose)
		i = i + 0.1
		rate.sleep()
	
if __name__ == "__main__":
	navi()
