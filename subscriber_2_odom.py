#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
	rospy.loginfo(rospy.get_caller_id()+" Odom %s", data.pose.pose)
	
def listener():
	rospy.init_node('udo_odom_listener_node', anonymous=True) 
	rospy.Subscriber('/odom', Odometry, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
