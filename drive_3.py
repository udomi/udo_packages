#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

linear_velocity = 0.3
turn_velocity = 0.3
backup_distance = 0.3


def callback(msg):
	print (msg.range)

	if msg.range > backup_distance:
		move.linear.x = linear_velocity/5
		move.angular.z = 0.0
	
	if msg.range < backup_distance:
		move.linear.x = -linear_velocity/10
		move.angular.z = turn_velocity

	pub.publish(move)

rospy.init_node('sub_node')
sub = rospy.Subscriber('/range/fl', Range, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()
