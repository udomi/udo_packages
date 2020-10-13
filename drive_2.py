#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

turn_velocity = 0.2
linear_velocity = 0.3
first_turn_distance = 1
second_turn_distance = 0.4

def callback(msg):
	print len(msg.ranges)

	if msg.ranges[359] > first_turn_distance:
		move.linear.x = linear_velocity
		move.angular.z = 0.0
	
	if msg.ranges[359] < 1:
		move.linear.x = linear_velocity/10
		move.angular.z = turn_velocity

	if msg.ranges[180] < second_turn_distance:
		move.linear.x = linear_velocity/10
		move.angular.z = -turn_velocity

	if msg.ranges[0] < second_turn_distance:
		move.linear.x = linear_velocity/10
		move.angular.z = turn_velocity

	pub.publish(move)

rospy.init_node('sub_node')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()
