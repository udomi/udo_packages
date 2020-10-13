#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

linear_speed = 0.5
angular_speed = 0


def callback_fl(data):
	print('fl', data.range)
	

def callback_rl(data_rl):
	print('rl', data_rl.range)
	move = Twist()
	if data_rl.range < 0.2:
		move.linear.x = 0.1
		move.angular.z = 0
	else:
		move.linear.x = linear_speed
		move.angular.z = angular_speed
	pub = rospy.Publisher('/cmd_vel', Twist)
	pub.publish(move)

def main():
	rospy.init_node('udo_drive_range_node', anonymous=True) 
	rospy.Subscriber('/range/fl', Range, callback_fl)
	rospy.Subscriber('/range/rl', Range, callback_rl)
	rospy.spin()

if __name__ == '__main__':
    	main()

