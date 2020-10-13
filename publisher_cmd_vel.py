#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rospy.init_node('udo_node', anonymous=True)
	r = rospy.Rate(10) #10Hz
	var = Twist()
	while not rospy.is_shutdown():
		var.linear.x = 0.1
		var.angular.z = 0.5
		rospy.loginfo(var)
		pub.publish(var)
		r.sleep()

if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException: pass
