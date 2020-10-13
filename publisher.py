#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher("udo_topic", String, queue_size=10)
	rospy.init_node('udo_node', anonymous=True)
	r = rospy.Rate(10) #10Hz
	while not rospy.is_shutdown():
		str = "hello udo %s"%rospy.get_time()
		rospy.loginfo(str)
		pub.publish(str)
		r.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass

