#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

RATE = 10
LIN_VEL = 0.3
SAFETY_THRESHOLD = 0.3


class RobotState:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.rate = rospy.Rate(RATE)
        self.state = 0 # 0: stopped, 1: move forward

    def scan_callback(self, msg):
       d = min(msg.ranges)
       print (d)
       if d < SAFETY_THRESHOLD:
           self.state = 0
           return
       self.state = 1

    def spin(self):
       vel_msg = Twist()
       while not rospy.is_shutdown():
           if self.state == 0:
               vel_msg.linear.x = 0
           elif self.state == 1:
               vel_msg.linear.x = LIN_VEL 
           self.cmd_pub.publish(vel_msg)
           
           self.rate.sleep() 

if __name__ == "__main__":
    rospy.init_node("fsm_node") 
    r = RobotState()
    r.spin()
