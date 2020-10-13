#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class LoadImage(object):
	
	def __init__(self):

		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
		self.bridge_object = CvBridge()

	def camera_callback(self, data):
		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print (e)
		



def main():
		load_image_object = LoadImage()
		rospy.init_node("load_image_node", anonymous=True)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("shutting down")
		cv2.destroyAllWindows()
		

if __name__ == '__main__':
	main()
