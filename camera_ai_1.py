#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ShowImage(object):
	
	def __init__(self):

		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
		self.bridge_object = CvBridge()

	def camera_callback(self, data):
		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print (e)
			
		cv2.imshow('image', cv_image)
		cv2.waitKey(1)


def main():
	show_image_object = ShowImage()
	rospy.init_node("show_image_node", anonymous=True)
	rospy.spin()
	

if __name__ == '__main__':
	main()
