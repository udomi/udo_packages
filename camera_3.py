#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LoadImage(object):
	
	def __init__(self):

		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
		self.bridge_object = CvBridge()

	def camera_callback(self, data):
		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print (e)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
		minLineLength = 500
		maxLineGap = 50
		lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength, maxLineGap)
		for x1,y1,x2,y2 in lines[0]:
			cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),5)

		cv2.imshow('gray', gray)		
		cv2.imshow('edges',edges)		
		cv2.imshow('image',cv_image)
		cv2.waitKey(1)




def main():
		load_image_object = LoadImage()
		rospy.init_node("load_image_node", anonymous=True)
		rospy.spin()
		
		
		

if __name__ == '__main__':
	main()
