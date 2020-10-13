#!/usr/bin/env python


import cv2
import os

cap = cv2.VideoCapture('/home/husarion/ros_workspace/src/udo_package/image_rec/udo.mp4')
cap.set(3,640)
cap.set(4,480)
while True:
	success, img = cap.read()
	cv2.imshow('video', img)
	cv2.waitKey(1)


