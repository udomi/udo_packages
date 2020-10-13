#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LoadImage(object):
    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        face_cascade = cv2.CascadeClassifier('/home/husarion/ros_workspace/src/udo_package/haar_cascades/frontalface.xml')
        eyes_cascade = cv2.CascadeClassifier('/home/husarion/ros_workspace/src/udo_package/haar_cascades/eye.xml')
        
        img = cv_image
	#img = cv2.imread('/home/husarion/ros_workspace/src/udo_package/image_rec/chris.jpg')
        img = cv2.resize(img,(350,550))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        face = face_cascade.detectMultiScale(gray, 1.2, 3)
	print(face)
        
        for (x,y,w,h) in face:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            
            eyes = eyes_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color, (ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                
		cv2.imwrite('/home/husarion/ros_workspace/src/udo_package/image_rec/eustakio.jpg',img)      
           
        cv2.imshow('image',img)
	        
        cv2.waitKey(1)

def main():
    load_image_object = LoadImage()
    rospy.init_node('load_image_node', anonymous=True)
    try:
	rospy.spin()
    except KeyboardInterrupt:
	print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

      
