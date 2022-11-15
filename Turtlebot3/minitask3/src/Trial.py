#!/usr/bin/env python

from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
from geometry_msgs.msg import Twist

class Follower:
   	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("original", 1)
		cv2.namedWindow("masked", 1)
		cv2.namedWindow("final", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
		self.twist = Twist()
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)		
		self.green_beacon_found = False

   	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

		#Original Image		
		(h, w) = image.shape[:2]
		image_resized = cv2.resize(image, (w/4,h/4))

		#Mask image to find green beacon
		hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)   
		low_green = np.array([25, 52, 72])
		high_green = np.array([102, 255, 255])
		green_mask = cv2.inRange(hsv, low_green, high_green)
		green = cv2.bitwise_and(image_resized, image_resized, mask=green_mask)
		h, w, d = image_resized.shape
		contours = []

		ret, thresh = cv2.threshold(green_mask, 50, 255, 0)
		(_, contours, _)= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		def get_contour_areas(contours):
		    all_areas= []
		    for cnt in contours:
			area= cv2.contourArea(cnt)
			all_areas.append(area)
		    return all_areas
		
		if len(contours) is 0:
			print("No green beacon found")
		else:
			if self.green_beacon_found is False:
				largest_contour= max(contours, key = cv2.contourArea)		

				min_height = int(min(largest_contour[:,:,1]))
				max_height = int(max(largest_contour[:,:,1]))
				min_width = int(min(largest_contour[:,:,0]))
				max_width = int(max(largest_contour[:,:,0]))
					
				largest_object = green_mask
				largest_object[0:min_height, :] = 0
				largest_object[max_height:h, :] = 0
				largest_object[:, 0:min_width] = 0
				largest_object[:, max_width:w] = 0
			else:
				largest_object = green_mask
				largest_object[:, 0:w/2 - 25] = 0
				largest_object[:, w/2 +25:w] = 0
	


			M = cv2.moments(largest_object)
			if M['m00'] > 0:
				#Centroid formula
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(green, (cx, cy), 8, (0,0,255), -1)
				
			if 0 in green[:, 0:w/2 - 20, 0:1] and 255 in green[:, 0:w/2 - 20, 2]:
				print("In Left")
				self.twist.linear.x = 0
				self.twist.angular.z = 0.1
			elif 0 in green[:, w/2 +20:w, 0:1] and 255 in green[:, w/2 +20:w, 2]:
				print("In Right")
				self.twist.linear.x = 0
				self.twist.angular.z = -0.1
			else:
				print("Move")
				self.twist.linear.x = 0.2
				self.twist.angular.z = 0
				self.green_beacon_found = True
				
			self.pub.publish(self.twist)
		
			cv2.imshow("final", largest_object)
		
		#Show Original image and masked image
		cv2.imshow("original", image_resized)
		cv2.imshow("masked", green)	
		cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
#try:
#	rospy.spin()
#except KeyboardInterrupt:
#	print("Shutting Down")
#cv2.destroyAllWindows()


