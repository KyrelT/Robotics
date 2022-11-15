#!/usr/bin/env python2

import rospy
import math
import random
import numpy as np
import cv2, cv_bridge
import matplotlib.pyplot as plt
import tf
from numpy import inf, array
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class Turtlebot:
	def main(self):
    		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
    		self.pose = Pose2D()
    		self.trajectory = list()
    		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
    		self.obstacle_found = False
    		self.first_run = True
    		self.msg = Twist()
		self.green_beacon_found = False
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("original", 1)
		cv2.namedWindow("masked", 1)
		#cv2.namedWindow("final", 1)
    		rospy.spin()


	def callback(self, msg):

    		msg.ranges = array(msg.ranges)
    		msg.ranges[msg.ranges == 0] = inf
    		msg.ranges = tuple(msg.ranges)

    		# region is define from 0 to 360, like a circle
    		regions = {
        		'frontleft': min(min(msg.ranges[0:44]), 10),
        		'leftright': min(min(msg.ranges[45:89]), 10),
        		'leftleft': min(min(msg.ranges[90:134]), 10),
        		'backright': min(min(msg.ranges[135:179]), 10),
        		'backleft': min(min(msg.ranges[180:224]), 10),
        		'rightright': min(min(msg.ranges[225:269]), 10),
        		'rightleft': min(min(msg.ranges[270:314]), 10),
        		'frontright': min(min(msg.ranges[315:359]), 10),
    			}

    		self.obstacles_avoid(regions)


	def obstacles_avoid(self, regions):
	    linear_x = 0
	    angular_z = 0
	    state_description = ''

	    if regions['frontleft'] < 0.5 and regions['frontright'] > 0.5:
		state_description = 'obstacles detected (front left)'
		linear_x = 0
		angular_z = -0.3
		self.obstacle_found = True
	    elif regions['frontleft'] > 0.5 and regions['frontright'] < 0.5:
		    state_description = 'obstacles detected (front right)'
		    linear_x = 0
		    angular_z = 0.3
		    self.obstacle_found = True
	    elif regions['frontleft'] < 0.5 and regions['frontright'] < 0.5:
		state_description = 'obstacles detected (front)'
		linear_x = 0
		angular_z = 0.3
		self.obstacle_found = True
	    else:
		if self.green_beacon_found is False:
			state_description = 'performing random walk and detecting for green beacon'
		else:
			state_description = 'Moving towards green beacon'
		linear_x = 0.3
		angular_z = 0
		self.random_walk()
		self.obstacle_found = False

	    print(state_description)
	    self.msg.linear.x = linear_x
	    self.msg.angular.z = angular_z
	    self.pub.publish(self.msg)


	def random_walk(self):
		global x_init
		global y_init
		global a_init
		if self.first_run is True or self.obstacle_found is True or self.green_beacon_found is True: # init everything once it stops
			x_init = self.pose.x
			y_init = self.pose.y
			a_init = self.pose.theta
			self.first_run = False
		if self.green_beacon_found is False:
			if math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2) >= 3:
				random_angle = math.pi * 2 * random.random()
				print("3m limit, randomise direction, angle: " + str(random_angle))
				if random_angle < math.pi:
					while (max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle):
						if (max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle):
							self.msg.angular.z = 0
							self.pub.publish(self.msg)
						else:
							self.msg.angular.z = 0.3
							self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from " + str(a_init) + " -> " + str(random_angle) )
					self.first_run = True
				else:
					while(max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle/2):
						if(max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle/2):
					    		self.msg.angular.z = 0
					    		self.pub.publish(self.msg)
						else:
					    		self.msg.angular.z = 0.3
					    		self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from "+ str(a_init) + " -> " + str(random_angle))
					a_init = self.pose.theta
					while (max(self.pose.theta - a_init, a_init - self.pose.theta) < random_angle/2):
						if(max(self.pose.theta - a_init, a_init - self.pose.theta) > random_angle/2):
					    		self.msg.angular.z = 0
					    		self.pub.publish(self.msg)
						else:
							self.msg.angular.z = 0.3
							self.msg.linear.x = 0
						self.pub.publish(self.msg)
						print("rotating angle from "+ str(a_init) + " -> " + str(random_angle) )
					self.first_run = True


	def odom_callback(self, msg):
	    # get pose = (x, y, theta) from odometry topic
	    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
	    self.pose.theta = yaw
	    self.pose.x = msg.pose.pose.position.x
	    self.pose.y = msg.pose.pose.position.y

	    self.trajectory.append([self.pose.x, self.pose.y])


	def visualise(self):
		_, ax = plt.subplots(1)
		ax.set_aspect('equal')

		self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
		plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], linewidth=2)

		plt.xlim(-3, 3)
		plt.ylim(-3, 3)
		plt.show()


	def plot(self):
		np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


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
				self.msg.linear.x = 0
				self.msg.angular.z = 0.1
			elif 0 in green[:, w/2 +20:w, 0:1] and 255 in green[:, w/2 +20:w, 2]:
				print("In Right")
				self.msg.linear.x = 0
				self.msg.angular.z = -0.1
			else:
				self.msg.linear.x = 0.2
				self.msg.angular.z = 0
				self.green_beacon_found = True
			
			if self.obstacle_found is False:	
				self.pub.publish(self.msg)
		
			#cv2.imshow("final", largest_object)
		
		#Show Original image and masked image
		cv2.imshow("original", image_resized)
		cv2.imshow("masked", green)	
		cv2.waitKey(3)


if __name__ == '__main__':
	rospy.init_node('reading_laser')
        a = Turtlebot()
	while not rospy.is_shutdown():
		a.main()
		a.plot()
	a.visualise()
