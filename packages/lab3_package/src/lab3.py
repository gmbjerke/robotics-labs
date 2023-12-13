#!/usr/bin/python3
# Griffin Bjerke


import sys, math
import numpy as np
import rospy
import cv2

from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped, FSMState

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge

class laneDetector:

	
	def __init__(self):
		self.bridge = CvBridge()
		#self.running = 0
		# subscribe to compressed camera image
		rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.handle_camera_img, queue_size=1, buff_size=2**24)
		# create publishers
		self.pub_yellow = rospy.Publisher("/image_lines_yellow", Image, queue_size=10)
		self.pub_white = rospy.Publisher("/image_lines_white", Image, queue_size=10)
		self.pub_lines_all = rospy.Publisher("/image_lines_all", Image, queue_size=10)



	def handle_camera_img(self, msg):
		image_size = (160, 120)
		offset = 40
		
		# extract compressed image
		cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
		rospy.loginfo("COMPRESSED IMAGE EXTRACTED")
		# cropping
		new_image = cv2.resize(cv_image, image_size, interpolation=cv2.INTER_NEAREST)
		image_cropped = new_image[offset:, :]
		image_gray = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2GRAY)
		image_edges = cv2.Canny(image_gray, 50, 200)
		
		image_hsv = cv2.cvtColor(image_cropped, cv2.COLOR_RGB2HSV)
		
		image_white = self.filter_white(image_hsv)
		
		rospy.loginfo("image_white created")
		
		image_yellow = self.filter_yellow(image_hsv)				
		self.image_processing(image_cropped, image_edges, image_white, image_yellow)
		
		# FINISHed?

	def image_processing(self, image_cropped, image_edges, image_white, image_yellow):

		
		and_white = cv2.bitwise_and(image_edges, image_white, mask=None)
		and_yellow = cv2.bitwise_and(image_edges, image_yellow, mask=None)
		
		hough_yellow = cv2.HoughLinesP(and_yellow, 1, np.pi/180, 1, 1, 2)
		hough_white = cv2.HoughLinesP(and_white, 1, np.pi/180, 1, 1, 2)
		
		image_lines_white = self.output_lines(image_cropped, hough_white)
		image_lines_yellow = self.output_lines(image_cropped, hough_yellow)
		image_lines_all = self.output_lines(image_lines_yellow, hough_white)
		
		# convert images back
		yellow_img_msg = self.bridge.cv2_to_imgmsg(image_lines_yellow, encoding="rgb8")
		self.pub_yellow.publish(yellow_img_msg)
		white_img_msg = self.bridge.cv2_to_imgmsg(image_lines_white, encoding="rgb8")
		self.pub_white.publish(white_img_msg)
		lines_all_img_msg = self.bridge.cv2_to_imgmsg(image_lines_all, encoding="rgb8")
		self.pub_lines_all.publish(lines_all_img_msg)
		
		
		#FINISHED
		
	def output_lines(self, original_image, lines):
		output = np.copy(original_image)
		if lines is not None:
			for i in range(len(lines)):
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
	#finished
	def filter_white(self, cv_image):
		n = 75
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		
		white_lower = np.array([0,0,255 - n], dtype=np.uint8)
		white_upper = np.array([255, n, 255], dtype=np.uint8)
		white_image = cv2.inRange(cv_image, white_lower, white_upper)
		
		white_image = cv2.erode(white_image, kernel)
		white_image = cv2.dilate(white_image, kernel)
		
		return white_image
	def filter_yellow(self, cv_image):
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		
		yellow_lower = np.array([10, 20, 0], dtype=np.uint8)
		yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
		yellow_image = cv2.inRange(cv_image, yellow_lower, yellow_upper)
		
		yellow_image = cv2.erode(yellow_image, kernel)
		yellow_image = cv2.dilate(yellow_image, kernel)
		
		return yellow_image
if __name__ == '__main__':
	try: 
		rospy.init_node('lab3', anonymous=True)
		laneDetector()
		rospy.spin()
	except rospy.ROSInteruptException:
		pass
		
