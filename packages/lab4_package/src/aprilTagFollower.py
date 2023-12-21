#!/usr/bin/python3
# Griffin Bjerke

import sys, math
import numpy as np
import rospy

from pid import PID

from geometry_msgs.msg import PoseStamped

from duckietown_msgs.msg import AprilTagDetectionArray

from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Lab4:
	fsm_mode = "fsm_node/mode"
	tag_detection = "apriltag_detector_node/detections"
	car_cmd = "car_cmd_switch_node/cmd"
	
	desired_yaw = 0.0
	desired_phi = 0.0
	desired_z_trans = 0.1
	min_angle_error = 0.3
	min_linear_error = 0.05
	#follow = False
	
	def __init__(self):
		self.angular_controller = PID(2.8, 0.01, 0.05)
		self.linear_controller = PID(1.25, 0.003, 0.06)
		
		#rospy.Subscriber(self.fsm_mode, FSMState, self.handle_fsm_mode)
		
		rospy.Subscriber(self.tag_detection, AprilTagDetectionArray, self.receive_tag)
		self.pub = rospy.Publisher(self.car_cmd, Twist2DStamped, queue_size=10)
		
	def receive_tag(self, msg):
	#	if(self.follow):
		
		detections = msg.detections
		
		if(len(detections) > 0):
			actual_x_trans = detections[0].transform.translation.x
			actual_z_trans = detections[0].transform.translation.z
			actual_yaw = detections[0].transform.rotation.y
			actual_phi = math.atan(actual_x_trans/actual_z_trans)
			
			rospy.loginfo("April Tag Depth: " + str(actual_z_trans) + "April Tag horizontal trans: " + str(actual_x_trans))
			
			depth_error = self.desired_z_trans - actual_z_trans
			phi_error = self.desired_phi - actual_phi
			yaw_error = self.desired_yaw - actual_yaw
			
			rospy.loginfo("Depth Error: " + str(depth_error))
			rospy.loginfo("Phi Error: " + str(phi_error))
			rospy.loginfo("Yaw Error: " + str(yaw_error))
			
			linear_error = depth_error
			angular_error = phi_error
			self.handle_errors(linear_error, angular_error)
		else:
			self.publish_car_cmd(0, 0)
			
	def publish_car_cmd(self, v, omega):
		output_cmd = Twist2DStamped()
		output_cmd.v = -v
		output_cmd.omega = omega
		
		self.pub.publish(output_cmd)
	
	def handle_errors(self, linear_error, angular_error):
		omega = 0.0
		v = 0.0
		if(abs(angular_error) > self.min_angle_error):
			omega = self.angular_controller.get_control_signal(angular_error)
		if(abs(linear_error) > self.min_linear_error):
			v = self.linear_controller.get_control_signal(linear_error)
		self.publish_car_cmd(v, omega)
		

if __name__ == '__main__':
	rospy.init_node('Lab4')
	Lab4()
	
	rospy.spin()
