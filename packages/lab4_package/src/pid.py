#!/usr/bin/python3
# Griffin Bjerke

import sys, math
import numpy as np
import rospy

from collections import deque

class PID:
	p = 0.7 # Proportional Gain
	i = 1.0 # Integral Gain
	d = 1.0 # Derivative Gain
	
	error_count = 0.0
	
	def __init__(self, K_p, K_i, K_d):
		
		self.p = K_p
		self.i = K_i
		self.d = K_d
		
		self.error_queue = deque([], maxlen=2)
		self.error_count = 0.0
		
	def increment_error(self, error):
		time = rospy.get_rostime().to_sec()
		self.error_queue.append((error, time))
		self.error_count += error
		
		if(len(self.error_queue) > 1):
			error_diff = self.error_queue[1][0] - self.error_queue[0][0]
			time_diff = self.error_queue[1][1] - self.error_queue[0][1]
			error_der = error_diff/time_diff
			return [error, self.error_count, error_der]
		else:
			return [error, error, 0]
	def get_control_signal(self, error):
		p_error, i_error, d_error = self.increment_error(error)
		
		K_p = self.p 
		K_i = self.i
		K_d = self.d
		control_signal = (K_p * p_error) + (K_i * i_error) + (K_d * d_error)
		return control_signal
		
