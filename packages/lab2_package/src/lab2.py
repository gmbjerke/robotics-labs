#!/usr/bin/python3

# Griffin Bjerke
import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped

class driveSq:
	def __init__(self):
		self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
		self.my_msg = Twist2DStamped()
		
	def send_motor_msg(self, vel, angVel):
		self.my_msg.v = vel
		self.my_msg.omega = angVel
		self.pub.publish(self.my_msg)
		
if __name__ == '__main__':
	try:
		driveSquare = driveSq()
		rospy.init_node('lab2', anonymous=True)
		
		time_straight = 4.2
		time_rot = 0.5
		run_vel = 0.3
		vel_omega = 2
		
		for i in range(12):
			# drive forward
			driveSquare.send_motor_msg(run_vel, 0)
			rospy.sleep(time_straight)
			# stop motors and wait 5 seconds
			driveSquare.send_motor_msg(0,0)
			rospy.sleep(5.0)
			# begin turn
			driveSquare.send_motor_msg(0, vel_omega)
			rospy.sleep(time_rot)
			# stop motors and wait for 0.25 seconds
			driveSquare.send_motor_msg(0,0)
			rospy.sleep(0.25)
	except rospy.ROSInteruptException:
		pass
		
