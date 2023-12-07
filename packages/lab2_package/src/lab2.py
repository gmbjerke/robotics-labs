#!/usr/bin/python3

# Griffin Bjerke
import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped, FSMState

class driveSq:
	def __init__(self):
		self.running = "0"
		rospy.Subscriber("fsm_node/mode", FSMState, self.callback)
		
		self.pub = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
#		self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
		self.my_msg = Twist2DStamped()
	
	def send_motor_msg(self, vel, angVel):
		self.my_msg.v = vel
		self.my_msg.omega = angVel
		self.pub.publish(self.my_msg)
		
	def callback(self, msg):
		time_straight = 2.25
		time_rot = 0.7
		run_vel = 0.5
		vel_omega = 5
		
		if msg.state == "LANE_FOLLOWING" and self.running == "0":
			print("REACHED START OF TURN=====================")
			self.running = "1"
			if self.running == "1":
				rospy.sleep(5.0)	
				for i in range(4):
					# rospy.sleep(5.0)
					# drive forward
					self.send_motor_msg(run_vel, 0)
					rospy.sleep(time_straight)
					# stop motors and wait 5 seconds
					self.send_motor_msg(0,0)
					rospy.sleep(5.0)
					# begin turn
					self.send_motor_msg(0, vel_omega)
					rospy.sleep(time_rot)
					# stop motors and wait for seconds
					self.send_motor_msg(0,0)
					rospy.sleep(1.0)
				self.running == "0"
				
		self.running = "0"
if __name__ == '__main__':
	try:
		
		driveSquare = driveSq()
		rospy.init_node('lab2', anonymous=True)
		rospy.spin()
	except rospy.ROSInteruptException:
		pass
		
