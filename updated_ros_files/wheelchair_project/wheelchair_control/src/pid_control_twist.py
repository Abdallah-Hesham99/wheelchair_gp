#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from simple_pid import PID
import numpy as np
import time
import RPi.GPIO as gpio

class controller:

	def __init__(self):
		self.wheel_diameter = 0.3
		self.center_dist = 0.61
		self.ppr = 20*32
		self.dist_per_tick = (np.pi*self.wheel_diameter)/self.ppr
		self.right_ticks_diff=0
		self.left_ticks_diff=0
		self.mean_s = 0
		self.dx = 0
		self.dy = 0
		self.dtheta = 0 
		self.x=0
		self.y=0
		self.theta = 0
		self.vx = 0
		self.vy=0
		self.w=0
		self.new_time =0
		self.old_time = rospy.Time.now().secs
		self.linear_target = 0
		self.angular_target = 0
		self.linear_pid = PID(0.1,0.1,0.01,setpoint=self.linear_target)
		self.linear_pid.output_limits = (-100,100)
		self.angular_pid = PID(1,1,0.01,setpoint=self.angular_target)
		self.angular_pid.output_limits = (-100,100)
		self.rsub = rospy.Subscriber("/wheel_encoders/right_ticks",UInt16,self.right_wheel_cb)
		self.lsub = rospy.Subscriber("/wheel_encoders/left_ticks",UInt16,self.left_wheel_cb)
		self.com_sub=rospy.Subscriber("cmd_vel",Twist,self.twist_callback)
		self.rate = rospy.Rate(10)
		self.motor1_pins = [21,20]
		self.motor2_pins = [7,1]
		self.pwm1 = 1
		self.pwm2 = 1
		self.setup_pins()
	def right_wheel_cb(self,msg):
		self.right_ticks_diff = msg.data
	
	def left_wheel_cb(self,msg):
		self.left_ticks_diff = msg.data
	def calc_speeds(self):
		
		sl = self.left_ticks_diff * self.dist_per_tick
		sr = self.right_ticks_diff * self.dist_per_tick
		self.mean_s= (sl+sr)/2.0
		
		self.dx = self.mean_s * np.cos(self.theta)
		self.dy = self.mean_s * np.sin(self.theta)
		self.dtheta = (sr-sl)/self.center_dist
		self.x+=self.dx
		self.y += self.dy
		self.theta += self.dtheta
		if self.theta>2*np.pi:
			self.theta -= 2*np.pi
		elif self.theta<2*np.pi:
			self.theta += 2*np.pi
		self.new_time = rospy.Time.now().secs
		self.dt = self.new_time-self.old_time
		self.vx = self.dx/self.dt
		self.vy = self.dy/self.dt
		self.w = self.dtheta/self.dt
		self.old_time = self.new_time
		
		
		
	def twist_callback(self,msg):
		self.linear_target = msg.linear.x
		self.angular_target = msg.angular.z
		
		
	def control(self):
		linear_output = self.linear_pid(self.vy)
		angular_output = self.angular_pid(self.w)
		vr = linear_output + angular_output*(self.wheel_diameter/2 + self.center_dist/2)
		vl = linear_output + angular_output*(self.wheel_diameter/2 - self.center_dist/2)
		
		if vr <0:
			vr = abs(vr)
			self.pwm1 = int(not self.pwm1)
		if vl < 0:
			vl = abs(vr)
			self.pwm2 = int(not self.pwm2)
		if self.pwm1:
			self.m11.ChangeDutyCycle(0)
			self.m12.ChangeDutyCycle(vr)
						
		else:
			self.m12.ChangeDutyCycle(0)
			self.m11.ChangeDutyCycle(vr)

						
		if self.pwm2:
			self.m21.ChangeDutyCycle(0)
			self.m22.ChangeDutyCycle(vl)
				
		else :
			self.m22.ChangeDutyCycle(0)
			self.m21.ChangeDutyCycle(vl)

		
		
		
		
		
	def setup_pins(self):
		gpio.setmode(gpio.BCM)
		gpio.setwarnings(False)
		for i in range(2):
			gpio.setup(self.motor1_pins[i],gpio.OUT)
			gpio.setup(self.motor2_pins[i],gpio.OUT)
		self.m11 = gpio.PWM(self.motor1_pins[0],1000)
		self.m11.start(0)
		self.m12 = gpio.PWM(self.motor1_pins[1],1000)
		self.m12.start(0)
		self.m21 = gpio.PWM(self.motor2_pins[0],1000)
		self.m21.start(0)
		self.m22 = gpio.PWM(self.motor2_pins[1],1000)
		self.m22.start(0)	
		print("all set")
		
	def run(self):
		self.calc_speeds()
		self.control()
		
		
if __name__=="__main__":
	c = controller()
	while rospy.is_ok():
		rospy.spinOnce()
		c.run()
		c.rate.sleep()
		

		
	
		
		
		
		
		
		
		 			

