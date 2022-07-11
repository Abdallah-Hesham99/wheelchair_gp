#! /usr/bin/env python3
import numpy as np
import rospy 
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time 

import uuid
class record:


	def __init__(self):
		self.rcmds = [0]
		self.lcmds = [0]
		self.speeds= []
		
	def rcmd_cb(self,msg):
		self.rcmds.append(msg.data)
		np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/rcmds6",self.rcmds)
	def lcmd_cb(self,msg):
		self.lcmds.append(msg.data)
		np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/lcmds6",self.lcmds)
	
	def speed_cb(self,msg):
		data = [time.time(),self.rcmds[-1],self.lcmds[-1],msg.twist.twist.linear.y,msg.twist.twist.angular.z]
		self.speeds.append(data)
		#np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/speeds"+str(uuid.uuid1()),self.speeds)
		np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/speeds6",self.speeds)
if __name__=="__main__":
	rospy.init_node("recorder_node")
	r = record()
	rospy.Subscriber("/odom",Odometry,r.speed_cb)
	rospy.Subscriber("control/open_loop/right_wheel",Int16,r.rcmd_cb)
	rospy.Subscriber("control/open_loop/left_wheel",Int16,r.lcmd_cb)
	rospy.spin()
