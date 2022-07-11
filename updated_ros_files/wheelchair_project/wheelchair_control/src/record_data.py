#! /usr/bin/env python3
import numpy as np
import rospy 
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time 


class record:


	def __init__(self):
		self.cmds = []
		self.speeds= []
	def cmd_cb(self,msg):
		self.cmds.append(msg.data)
		np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/cmds.np",self.cmds)
	def speed_cb(self,msg):
		data = [time.time(),self.cmds[-1],msg.twist.twist.linear.y,msg.twist.twist.angular.z]
		self.speeds.append(data)
		np.save("/home/abdallah/catkin_ws/src/wheelchair_project/wheelchair_control/data/speeds.np",self.speeds)
		
if __name__=="__main__":
	rospy.init_node("recorder_node")
	r = record()
	rospy.Subscriber("/odom",Odometry,r.speed_cb)
	rospy.Subscriber("control/open_loop_cmd",Int16,r.cmd_cb)
	rospy.spin()
