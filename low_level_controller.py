#! /usr/bin/env python3


import rospy 
from geometry_msgs.msg import Twist
import RPi.GPIO as gpio 
import time
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

gpio.setup(16,gpio.OUT)
gpio.setup(12,gpio.OUT)
gpio.setup(20,gpio.OUT)
#gpio.output(25,gpio.HIGH)
gpio.setup(21,gpio.OUT)
#gpio.output(21,gpio.HIGH)


gpio.setwarnings(False)

p1 = gpio.PWM(20,1000)
p2 = gpio.PWM(21,1000)
p1.start(0)
p2.start(0)

def callback(msg):

        x_sped = msg.linear.x
        print(x_sped)
        pwm_val = x_sped * 60/0.7
        inc = msg.angular.z * 30/0.4 
        print("inc ",inc)
#       p_1 = False
#       p_2 = False
        if pwm_val > 0 : 
                gpio.output(16,gpio.HIGH)
                gpio.output(12,gpio.HIGH)
#               p_1 = False
#               p_2 = False
        else: 
                gpio.output(16,gpio.LOW)
                gpio.output(12,gpio.LOW)
#               p_1 = True
#               p_2 = True
        rpwm = abs(pwm_val)
        lpwm = abs(pwm_val)
#       pwm_val = pwm_val if pwm_val>0 else -1*pwm_val

        rpwm = rpwm - inc
        lpwm = lpwm + incrpwm =  rpwm if rpwm >=0 else 0
        lpwm = lpwm if lpwm >= 0 else 0
        print('rpwm ',rpwm)
        print('lpwm ',lpwm)


        p1.ChangeDutyCycle(rpwm)
        p2.ChangeDutyCycle(lpwm)




rospy.init_node("topic_subscriber")
sub = rospy.Subscriber('cmd_vel',Twist,callback)
rospy.spin()

        






