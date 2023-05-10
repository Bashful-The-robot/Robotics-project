#!/usr/bin/env python3

import rospy
from robp_msgs.msg import DutyCycles
from geometry_msgs.msg import Twist
import numpy as np
import math
import getch
import keyboard
import sys
msg = "forward: w \n backward: s \n left: a \n right: d \n stop: p"

rospy.init_node('ms1_control')

control_info = DutyCycles()
linear_vel = 0
angular_vel = 0

control_pub = rospy.Publisher('/motor_controller/twist', Twist, queue_size = 10)
info = Twist()

while not rospy.is_shutdown():
    print(msg)
    choice = input() #getch.getche() #sys.stdin.readline()
    if choice == "w":
        linear_vel += 0.1
    elif choice == "s":
        linear_vel -= 0.1
    elif choice == "a":
        angular_vel += 0.05
    elif choice == "d":
        angular_vel -= 0.05
    elif choice == "p":
        linear_vel = 0.0
        angular_vel = 0.0
    angular_vel = max(-1, angular_vel)
    angular_vel = min(angular_vel, 1)

    linear_vel = max(-1, linear_vel)
    linear_vel = min(linear_vel, 1)
    print(linear_vel, angular_vel)
    info.linear.x = linear_vel
    info.angular.z = angular_vel
    control_pub.publish(info)
    
if __name__ == '__main__':
    rospy.spin()
