#!/usr/bin/env python3
'''
Camera closes to the rasberrypi is camera_color_optical_frame
Measurments:
    3.5 cm of center in + y
    z = 10.4
    x= 9.6
Robot center:
    y=13.25cm from wheel
    z=0
    x = 0
'''
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math

rospy.init_node('transform_map_odom')
br = tf2_ros.TransformBroadcaster()

t = TransformStamped()
t.header.frame_id = "base_link"
t.child_frame_id = "camera_link"


tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    t.header.stamp = rospy.Time.now()

    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    t.transform.translation.x = 0.096
    t.transform.translation.y = 0.035
    t.transform.translation.z = 0.104
    br.sendTransform(t)
    rate.sleep()

if __name__ == '__main__':
    rospy.spin()