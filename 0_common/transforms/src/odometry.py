#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math

class odometry:
    def __init__(self):
        
        rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
        
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.ticks_per_rev = 3072
        self.r = 0.04921
        self.B = 0.3
        self.rate = 20 #50ms, 20Hz
        self.K = 0.002

        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()

        while not rospy.is_shutdown():
            pass

    def encoder_callback(self, msg):
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_link2"
        self.t.header.stamp = msg.header.stamp

        E_r = msg.delta_encoder_right
        E_l = msg.delta_encoder_left
        T_r = msg.delta_time_right
        T_l = msg.delta_time_left

        D = (self.r/2)*(self.K*E_r + self.K*E_l)
        delta_theta = (self.r/self.B)*(self.K*E_r - self.K*E_l)

        self.x += D*math.cos(self.yaw)
        self.y += D*math.sin(self.yaw)
        self.yaw = self.yaw + delta_theta

        self.t.transform.translation.x = self.x
        self.t.transform.translation.y = self.y
        self.t.transform.translation.z = 0.0

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)

        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        print(self.t)

        self.br.sendTransform(self.t)


def main():
    rospy.init_node("odometry")
    odom = odometry()

if __name__ == '__main__':
    main()
