#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion #, Pose
from aruco_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage

import tf_conversions
import tf2_ros

class localization:
    def __init__(self):
        #subscriber
        rospy.Subscriber('/camera/aruco/markers', MakrerArray, self.aruco_callback)
        rospy.Subscriber('/tf', TFMessage, self.odometry_callback)

        #broadcast transform
        br = tf2.TransformBroadcaster()

        #transform
        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "/state_estimattion"
        t.header.stamp = rospy.Time.now()

        #variable
        self.marker_id = None
        self.marker_pos = None
        self.marker_ori = None
        self.odometry_pos = None
        self.odometry_ori = None
        self.transform_time_Stamp = None

        self.x = None
        self.predicted_x = 
        self.p = None #covariance
        self.sigma_points = np.zeros()
        self.P_last_state = 
        self.n = 
        self.alpha = 0.001
        self.kappa = 1
        self.beta = 2
        self._lambda = self.alpha**2(self.n+self.kappa) - self.n
        self.gamma = np.sqrt(self.n + self._lambda)

        self.weight
        
        while self.marker_id == None:
            print("Waiting for the starting Aruco marker")
            continue

        self.initialization()
        
    def initialization(self):
        self.x = [0,0,0] #x,y,theta
        self.p = 

    def prediction(self):
        self.predicted_x = np.sum(se)
        sqrt_P = np.linalg.cholesky(self.P_last_state)


    def aruco_callback(self, msg):
        self.marker_id = msg.markers[0].id
        self.marker_pos = msg.markers[0].pose.pose.position
        self.marker_ori = msg.markers[0].pose.pose.orientation

    def odometry_callback(self, msg):
        if msg.child_frame_id == "base_link"
            self.transform_time_Stamp = msg.header.stamp
            self.odometry_pos = msg.transform.translation
            self.odometry_ori = msg.transform.rotation


def main(args):
    rospy.init_node("localization hate me")
    loc = localization()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("BYEEEEEEEEE")

if __name__ == '__main__':
    main(sys.argv)
