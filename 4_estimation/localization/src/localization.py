#!/usr/bin/env python3

import rospy

import numpy as np
import math

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Pose #, Transform, Vector3, Quaternion, Twist #, Pose
from sensor_msgs.msg import Imu
from aruco_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage
from robp_msgs.msg import Encoders

import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs

import copy

from visualization_msgs.msg import MarkerArray as visMA
from visualization_msgs.msg import Marker as visM

from tf import transformations


class localization:
    def __init__(self):
        #subscriber
        rospy.Subscriber('/imu/data', Imu , self.imu_callback)
        rospy.Subscriber('/motor/encoders',Encoders, self.encoder_callback)
        rospy.Subscriber('/aruco_single/pose',PoseStamped, self.aruco_single_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/marker', visMA, self.vismarker_callback)
        #rospy.Subscriber('/camera/aruco/markers', MarkerArray, self.aruco_callback)

        #publsiher
        self.cov_pub = rospy.Publisher('/state/cov', PoseWithCovarianceStamped, queue_size=10)
        
        #variable-imu
        self.new_imu = False
        self.angular_velocity = None

        #variable-encoder
        self.new_encoder = False
        self.motion = [0.0, 0.0]
        self.transform_time_Stamp = None
        self.odometry_ori = np.zeros(3)
        self.odometry_pos = np.zeros(3)

        #variable-aruco
        self.counter = 0
        self.new_measurement = False
        self.marker_id = None 
        self.marker_pos = None
        self.marker_ori = None

        #variable-VisMA
        self.counter2 = 0
        self.map_marker = False
        self.start_predict = False
        self.map_marker_id = None
        self.map_marker_pos = None
        self.map_marker_ori = None

        #variable-EKF
        self.F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.B = np.zeros((2,3))
        self.jf = np.zeros((3,3))
        self.dt = 0.06
        self.R = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, np.deg2rad(0.01)]])

        self.mean_pred = np.zeros(3)
        self.covariance_pred = np.zeros((3,3))
        self.mean_est = np.zeros(3)
        self.covariance_est = np.zeros((3,3))
        self.K = None
        self.jg = None
        self.Q = 0.05

        #broadcast-transform
        self.br = tf2_ros.TransformBroadcaster()

        #lookup-transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None

        self.rate = rospy.Rate(10)

        while not self.start_predict and not rospy.is_shutdown():
            self.init_broadcast()

        while not rospy.is_shutdown():
            if self.new_encoder:
                self.prediction()
                self.rate.sleep()
            

    def prediction(self):
        self.new_encoder = False
        self.B = np.array([[np.cos(self.odometry_ori[2])*self.dt, 0], [np.sin(self.odometry_ori[2])*self.dt, 0], [0, self.dt]])
        self.jf = np.array([[1,0, -self.motion[0]*np.sin(self.odometry_ori[2])*self.dt], [0, 1, self.motion[0]*np.cos(self.odometry_ori[2])*self.dt], [0, 0, 1]])
        self.mean_pred = (self.F).dot(self.mean_est) + (self.B).dot(self.motion)
        if np.any([abs(round(self.motion[0], 2)), abs(round(self.motion[1], 2))]):
            self.covariance_pred = ((self.jf).dot(self.covariance_est).dot((self.jf).T)) + self.R
        
        if self.new_measurement and self.map_marker:
            self.update()

        else:
            self.mean_est = copy.deepcopy(self.mean_pred)
            self.covariance_est = copy.deepcopy(self.covariance_pred)
            self.broadcast(self.mean_pred, self.covariance_pred)

#        print("prediction")
#        print("mean", self.mean_pred)
#        print("cov", self.covariance_pred)
#        print("------------------------------------------")

    def update(self):
        self.jg = np.array([[1,0,0],[0,1,0]])
        z = np.array([self.map_marker_pos.x, self.map_marker_pos.y]) 
        z_pred = np.array([self.marker_pos[0], self.marker_pos[1]]) 
        y = z - z_pred 
        S = (self.jg).dot(self.covariance_pred).dot((self.jg).T) + self.Q
        self.K = (self.covariance_pred).dot((self.jg).T).dot(np.linalg.pinv(S))
        self.mean_est = self.mean_pred + (self.K).dot(y)
        self.covariance_est = (np.eye(3) - (self.K).dot(self.jg)).dot(self.covariance_pred)
        self.new_measurement = False
        self.map_marker = False
        temp = self.covariance_est
        self.broadcast(self.mean_est, temp)

#        print("update")
#        print("mean", self.mean_est)
#        print("cov", self.covariance_est)
#        print("------------------------------------------")

    def init_broadcast(self):
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now()

        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        q = quaternion_from_euler(0,0,0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        self.rate.sleep()

    def broadcast(self, mean, covariance):
        #transform
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now() #self.transform_time_Stamp

        t.transform.translation.x = mean[0]
        t.transform.translation.y = mean[1]
        t.transform.translation.z = 0

        q = quaternion_from_euler(0,0,mean[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

        cov = PoseWithCovarianceStamped()

        cov.header.stamp = rospy.Time.now() #self.transform_time_Stamp
        cov.header.frame_id = "base_link"

        cov.pose.pose.position.x = mean[0]
        cov.pose.pose.position.y = mean[1]
        cov.pose.pose.position.z = 0

        cov.pose.pose.orientation.x = q[0]
        cov.pose.pose.orientation.y = q[1]
        cov.pose.pose.orientation.z = q[2]
        cov.pose.pose.orientation.w = q[3]
        
        cov_list = [0] * 35
        cov_list[0:8] = list(covariance.reshape((1,-1))[0])
        cov.pose.covariance = cov_list

        self.cov_pub.publish(cov)

    def imu_callback(self, msg):
        self.new_imu = True
        self.angular_velocity = -msg.angular_velocity.z

    def encoder_callback(self, msg):
        self.new_encoder = True
        r = 0.04921 #radious of the wheel
        b = 0.3 #distance between the wheels
        f = 20 #desired frequency in Hz
        ticks = 3072 #number of ticks per rev of the encoder
        K = 0.002
        E_r = msg.delta_encoder_right
        E_l = msg.delta_encoder_left

        D = (r/2)*(K*E_r + K*E_l)
        delta_theta = (r/b)*(K*E_r - K*E_l)
        self.odometry_ori = [0, 0, self.odometry_ori[2] + delta_theta]
        self.odometry_pos = [self.odometry_pos[0] + D*math.cos(self.odometry_ori[2]), self.odometry_pos[1] + D*math.sin(self.odometry_ori[2]), 0]

        if self.transform_time_Stamp != None:
            self.dt = (msg.header.stamp-self.transform_time_Stamp).to_sec()
        self.transform_time_Stamp = msg.header.stamp
        w1 = (2*np.pi*r*f*msg.delta_encoder_left)/ticks
        w2 = (2*np.pi*r*f*msg.delta_encoder_right)/ticks

        if self.new_imu:
            self.motion = [(w1+w2)/2, self.angular_velocity]
            self.new_imu = False
        else:
            self.motion = [(w1+w2)/2, (w2-w1)/(2*b)]

    def aruco_single_callback(self, msg):
        self.new_measurement = True

    def tf_callback(self, msg):
        if msg.transforms[0].header.frame_id == "map":
            try:
                transform = transformations.concatenate_matrices(transformations.translation_matrix([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z]), transformations.quaternion_matrix([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]))
                self.inversed_transform = transformations.inverse_matrix(transform)
                self.msg_header = msg.transforms[0].header.stamp
                self.marker_pos = transformations.translation_from_matrix(self.inversed_transform)
                self.marker_ori = transformations.quaternion_from_matrix(self.inversed_transform)
                self.marker_id = 500
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("No transform for tf")

    def vismarker_callback(self, msg): 
        markers = msg.markers
        for marker in markers:
            if marker.id == 500:
                try:    
                    trans = self.tfBuffer.lookup_transform("base_link", marker.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                    do_trans = tf2_geometry_msgs.do_transform_pose(marker, trans)
                    self.map_marker_pos = do_trans.pose.position
                    self.map_marker_ori = do_trans.pose.orientation
                    self.map_marker = True  
                    self.start_predict = True
                    self.map_marker_id = marker.id
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.loginfo("No transfrom in vismarker_callback")
                    self.rate.sleep()

    def aruco_callback(self, msg):
        pass


def main():
    rospy.init_node("localization")
    loc = localization()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("BYEEEEEEEEE")

if __name__ == '__main__':
    main()