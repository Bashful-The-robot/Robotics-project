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
        rospy.Subscriber('/camera/aruco/markers', MarkerArray, self.aruco_callback)

        #publsiher
        self.cov_pub = rospy.Publisher('/state/cov', PoseWithCovarianceStamped, queue_size=10)
        
        #number of landmark
        self.landmark = 2

        #variable-imu
        self.new_imu = False
        self.angular_velocity = None

        #variable-encoder
        self.new_encoder = False
        self.motion = [0.0, 0.0]
        self.transform_time_Stamp = None
        self.odometry_ori = np.zeros(3)
        self.odometry_pos = np.zeros(3)

        #variable-aruco_single
        self.anchor = False
        self.marker_id = None 
        self.marker_pos = None
        self.marker_ori = None

        #variable-VisMA
        self.new_landmark = False
        self.start_predict = False
        self.landmark_det = np.zeros((self.landmark)) #500, 1, 2, 3 #1:deteced
        self.landmark_loc = np.zeros((2,self.landmark)) # 500, 1, 2, 3
        #self.map_marker_id = None
        #self.map_marker_pos = None
        #self.map_marker_ori = None !!!! Do you really need it???

        #variable-EKF
        self.F = np.concatenate((np.eye(3), np.zeros((3, 2*self.landmark))),axis=1)
        self.B = np.zeros((2,3+2*self.landmark))
        self.G = np.zeros((3+2*self.landmark,3+2*self.landmark))
        self.dt = 0.06
        self.R = np.eye(3)*0.01

        self.mean_pred = np.zeros(3+2*self.landmark)
        self.covariance_pred = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_pred, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))
        self.mean_est = np.zeros(3+2*self.landmark)
        self.covariance_est = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_est, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))

        self.K = None
        self.J = np.eye(2,5)
        self.J[0,3] = 1
        self.J[1,4] = 1
        self.Q = np.eye(2,2)*0.01

        self.update_done = True
        self.detected_before = np.zeros(self.landmark)

        #broadcast-transform
        self.br = tf2_ros.TransformBroadcaster()

        #lookup-transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None

        self.rate = rospy.Rate(5)

        while not self.anchor and not rospy.is_shutdown():
            rospy.loginfo("Waiting for anchor aruco")
            self.init_broadcast()

        while not rospy.is_shutdown():
            if self.new_encoder and self.update_done:
                self.prediction()
            

    def prediction(self):
        self.B[:,0:3] = np.array([[np.cos(self.odometry_ori[2])*self.dt, 0], [np.sin(self.odometry_ori[2])*self.dt, 0], [0, self.dt]]).T
        self.mean_pred = self.mean_est + ((self.B.T).dot(self.motion))
        self.G = np.eye(3+2*self.landmark) + (self.F.T).dot(np.array([[0,0, -self.motion[0]*np.sin(self.odometry_ori[2])*self.dt], [0, 0, self.motion[0]*np.cos(self.odometry_ori[2])*self.dt], [0, 0, 0]])).dot(self.F)
        if np.any([abs(round(self.motion[0], 2)), abs(round(self.motion[1], 2))]):
            self.covariance_pred = ((self.G).dot(self.covariance_est).dot((self.G).T)) + (self.F.T).dot(self.R).dot(self.F) #!!!!!! .T of the jacobian only????

#        print("prediction")
#        print("mean", self.mean_pred)
#        print("cov")
#        for cov in self.covariance_pred:
#            print(cov)
#        print("------------------------------------------")
        self.new_encoder = False
        if self.new_landmark: #np.any(self.landmark_det) == 1:
            self.update()

        else:
            self.mean_est = copy.deepcopy(self.mean_pred)
            self.covariance_est = copy.deepcopy(self.covariance_pred)
            self.broadcast(self.mean_pred, self.covariance_pred)

    def update(self):
        self.update_done = False
        for idx, landmark in enumerate(self.landmark_det):
            if landmark == 1:
                if self.detected_before[idx] == 0:
                    self.mean_pred[3+idx*2:4+idx*2+1] = np.array([self.landmark_loc[0][idx], self.landmark_loc[1][idx]])
                    self.detected_before[idx] = 1
                temp = np.zeros((5,3+2*self.landmark))
                np.fill_diagonal(temp, np.concatenate((np.ones(3), np.zeros(2*self.landmark))))
                temp[3,3+idx*2] = 1
                temp[4,4+idx*2] = 1
                H = np.array(self.J.dot(temp))
                z = np.array([self.mean_pred[3+idx*2], self.mean_pred[4+idx*2]]) 
                z_pred = np.array([self.landmark_loc[0][idx], self.landmark_loc[1][idx]]) 
                y = z_pred - z 

                S = (H).dot(self.covariance_pred).dot((H.T)) + self.Q
                self.K = (self.covariance_pred).dot(H.T).dot(np.linalg.pinv(S))

                self.mean_pred = self.mean_pred + (self.K).dot(y)
                self.covariance_pred = (np.eye(3+2*self.landmark) - (self.K).dot(H)).dot(self.covariance_pred)

        self.mean_est = copy.deepcopy(self.mean_pred)
        self.covariance_est = copy.deepcopy(self.covariance_pred)
        
        self.broadcast(self.mean_est, self.covariance_est)

        self.anchor = False
        self.new_landmark = False
        self.landmark_det = np.zeros((self.landmark))
        self.landmark_loc = np.zeros((2,self.landmark))
        self.update_done = True
        print("update")
        print("self.landmark_loc upadate", self.landmark_loc)
        print("mean", self.mean_est)
#        print("cov")
#        for cov in self.covariance_est:
#            print(cov)
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
        cov.header.frame_id = "bask_link"

        cov.pose.pose.position.x = mean[0]
        cov.pose.pose.position.y = mean[1]
        cov.pose.pose.position.z = 0

        cov.pose.pose.orientation.x = q[0]
        cov.pose.pose.orientation.y = q[1]
        cov.pose.pose.orientation.z = q[2]
        cov.pose.pose.orientation.w = q[3]
        
        cov_list = [0] * 36
        cov_list[0:9] = list(covariance[0:3,0:3].reshape((1,-1))[0])
        for i in range(min(self.landmark, 6)):
            if self.landmark >= 7:
                rospy.loginfo("cannot show all landmark's covariance ")
            cov_list[9+i*4:9+(i+1)*4] = list(covariance[3+i*2:5+i*2,3+i*2:5+i*2].reshape((1,-1))[0])
        cov.pose.covariance = cov_list
        self.cov_pub.publish(cov)

    def imu_callback(self, msg):
#        self.new_imu = True
        #p = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.z])
        #self.odometry_ori[2] = -p[1]
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
        self.odometry_ori[2] = self.odometry_ori[2] + delta_theta
        self.odometry_pos[0] += D*math.cos(self.odometry_ori[2])
        self.odometry_pos[1] += D*math.sin(self.odometry_ori[2])

        if self.transform_time_Stamp != None:
            self.dt = (msg.header.stamp-self.transform_time_Stamp).to_sec()
        self.transform_time_Stamp = msg.header.stamp
        w1 = (2*np.pi*r*f*msg.delta_encoder_left)/ticks
        w2 = (2*np.pi*r*f*msg.delta_encoder_right)/ticks

#        if self.new_imu:
        self.motion = [(w1+w2)/2, self.angular_velocity]
#            self.new_imu = False
#        else:
#            self.motion = [(w1+w2)/2, (w2-w1)/(2*b)]

            #not using the else

    def aruco_single_callback(self, msg):
        self.anchor = True

    def aruco_callback(self,msg):
        self.new_landmark = True
        
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
            if self.new_landmark and marker.id != 500:
                try:
                    trans = self.tfBuffer.lookup_transform("base_link", marker.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                    do_trans = tf2_geometry_msgs.do_transform_pose(marker, trans)
                    self.landmark_loc[:,marker.id] = np.array([do_trans.pose.position.x, do_trans.pose.position.y]) #!!!! Do you care about the z-axis???
                    self.landmark_det[marker.id] = 1
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.loginfo("No transfrom in vismarker_callback")
            if self.anchor and marker.id == 500:
                try:
                    trans = self.tfBuffer.lookup_transform("base_link", marker.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
                    do_trans = tf2_geometry_msgs.do_transform_pose(marker, trans)
                    self.landmark_loc[:,0] = np.array([do_trans.pose.position.x, do_trans.pose.position.y]) #!!!! Do you care about the z-axis???
                    self.landmark_det[0] = 1
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.loginfo("No transfrom in vismarker_callback")
            print("self.landmark_loc", self.landmark_loc)

def main():
    rospy.init_node("localization")
    loc = localization()

if __name__ == '__main__':
    main()