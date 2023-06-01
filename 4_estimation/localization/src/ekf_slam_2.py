#!/usr/bin/env python3

import rospy

import numpy as np
import math

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Pose, PoseWithCovariance, PointStamped, TwistStamped #, Transform, Vector3, Quaternion, Twist #, Pose
from sensor_msgs.msg import Imu
from aruco_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage
from robp_msgs.msg import Encoders
from std_msgs.msg import Float32MultiArray
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs

import copy

from visualization_msgs.msg import MarkerArray as visMA
from visualization_msgs.msg import Marker as visM

from tf import transformations

import time

class localization:
    def __init__(self):
        #subscriber
        rospy.Subscriber('/imu/data', Imu , self.imu_callback, queue_size=1)
        #rospy.Subscriber('/motor/encoders',Encoders, self.encoder_callback)
        rospy.Subscriber('/aruco_single/pose',PoseStamped, self.aruco_single_callback, queue_size=1)
        rospy.Subscriber('/camera/aruco/markers', MarkerArray, self.aruco_callback, queue_size=1)
        rospy.Subscriber('/velocity', TwistStamped, self.velocity_callback, queue_size=1)
        rospy.Subscriber('/odometry', TransformStamped, self.odometry_callback, queue_size=1)
        #publsiher
        self.cov_pub = rospy.Publisher('/state/cov', PoseWithCovarianceStamped, queue_size=50)
        self.odom_map_diff = rospy.Publisher('/odom_map_diff', PointStamped, queue_size=10)
        self.anchor_pos = rospy.Publisher('/anchor_aruco', PoseStamped, queue_size=10)

        #number of landmark
        self.landmark = 4

        #variable-imu
        self.new_imu = False
        self.angular_velocity = None

#        #variable-encoder
#        self.new_encoder = False
#        self.motion = [0.0, 1e-12]
#        self.transform_time_stamp = rospy.Time.now()
#        self.odometry_ori = np.zeros(3)
#        self.odometry_pos = np.zeros(3)
#        self.dt = 0.06

        #variable-odometry
        self.new_odometry = False
        self.transform_time_stamp = rospy.Time.now()
        self.odometry_ori = np.zeros(3)
        self.odometry_pos = np.zeros(3)
        self.dt = 0.06
        
        #variable-velocity
        self.motion = [0.0, 1e-12]

        #variable-tf
        self.odom_loc = np.zeros(3)
        self.odom_ori = np.zeros(3)

        #variable-aruco
        self.max_aruco_dist = 1.5
        self.anchor = False
        self.landmark_aruco = np.zeros((2,self.landmark)) # 500, 1, 2, 3
        self.buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.new_landmark = False
        self.start_predict = False
        self.landmark_det = np.zeros((self.landmark)) #500, 1, 2, 3 #1:deteced
        self.landmark_loc = np.zeros((2,self.landmark)) # 500, 1, 2, 3
        self.update_timestamp_500 = rospy.Time.now()
        self.update_timestamp_box = rospy.Time.now()

        #variable-EKF
        self.F = np.eye(3)
        self.B = np.zeros((3,2))
        self.G = np.zeros((3,3))
        self.R = np.eye(3)*1e-3

        self.mean_pred = np.zeros(3+2*self.landmark)
        self.covariance_pred = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_pred, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))

        self.mean_est = np.zeros(3+2*self.landmark)
        self.covariance_est = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_est, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))

        self.K = None
        self.Q = np.eye(2)*1e-8

        self.update_done = True
        self.detected_before = np.zeros(self.landmark)

        self.diff = np.zeros(3)

        #broadcast-transform
        self.br = tf2_ros.TransformBroadcaster()

        #lookup-transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None

        self.rate = rospy.Rate(20)

        while not self.anchor and not rospy.is_shutdown():
            rospy.loginfo("Waiting for anchor aruco")
            self.init_broadcast()

        #time.sleep(5)
        while not rospy.is_shutdown():
            if self.new_odometry and self.update_done:
                self.prediction()
                self.rate.sleep()
            

    def prediction(self):
        jF = np.array(
            [
            [0, 0, 0],
            [0, 0, 0], 
            [0, 0, 0]
            ])
        if self.motion[1] != 0:
            self.B = np.array(
                [
                [(-self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2] + self.motion[1]*self.dt)],
                [(self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]) - (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2] + self.motion[1]*self.dt)],
                [self.motion[1]*self.dt]
                ]
            )
            self.mean_pred[0:3] = (self.F @ self.mean_pred[0:3]) + (self.F @ self.B).T
            
            self.mean_pred[2] = (self.mean_pred[2] + math.pi) % (2 * math.pi) - math.pi

            jF = np.array(
                [
                [0,0, (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]) - (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2] + self.motion[1]*self.dt)], 
                [0, 0, (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2]) - (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2] + self.motion[1]*self.dt)], 
                [0, 0, 0]
                ])
            
    #        jF = np.array(
    #            [
    #            [0,0, (-self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2] + self.motion[1]*self.dt)], 
    #            [0, 0, (-self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2] + self.motion[1]*self.dt)], 
    #            [0, 0, 0]
    #            ])
            
        Fx = np.eye(3)
        self.G = np.eye(3) + Fx.T @ jF @ Fx

        if np.any([abs(round(self.motion[0], 2)), abs(round(self.motion[1], 2))]):
            self.covariance_pred[0:3,0:3] = (self.G @ self.covariance_est[0:3,0:3] @ (self.G.T)) + (Fx.T @ self.R @ Fx) 

        print("prediction")
        print("mean", self.mean_pred)
        print("------------------------------------------")
        self.new_odometry = False
        if (self.new_landmark or self.anchor): 
            self.update()
        else:
            self.mean_est = copy.deepcopy(self.mean_pred)
            self.covariance_est = copy.deepcopy(self.covariance_pred)
            self.pred_broadcast(self.mean_est, self.covariance_est)
            self.update_broadcast(np.zeros(3), self.transform_time_stamp)

    def update(self):
        self.update_done = False
        print(self.landmark_det)
        for idx, landmark in enumerate(self.landmark_det):
#            print("self.landmark_loc",self.landmark_loc.reshape(1,-1))
#            print("self.landmark_aruco",self.landmark_aruco.reshape(1,-1))
            if landmark == 1 :
                if self.detected_before[idx] == 0:
                    self.mean_pred[3+idx*2:4+idx*2+1] = np.array([self.landmark_loc[0][idx], self.landmark_loc[1][idx]]) #aruco coordinate in terms of map
                    self.detected_before[idx] = 1
                
                delta = np.array([self.mean_pred[3+idx*2] - self.mean_pred[0], self.mean_pred[4+idx*2] - self.mean_pred[1]]) 
                q = (delta @ delta.T)
                if 1e-9 < q: # < 1:
                    #calculate z_est (expected measurement)
                    z_est_angle = math.atan2(delta[1], delta[0]) - self.mean_pred[2]
                    z_est_angle = (z_est_angle + math.pi) % (2 * math.pi) - math.pi
                    z_est = np.array([math.sqrt(q), z_est_angle]) 

                    #calculate z (actual measurement)
                    est_lm_pose = np.array([self.landmark_aruco[0][idx] - self.mean_pred[0] , self.landmark_aruco[1][idx] - self.mean_pred[1]]) #!!!!!!!!!!!!!!!!!!!
                    z_range = math.sqrt((est_lm_pose @ est_lm_pose.T)) 
                    z_angle = math.atan2(est_lm_pose[1], est_lm_pose[0]) - self.mean_pred[2] #!!!!!!!! self.mean_est to self.mean_pred
                    z_angle = (z_angle + math.pi) % (2 * math.pi) - math.pi
                    z = np.array([z_range, z_angle])

                    y = z-z_est

                    F1 = np.eye(3,3+2*self.landmark)
                    F2 = np.eye(2,3+2*self.landmark, 3+idx*2)
                    F = np.concatenate((F1,F2))

                    H = np.array(
                        [
                        [-math.sqrt(q)*delta[0], -math.sqrt(q)*delta[1], 0, math.sqrt(q)*delta[0], math.sqrt(q)*delta[1]],
                        [delta[1], -delta[0], -q, -delta[1], delta[0]]
                        ])

#                    H = np.array(
#                        [
#                        [math.sqrt(q)*delta[0], -math.sqrt(q)*delta[1], 0, -math.sqrt(q)*delta[0], math.sqrt(q)*delta[1]],
#                        [delta[1], delta[0], -1, -delta[1], -delta[0]]
#                        ])
                                        
                    H = H/q

                    H = H @ F

                    S = (H @ self.covariance_pred @ H.T) + self.Q
                    self.K = (self.covariance_pred @ H.T) @ np.linalg.pinv(S)
                    self.mean_pred = self.mean_pred + (self.K @ y.T)
                    self.covariance_pred = ((np.eye(3+2*self.landmark) - (self.K @ H)) @ self.covariance_pred)

        self.diff = self.mean_pred[0:3] - self.mean_est[0:3]
        self.mean_est = copy.deepcopy(self.mean_pred)
        self.covariance_est = copy.deepcopy(self.covariance_pred)
        
        if 1 in self.landmark_det[0:-2]:
            self.update_broadcast(self.diff, self.update_timestamp_box)
        elif self.landmark_det[-1] == 1:
            self.update_broadcast(self.diff, self.update_timestamp_500)

        self.anchor = False
        self.new_landmark = False
        self.landmark_det = np.zeros((self.landmark))
        self.landmark_loc = np.zeros((2,self.landmark))
        self.update_done = True
        print("update")
        print("mean", self.mean_est)
        print("------------------------------------------")

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

    def pred_broadcast(self, mean, covariance):
        #transform
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = self.transform_time_stamp #rospy.Time.now()

        t.transform.translation.x = mean[0]
        t.transform.translation.y = mean[1]
        t.transform.translation.z = 0

        q = quaternion_from_euler(0,0,mean[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)
        
        for i in range(self.landmark):
            if self.detected_before[i] == 1:
                t = TransformStamped()
                t.header.frame_id = "odom"
                t.child_frame_id = "landmark" + str(i)
                t.header.stamp = self.transform_time_stamp

                t.transform.translation.x = mean[3+i*2]
                t.transform.translation.y = mean[4+i*2]
                t.transform.translation.z = 0

                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
                self.br.sendTransform(t)

        cov = PoseWithCovarianceStamped()

        cov.header.stamp = self.transform_time_stamp
        cov.header.frame_id = "odom"

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
                rospy.loginfo("Cannot show all landmark's covariance ")
            cov_list[9+i*4:9+(i+1)*4] = list(covariance[3+i*2:5+i*2,3+i*2:5+i*2].reshape((1,-1))[0])
        cov.pose.covariance = cov_list
        self.cov_pub.publish(cov)
        

    def update_broadcast(self, diff, timestamp):
        msg = PointStamped ()
        msg.header.stamp = timestamp
        msg.point.x = diff[0]
        msg.point.y = diff[1]
        msg.point.z = diff[2]
        self.odom_map_diff.publish(msg)

    def imu_callback(self, msg):
        self.angular_velocity = -msg.angular_velocity.z

#    def encoder_callback(self, msg):
#        self.new_odometry = True
#        r = 0.04921 #radious of the wheel
#        b = 0.3 #distance between the wheels
#        f = 20 #desired frequency in Hz
#        ticks = 3072 #number of ticks per rev of the encoder
#        K = 0.002
#        E_r = msg.delta_encoder_right
#        E_l = msg.delta_encoder_left
#
#        D = (r/2)*(K*E_r + K*E_l)
#        delta_theta = (r/b)*(K*E_r - K*E_l)
#        self.odometry_ori[2] = self.odometry_ori[2] + delta_theta
#        self.odometry_pos[0] += D*math.cos(self.odometry_ori[2])
#        self.odometry_pos[1] += D*math.sin(self.odometry_ori[2])
#
#        if self.transform_time_stamp != None:
#            self.dt = (msg.header.stamp-self.transform_time_stamp).to_sec()
#        self.transform_time_stamp = msg.header.stamp
#        w1 = (2*np.pi*r*f*msg.delta_encoder_left)/ticks
#        w2 = (2*np.pi*r*f*msg.delta_encoder_right)/ticks
#
#        if self.angular_velocity != None:
#            self.motion = [(w1+w2)/2, self.angular_velocity+1e-9]
#        else:
#            self.motion = self.motion = [(w1+w2)/2, (w2-w1)/(2*b)]

    def odometry_callback(self, msg):
        self.new_odometry = True
        if self.transform_time_stamp != None:
            self.dt = (msg.header.stamp-self.transform_time_stamp).to_sec()
        self.transform_time_stamp = msg.header.stamp
        theta = euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        self.odometry_ori[2] = theta[2]
        self.odometry_pos[0] = msg.transform.translation.x
        self.odometry_pos[1] = msg.transform.translation.y

    def velocity_callback(self, msg):
        if self.angular_velocity != None:
            self.motion = [msg.twist.linear.x, self.angular_velocity]
        else:
            self.motion = [msg.twist.linear.x, msg.twist.angular.z]

    def aruco_single_callback(self, msg):
        if self.update_done:
            print("IN ARUCO ANCHORRRRRRRRRRRRRRRR")
            #if np.sqrt(msg.pose.position.z**2 + msg.pose.position.x**2) < self.max_aruco_dist:
            if abs((msg.header.stamp-self.transform_time_stamp).to_sec()) < self.dt:
                self.anchor = True
                try:
                    trans = self.buffer.lookup_transform("odom",msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
                    do_trans = tf2_geometry_msgs.do_transform_pose(msg, trans)
                    self.update_timestamp_500 = msg.header.stamp
                    #if do_trans.pose.position.x < self.max_aruco_dist and do_trans.pose.position.y < self.max_aruco_dist: 
                    if self.detected_before[0] == 0:
                        self.landmark_loc[:,0] = np.array([do_trans.pose.position.x, do_trans.pose.position.y])
                    self.landmark_aruco[0,0] = do_trans.pose.position.x
                    self.landmark_aruco[1,0] = do_trans.pose.position.y
                    self.landmark_det[0] = 1
                    anchor_pub = PoseStamped()
                    anchor_pub.header.stamp = msg.header.stamp
                    anchor_pub.header.frame_id = "odom"
                    anchor_pub.pose.position.x = do_trans.pose.position.x
                    anchor_pub.pose.position.y = do_trans.pose.position.y
                    anchor_pub.pose.position.z = 0
                    anchor_pub.pose.orientation.x = 0
                    anchor_pub.pose.orientation.y = 0
                    anchor_pub.pose.orientation.z = 0
                    anchor_pub.pose.orientation.w = 1
                    self.anchor_pos.publish(anchor_pub)
                except:
                    rospy.loginfo("No transform aruco single")

    def aruco_callback(self,msg):
        if self.update_done:
            temp_list_for_id = []
            for marker in msg.markers:
                if 1 <= marker.id <= 3:
                    if np.sqrt(marker.pose.pose.position.z**2 + marker.pose.pose.position.x**2) < self.max_aruco_dist:
                        if abs((msg.header.stamp-self.transform_time_stamp).to_sec()) < self.dt:
                            self.new_landmark = True
                            try:
                                trans = self.buffer.lookup_transform("odom",marker.header.frame_id, marker.header.stamp, rospy.Duration(0.5))
                                do_trans = tf2_geometry_msgs.do_transform_pose(marker.pose, trans)
                                self.update_timestamp_box = marker.header.stamp
                                #if do_trans.pose.position.x < self.max_aruco_dist and do_trans.pose.position.y < self.max_aruco_dist: 
                                if self.detected_before[marker.id] == 0:
                                    self.landmark_loc[:,marker.id] = np.array([do_trans.pose.position.x, do_trans.pose.position.y])
                                self.landmark_aruco[0,marker.id] = do_trans.pose.position.x
                                self.landmark_aruco[1,marker.id] = do_trans.pose.position.y
                                self.landmark_det[marker.id] = 1
                            except:
                                rospy.loginfo("No transform aruco")

def main():
    rospy.init_node("localization")
    loc = localization()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()