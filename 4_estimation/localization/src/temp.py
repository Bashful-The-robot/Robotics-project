#!/usr/bin/env python3

import rospy

import numpy as np
import math

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, Pose, PoseWithCovariance, PointStamped #, Transform, Vector3, Quaternion, Twist #, Pose
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

import time

class localization:
    def __init__(self):
        #subscriber
        rospy.Subscriber('/imu/data', Imu , self.imu_callback)
        rospy.Subscriber('/motor/encoders',Encoders, self.encoder_callback)
        rospy.Subscriber('/aruco_single/pose',PoseStamped, self.aruco_single_callback)

        rospy.Subscriber('/camera/aruco/markers', MarkerArray, self.aruco_callback)

        #publsiher
        self.cov_pub = rospy.Publisher('/state/cov', PoseWithCovarianceStamped, queue_size=10)
        self.odom_map_diff = rospy.Publisher('/odom_map_diff', PointStamped, queue_size=10)

        #number of landmark
        self.landmark = 4

        #variable-imu
        self.new_imu = False
        self.angular_velocity = None

        #variable-encoder
        self.new_encoder = False
        self.motion = [0.0, 0.0]
        self.transform_time_Stamp = None
        self.odometry_ori = np.zeros(3)
        self.odometry_pos = np.zeros(3)

        #variable-tf
        self.odom_loc = np.zeros(3)
        self.odom_ori = np.zeros(3)

        #variable-aruco_single
        self.anchor = False
        self.landmark_aruco = np.zeros((2,self.landmark)) # 500, 1, 2, 3
        self.buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.buffer)

        #variable-VisMA
        self.new_landmark = False
        self.start_predict = False
        self.landmark_det = np.zeros((self.landmark)) #500, 1, 2, 3 #1:deteced
        self.landmark_loc = np.zeros((2,self.landmark)) # 500, 1, 2, 3
        #self.map_marker_id = None
        #self.map_marker_pos = None
        #self.map_marker_ori = None !!!! Do you really need it???

        #variable-EKF
        self.F = np.eye(3)
        self.B = np.zeros((3,2))
        self.G = np.zeros((3,3))
        self.dt = 0.06
        self.R = np.eye(3)*1e-2

        self.mean_pred = np.zeros(3+2*self.landmark)
        self.covariance_pred = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_pred, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))
        self.mean_est = np.zeros(3+2*self.landmark)
        self.covariance_est = np.zeros((3+2*self.landmark,3+2*self.landmark))
        np.fill_diagonal(self.covariance_est, np.concatenate((np.zeros(3), np.ones(2*self.landmark)*10000)))

        self.K = None
        self.Q = np.eye(2)*1e-9

        self.update_done = True
        self.detected_before = np.zeros(self.landmark)

        self.diff = np.zeros(3)

        #broadcast-transform
        self.br = tf2_ros.TransformBroadcaster()

        #lookup-transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None

        self.rate = rospy.Rate(5)

        self.do_trans = None

        #while not self.anchor and not rospy.is_shutdown():
        #    rospy.loginfo("Waiting for anchor aruco")
        #    self.init_broadcast()
        time.sleep(5)
        while not rospy.is_shutdown():
            if self.new_encoder and self.update_done:
                self.prediction()
            

    def prediction(self):
#        self.B = np.array(
#            [
#            [np.cos(self.mean_pred[2])*self.dt, 0], 
#            [np.sin(self.mean_pred[2])*self.dt, 0], 
#            [0, self.dt]
#            ])
        
        self.B = np.array(
            [
            [(-self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2] + self.motion[1]*self.dt)],
            [(self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]) - (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]+self.motion[1]*self.dt)],
            [self.motion[1]*self.dt]
            ]
        )
        self.mean_pred[0:3] = (self.F @ self.mean_est[0:3]) + (self.F @ self.B).T
        #self.mean_pred[0:3] = (self.F @ self.mean_est[0:3]) + (self.B @ self.motion)
        
        self.mean_pred[2] = (self.mean_pred[2] + math.pi) % (2 * math.pi) - math.pi
        #self.mean_pred[2] = (self.mean_pred[2] + math.pi) % (2 * math.pi) - math.pi

        jF = np.array(
            [
            [0,0, (-self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.cos(self.mean_pred[2] + self.motion[1]*self.dt)], 
            [0, 0, (-self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2]) + (self.motion[0]/self.motion[1])*np.sin(self.mean_pred[2] + self.motion[1]*self.dt)], 
            [0, 0, 0]
            ])

#        jF = np.array(
#            [
#            [0,0, -self.motion[0]*np.sin(self.mean_pred[2])*self.dt], 
#            [0, 0, self.motion[0]*np.cos(self.mean_pred[2])*self.dt], 
#            [0, 0, 0]
#            ])
        
        #temp = np.eye(3+2*self.landmark) + (self.F.T).dot(np.array([[0,0, -self.motion[0]*np.sin(self.odometry_ori[2])*self.dt], [0, 0, self.motion[0]*np.cos(self.odometry_ori[2])*self.dt], [0, 0, 0]])).dot(self.F)
        #print(temp, self.G)

        Fx = np.eye(3) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #Fx = np.concatenate((np.eye(3), np.zeros((3, 2*self.landmark))), axis=1)
        self.G = np.eye(3) + Fx.T @ jF @ Fx

        if np.any([abs(round(self.motion[0], 2)), abs(round(self.motion[1], 2))]):
            self.covariance_pred[0:3,0:3] = (self.G @ self.covariance_est[0:3,0:3] @ (self.G.T)) + (Fx.T @ self.R @ Fx) #!!!!!! .T of the jacobian only????
#            self.covariance_pred[0:3,0:3] = (self.G.T @ self.covariance_est[0:3,0:3] @ (self.G)) + (Fx.T @ self.R @ Fx) #!!!!!! .T of the jacobian only????

#        print("prediction")
#        print("mean", self.mean_pred)
#        print("------------------------------------------")
#        print("cov")
#        for cov in self.covariance_pred:
#            print(cov)
        self.new_encoder = False
        if (self.new_landmark or self.anchor): # and np.sum(self.landmark_loc.reshape(1,-1)) != 0: #np.any(self.landmark_det) == 1:
            self.update()
        else:
            self.mean_est = copy.deepcopy(self.mean_pred)
            self.covariance_est = copy.deepcopy(self.covariance_pred)
            self.pred_broadcast(self.mean_est, self.covariance_est)
            self.update_broadcast(np.zeros(3))

    def update(self):
        self.update_done = False

        for idx, landmark in enumerate(self.landmark_det):
            new_trans = None
            print("self.landmark_loc",self.landmark_loc[:,0])
            print("self.landmark_aruco",self.landmark_aruco[:,0])
            if landmark == 1 :
                if self.detected_before[idx] == 0:
                    self.mean_pred[3+idx*2:4+idx*2+1] = np.array([self.landmark_loc[0][idx], self.landmark_loc[1][idx]]) #aruco coordinate in terms of map
                    self.detected_before[idx] = 1
                try:
                    trans = self.buffer.lookup_transform("map","odom", self.transform_time_Stamp, rospy.Duration(3.0))
                    try:
                        temp = PoseStamped()
                        temp.header.stamp = self.transform_time_Stamp
                        temp.header.frame_id = "odom"
                        temp.pose.position.x = self.mean_pred[0]
                        temp.pose.position.y = self.mean_pred[1]
                        temp.pose.position.z = 0
                        q = quaternion_from_euler(0,0,self.mean_pred[2])
                        temp.pose.orientation.x = q[0]
                        temp.pose.orientation.y = q[1]
                        temp.pose.orientation.z = q[2]
                        temp.pose.orientation.w = q[3]
                        temp_trans = tf2_geometry_msgs.do_transform_pose(temp, trans)
                        q2 = euler_from_quaternion([temp_trans.pose.orientation.x, temp_trans.pose.orientation.y, temp_trans.pose.orientation.z, temp_trans.pose.orientation.w])

                        new_trans = np.array([temp_trans.pose.position.x, temp_trans.pose.position.y, q2[2]])
                    except:
                        rospy.loginfo("No transform 2")

                except:
                    rospy.loginfo("No transform")
                
                if np.any(new_trans) != None:
                    #delta = np.array([self.mean_pred[3+idx*2] - self.mean_pred[0], self.mean_pred[4+idx*2] - self.mean_pred[1]]) 
                    delta = np.array([self.mean_pred[3+idx*2] - new_trans[0], self.mean_pred[4+idx*2] - new_trans[1]]) 
                    q = (delta @ delta.T)
                    if q >= 0.000000000001:
                        print("updating...")

                        #calculate z_est (expected measurement)
                        z_est_angle = math.atan2(delta[1], delta[0]) - new_trans[2]
                        z_est = np.array([math.sqrt(q), z_est_angle]) 

                        #calculate z (actual measurement)
                        est_lm_pose = np.array([self.landmark_aruco[0][idx] - new_trans[0] , self.landmark_aruco[1][idx] - new_trans[1]]) #!!!!!!!!!!!!!!!!!!!
                        z_range = math.sqrt((est_lm_pose @ est_lm_pose.T)) 
                        z_angle = math.atan2(est_lm_pose[1], est_lm_pose[0]) - new_trans[2] #!!!!!!!! self.mean_est to self.mean_pred
                        z = np.array([z_range, z_angle])

                        y = z - z_est 
                        #np.array([(z[0] - z_est[0]),(z[1] - z_est[1])])

    #                    F1 = np.eye(3,3+2*self.landmark)
    #                    F3 = np.eye(2,3+2*self.landmark, 3+idx*2)
    #                    F = np.concatenate((F1,F3))
                        F1 = np.eye(2,3+2*self.landmark)
                        F2 = np.zeros((1,3+2*self.landmark))
                        F3 = np.eye(2,3+2*self.landmark, 3+idx*2)
                        F = np.concatenate((F1,F2,F3))

                        H = np.array(
                            [
                            [-math.sqrt(q)*delta[0], -math.sqrt(q)*delta[1], 0, math.sqrt(q)*delta[0], math.sqrt(q)*delta[1]],
                            [delta[1], -delta[0], -q, -delta[1], delta[0]]
                            ])
                        
                        H = H/q

                        H = H @ F

                        S = (H @ self.covariance_pred @ H.T) + self.Q
                        self.K = (self.covariance_pred @ H.T) @ np.linalg.pinv(S)
                        self.mean_pred = self.mean_pred + (self.K @ y.T) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! added .T to y
                        self.covariance_pred = ((np.eye(3+2*self.landmark) - (self.K @ H)) @ self.covariance_pred)

        self.diff = np.array([self.mean_pred[0] - self.mean_est[0], self.mean_pred[1] - self.mean_est[1], self.mean_pred[2] - self.mean_est[2]])
        #print("self.diff",self.diff)
        self.mean_est = copy.deepcopy(self.mean_pred)
        self.covariance_est = copy.deepcopy(self.covariance_pred)
        #self.pred_broadcast(self.mean_est, self.covariance_est)
        
        self.update_broadcast(self.diff)

        self.anchor = False
        self.new_landmark = False
        self.landmark_det = np.zeros((self.landmark))
        self.landmark_loc = np.zeros((2,self.landmark))
        self.update_done = True
        print("update")
        print("mean", self.mean_est)
        print("------------------------------------------")
#        print("cov")
#        for cov in self.covariance_est:
#            print(cov)

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
        t.header.stamp = self.transform_time_Stamp

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
                t.header.frame_id = "map"
                t.child_frame_id = "landmark" + str(i)
                t.header.stamp = self.transform_time_Stamp

                t.transform.translation.x = mean[3+i*2]
                t.transform.translation.y = mean[4+i*2]
                t.transform.translation.z = 0

                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
                self.br.sendTransform(t)

        cov = PoseWithCovarianceStamped()

        cov.header.stamp = self.transform_time_Stamp
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
                rospy.loginfo("Cannot show all landmark's covariance ")
            cov_list[9+i*4:9+(i+1)*4] = list(covariance[3+i*2:5+i*2,3+i*2:5+i*2].reshape((1,-1))[0])
        cov.pose.covariance = cov_list
        self.cov_pub.publish(cov)
        

    def update_broadcast(self, diff):
        msg = PointStamped ()
        msg.header.stamp = self.transform_time_Stamp
        msg.point.x = diff[0]
        msg.point.y = diff[1]
        msg.point.z = diff[2]
        self.odom_map_diff.publish(msg)

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
        if self.update_done: #!!!!!!!added if
            self.anchor = True
            trans = None
            trans2 = None
            do_trans2 = None
            if self.detected_before[0] == 0:
                try:
                    trans = self.buffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
                    self.do_trans = tf2_geometry_msgs.do_transform_pose(msg, trans)
                    self.landmark_loc[:,0] = np.array([self.do_trans.pose.position.x, self.do_trans.pose.position.y]) #!!!! Do you care about the z-axis???
                    self.landmark_aruco[0,0] = self.do_trans.pose.position.x
                    self.landmark_aruco[1,0] = self.do_trans.pose.position.y
                    self.landmark_det[0] = 1
                except:
                    rospy.loginfo("No transform aruco single")
            else:
                trans = self.buffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
                self.do_trans = tf2_geometry_msgs.do_transform_pose(msg, trans)
                self.landmark_aruco[0,0] = self.do_trans.pose.position.x
                self.landmark_aruco[1,0] = self.do_trans.pose.position.y
                self.landmark_det[0] = 1
#                try:
#                    trans = self.buffer.lookup_transform("odom",msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
#                    self.do_trans = tf2_geometry_msgs.do_transform_pose(msg, trans)
#                    trans2 = self.buffer.lookup_transform("map","odom", msg.header.stamp, rospy.Duration(0.5))
#                    do_trans2 = tf2_geometry_msgs.do_transform_pose(self.do_trans, trans2)
#                    self.landmark_loc[:,0] = np.array([do_trans2.pose.position.x, do_trans2.pose.position.y]) #!!!! Do you care about the z-axis???
#                    self.landmark_aruco[0,0] = do_trans2.pose.position.x
#                    self.landmark_aruco[1,0] = do_trans2.pose.position.y
#                    self.landmark_det[0] = 1
#                except:
#                    rospy.loginfo("No transform aruco single")
#            else:
#                try:
#                    trans2 = self.buffer.lookup_transform("map","odom", msg.header.stamp, rospy.Duration(0.5))
#                except:
#                    rospy.loginfo("No transform aruco single 2")
#                try:
#                    do_trans2 = tf2_geometry_msgs.do_transform_pose(self.do_trans, trans2)
#
#                    self.landmark_aruco[0,0] = do_trans2.pose.position.x
#                    self.landmark_aruco[1,0] = do_trans2.pose.position.y
#                    self.landmark_det[0] = 1
#                except:
#                    rospy.loginfo("No transform aruco single 3")

    def aruco_callback(self,msg):
        if self.update_done: #!!!!added if
            for marker in msg.markers:
                if 1 <= marker.id <= 3:
                    self.new_landmark = True
                    try:
                        trans = self.buffer.lookup_transform("map",marker.header.frame_id, marker.header.stamp, rospy.Duration(0.5))
                        do_trans = tf2_geometry_msgs.do_transform_pose(marker.pose, trans)
                    except:
                        rospy.loginfo("No transform aruco")

                    if self.detected_before[marker.id] == 0:
                        self.landmark_loc[:,marker.id] = np.array([do_trans.pose.position.x, do_trans.pose.position.y]) #!!!! Do you care about the z-axis???
                    self.landmark_aruco[0,marker.id] = do_trans.pose.position.x
                    self.landmark_aruco[1,marker.id] = do_trans.pose.position.y
                    self.landmark_det[marker.id] = 1


def main():
    rospy.init_node("localization")
    loc = localization()

if __name__ == '__main__':
    main()