#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped

import tf2_ros
import tf2_geometry_msgs
from tf import transformations

import numpy as np
from std_msgs.msg import Bool

from tf2_msgs.msg import TFMessage

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ArucoDetection:
    def __init__(self):
        #Subscriber
        rospy.Subscriber('/aruco_single/pose',PoseStamped,self.callback)
        rospy.Subscriber('/odom_map_diff', PointStamped, self.diff_callback)

        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(10)
        self.msg_header = None
        self.inversed_transform = None
        self.trans = None
        self.diff = np.zeros(3)
        self.done = False
        self.diff_new = False
        self.temp = None
        self.temp2 = None
        while not rospy.is_shutdown():
            if self.msg_header != None:
                self.initMap()
                self.rate.sleep()

    def diff_callback(self, msg):
        self.diff_new = True
        self.msg_header = msg.header.stamp
        self.diff = [msg.point.x, msg.point.y, msg.point.z]

    def callback(self, msg):
        if not self.diff_new:
            self.msg_header = msg.header.stamp
        if not self.done:
            try:
                trans_temp = self.buffer.lookup_transform("base_link","camera_color_optical_frame", msg.header.stamp, rospy.Duration(0.5))
                try:
                    self.trans = tf2_geometry_msgs.do_transform_pose(msg, trans_temp)
                    transform = transformations.concatenate_matrices(transformations.translation_matrix([self.trans.pose.position.x, self.trans.pose.position.y, self.trans.pose.position.z]), transformations.quaternion_matrix([self.trans.pose.orientation.x, self.trans.pose.orientation.y, self.trans.pose.orientation.z, self.trans.pose.orientation.w]))
                    self.inversed_transform = transformations.inverse_matrix(transform)
                    #rospy.loginfo("done transform")
                    self.done = True
                except:
                    rospy.loginfo("no do_transform from baselink to camera")
            except:
                rospy.loginfo("No transform from baselink to camera")

    def initMap(self):
        self.bc = tf2_ros.TransformBroadcaster()
        self.init_map_odom = TransformStamped()
        self.init_map_odom.header.frame_id = "map"
        self.init_map_odom.child_frame_id = "odom"
        self.init_map_odom.header.stamp = rospy.Time.now() #self.msg_header

        if np.any(self.trans) != None and np.any(self.inversed_transform) != None:  
#            if np.any(self.temp) == None:
#                self.temp = transformations.translation_from_matrix(self.inversed_transform)
#            if np.any(self.temp2) == None:
#                self.temp2 = transformations.quaternion_from_matrix(self.inversed_transform)
#                
#            self.init_map_odom.transform.translation.x = self.temp[0]
#            self.init_map_odom.transform.translation.y = self.temp[1]
#            self.init_map_odom.transform.translation.z = self.temp[2]
#
#            self.init_map_odom.transform.rotation.x = self.temp2[0]
#            self.init_map_odom.transform.rotation.y = self.temp2[1] 
#            self.init_map_odom.transform.rotation.z = self.temp2[2]
#            self.init_map_odom.transform.rotation.w = self.temp2[3]

            diff = [self.diff[0], self.diff[1], self.diff[2]]     

            if np.any(self.temp) == None:
                self.temp = transformations.translation_from_matrix(self.inversed_transform)
            if np.any(self.temp2) == None:
                self.temp2 = transformations.quaternion_from_matrix(self.inversed_transform)
                
            new_pose = PoseStamped()
            new_pose.header.stamp = self.msg_header
            new_pose.header.frame_id = "map"
            new_pose.pose.position.x = self.temp[0]
            new_pose.pose.position.y = self.temp[1]
            new_pose.pose.position.z = self.temp[2]

            new_pose.pose.orientation.x = self.temp2[0]
            new_pose.pose.orientation.y = self.temp2[1]
            new_pose.pose.orientation.z = self.temp2[2]
            new_pose.pose.orientation.w = self.temp2[3]

            trans = TransformStamped()
            trans.header.stamp = self.msg_header
            trans.header.frame_id = "map"
            trans.child_frame_id = "odom"
            trans.transform.translation.x = diff[0]
            trans.transform.translation.y = diff[1]
            trans.transform.translation.z = 0
            q = quaternion_from_euler(0, 0, diff[2])

            trans.transform.rotation.x = q[0]
            trans.transform.rotation.y = q[1]
            trans.transform.rotation.z = q[2]
            trans.transform.rotation.w = q[3]

            do_trans = tf2_geometry_msgs.do_transform_pose(new_pose,trans)

            self.init_map_odom.transform.translation.x = do_trans.pose.position.x
            self.init_map_odom.transform.translation.y = do_trans.pose.position.y
            self.init_map_odom.transform.translation.z = do_trans.pose.position.z

            self.init_map_odom.transform.rotation.x = do_trans.pose.orientation.x
            self.init_map_odom.transform.rotation.y = do_trans.pose.orientation.y 
            self.init_map_odom.transform.rotation.z = do_trans.pose.orientation.z
            self.init_map_odom.transform.rotation.w = do_trans.pose.orientation.w

        else:
            self.init_map_odom.transform.translation.x = 0
            self.init_map_odom.transform.translation.y = 0
            self.init_map_odom.transform.translation.z = 0

            self.init_map_odom.transform.rotation.x = 0
            self.init_map_odom.transform.rotation.y = 0  
            self.init_map_odom.transform.rotation.z = 0  
            self.init_map_odom.transform.rotation.w = 1  

        # print(self.init_map_odom)
        self.bc.sendTransform(self.init_map_odom)
def main():
    # initialize the node
    rospy.init_node('map_anchor')
    # create an instance of the class
    aruco_detection = ArucoDetection()  
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()