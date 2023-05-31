#!/usr/bin/env python3

import rospy
import tf2_ros
from tf import transformations

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped

from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

import tf2_geometry_msgs

# make this node a class that creates a pulisher and a subscriber
class visualize_marker:
    def __init__(self):
        #Subscriber
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.callback)
        self.sub = rospy.Subscriber('/aruco_single/pose',PoseStamped, self.aruco_callback)

        #Publisher
        self.marker_pub = rospy.Publisher('/marker', MarkerArray, queue_size=100)

        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.msg_header = None
        self.inversed_transform = None
        
        self.rate = rospy.Rate(0.5)

        self.anchor_det = False

        self.init_aruco = False
        while not rospy.is_shutdown():      
            self.rVizFunc()
            self.rate.sleep()

    def aruco_callback(self, msg):
        self.anchor_det = True
        if not self.init_aruco:
            try:
                print("trying")
                trans = self.buffer.lookup_transform("map","camera_color_optical_frame", msg.header.stamp, rospy.Duration(0.5))
                do_trans = tf2_geometry_msgs.do_transform_pose(msg, trans)
                self.marker = Marker()
                self.marker.header.frame_id = "map"
                self.marker.header.stamp = msg.header.stamp
                self.marker.ns = "aruco_" + str(500)
                self.marker.id = 500
                self.marker.type = Marker.CUBE
                self.marker.action = Marker.ADD

                self.marker.pose.position = do_trans.pose.position
                self.marker.pose.orientation = do_trans.pose.orientation

                self.marker.scale.x = 0.05
                self.marker.scale.y = 0.05
                self.marker.scale.z = 0.05
                self.marker.color.a = 1.0
                self.marker.color.r = 1.0
                self.marker.color.g = 0.0
                self.marker.color.b = 0.0
                self.marker_array.markers.append(self.marker)
                self.init_aruco = True
                print(self.init_aruco)
            except:
                rospy.loginfo("No transform from map to camera")

    def callback(self, msg):
        if msg.transforms[0].header.frame_id == "map" and self.anchor_det:
            try:
                transform = transformations.concatenate_matrices(transformations.translation_matrix([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z]), transformations.quaternion_matrix([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]))
                self.inversed_transform = transformations.inverse_matrix(transform)
                self.msg_header = msg.transforms[0].header.stamp
            except:
                rospy.loginfo("No transform from odom to map")
           
    def rVizFunc(self):
        self.marker_array = MarkerArray()
        if self.init_aruco: 
            #if np.any(self.inversed_transform) != None:
            #self.marker_pos = transformations.translation_from_matrix(self.inversed_transform)
            #self.marker_ori = transformations.quaternion_from_matrix(self.inversed_transform)
            self.marker = Marker()
            self.marker.header.frame_id = "map"
            self.marker.header.stamp = self.msg_header
            self.marker.ns = "aruco_anchor"
            self.marker.id = 500
            self.marker.type = Marker.CUBE
            self.marker.action = Marker.ADD

            self.marker.pose.position.x = 0 #self.marker_pos[0]
            self.marker.pose.position.y = 0 #self.marker_pos[1]
            self.marker.pose.position.z = 0 #self.marker_pos[2]
            self.marker.pose.orientation.x = 0 #self.marker_ori[0]
            self.marker.pose.orientation.y = 0 #self.marker_ori[1]
            self.marker.pose.orientation.z = 0 #self.marker_ori[2]
            self.marker.pose.orientation.w = 1 #self.marker_ori[3]

            self.marker.scale.x = 0.05
            self.marker.scale.y = 0.05
            self.marker.scale.z = 0.05
            self.marker.color.a = 1.0
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
            self.marker_array.markers.append(self.marker)
            self.marker_pub.publish(self.marker_array)

def main():
    # initialize the node
    rospy.init_node('marker_visualization')
    # create an instance of the class
    vis_marker = visualize_marker()  
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()