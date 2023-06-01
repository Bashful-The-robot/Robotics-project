#!/usr/bin/env python3

import rospy
import numpy as np

from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as vMarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf
import tf2_geometry_msgs
import math
from tf2_msgs.msg import TFMessage

class AMdetector:
    def __init__(self):
        self.rate = rospy.Rate(10)
        #subscriber
        #rospy.Subscriber('/camera/aruco/markers',MarkerArray,self.marker_callback)
        rospy.Subscriber('/tf',TFMessage,self.tf_callback)
        
        #publisher
        self.marker_pub = rospy.Publisher('/marker', vMarkerArray, queue_size=10)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.marker_array = None

        self.init_aruco = np.zeros(4)
        self.marker_array = vMarkerArray()

        while not rospy.is_shutdown():
            self.rate.sleep()



    def tf_callback(self,msg):
        list = ['landmark1','landmark2','landmark3']
        self.marker_array = vMarkerArray()

        for tf in msg.transforms:
            if tf.child_frame_id in list:
                try:
                    temp = PoseStamped()
                    temp.pose.position.x = tf.transform.translation.x
                    temp.pose.position.y = tf.transform.translation.y
                    temp.pose.position.z = tf.transform.translation.z
                    temp.pose.orientation= tf.transform.rotation
                    trans = self.buffer.lookup_transform("camera_color_optical_frame","odom", tf.header.stamp, rospy.Duration(1.5))

                    do_trans = tf2_geometry_msgs.do_transform_pose(temp, trans)

                    z = do_trans.pose.position.z
                    x = do_trans.pose.position.x
                    theta = math.atan2(z,x)
                    #do_trans.pose.position.z += 0.05*math.sin(theta)
                    #do_trans.pose.position.x += 0.05*math.cos(theta)
                    trans = self.buffer.lookup_transform("map","camera_color_optical_frame", tf.header.stamp, rospy.Duration(0.5))
                    do_trans = tf2_geometry_msgs.do_transform_pose(do_trans, trans)
                    self.marker = Marker()
                    self.marker.header.frame_id = "map"
                    self.marker.ns = "aruco_" + str(tf.child_frame_id)
                    self.marker.header.stamp = tf.header.stamp
                    self.marker.id = int(tf.child_frame_id[-1])
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
                    
                except:
                    rospy.loginfo("No transform from map to camera") 
        self.pubMarker()



    '''def marker_callback(self, msg):
        self.marker = msg.markers
        for marker in self.marker:
            if marker.id <= 3 and marker.id != 0 : # id 500 is the map definition marker
                if not self.init_aruco[marker.id]:
                    if marker.pose.pose.position.z <= 1.5:
                        try:
                            #           Offset marker!
                            z=marker.pose.pose.position.z
                            x=marker.pose.pose.position.x
                            theta = math.atan2(z,x)
                            marker.pose.pose.position.z += 0.08*math.sin(theta)
                            marker.pose.pose.position.x += 0.08*math.cos(theta)

                            trans = self.buffer.lookup_transform("map","camera_color_optical_frame", msg.header.stamp, rospy.Duration(0.5))
                            do_trans = tf2_geometry_msgs.do_transform_pose(marker.pose, trans)
                            self.marker = Marker()
                            self.marker.header.frame_id = "map"
                            self.marker.header.stamp = msg.header.stamp
                            self.marker.ns = "aruco_" + str(marker.id)
                            self.marker.id = marker.id 
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
                            self.init_aruco[marker.id] = 1
                        except:
                            rospy.loginfo("AM detector: No transform from map to camera")       
        self.pubMarker()'''

    def pubMarker(self):
        if self.marker_array != None:
            self.marker_pub.publish(self.marker_array)

def main():
    rospy.init_node('AM_detector')
    am = AMdetector()
    while not rospy.is_shutdown():
        rospy.spin()
    print("Shutting down")



if __name__ == '__main__':
    main()
