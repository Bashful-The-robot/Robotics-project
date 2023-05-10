#!/usr/bin/env python3

import rospy
import numpy as np

from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as vMarkerArray
from visualization_msgs.msg import Marker
import tf2_ros

import tf2_geometry_msgs

class AMdetector:
    def __init__(self):
        self.rate = rospy.Rate(10)
        
        #subscriber

        #publisher
        self.marker_pub = rospy.Publisher('/origo', vMarkerArray, queue_size=10)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.marker_array = None

        self.init_aruco = np.zeros(3)
        self.marker_array = vMarkerArray()
            
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "origo" 
        self.marker.id = 0 
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 1
        self.marker.pose.position.z = 0 
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker_array.markers.append(self.marker)
        #print(f"self.marker_array.markers: {self.marker_array.markers}")
        
        self.marker_pub.publish(self.marker_array)
        print("Published marker")

def main():
    rospy.init_node('origoTest')
    while not rospy.is_shutdown():
        AMdetector()
    rospy.spin()

    print("Shutting down")



if __name__ == '__main__':
    main()
