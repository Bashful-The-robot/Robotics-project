#!/usr/bin/env python3

import rospy
import numpy as np

from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as vMarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import tf2_ros

import tf2_geometry_msgs

class Posepub:
    def __init__(self):
        self.rate = rospy.Rate(10)
        
        #subscriber

        #publisher
        self.pub = rospy.Publisher('/pospub', PointStamped, queue_size=10)

        ps = PointStamped()
        ps.header.frame_id = '6'
        ps.header.stamp = rospy.Time.now()
        ps.point.x = 1
        ps.point.y = 1
        ps.point.z = 0
        self.pub.publish(ps)
        self.rate.sleep()


def main():
    rospy.init_node('pospubTest')
    while not rospy.is_shutdown():
        Posepub()
    rospy.spin()

    print("Shutting down")



if __name__ == '__main__':
    main()
