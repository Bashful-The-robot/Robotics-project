#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


rospy.init_node('plot_path')
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)


def path_callback(path_msg):
    # Extract the points from the path message
    points = path_msg.poses

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.points = []
    for point in points:
        p = point.pose.position
        marker_point = Point()
        marker_point.x = p.x
        marker_point.y = p.y
        marker_point.z = p.z
        marker.points.append(marker_point)
    # Publish the marker
    marker_pub.publish(marker)


rospy.Subscriber('path_topic', Path, path_callback)

rospy.spin()
