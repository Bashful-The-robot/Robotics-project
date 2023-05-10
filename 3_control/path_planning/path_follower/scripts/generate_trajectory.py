#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped

from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs
import tf2_ros
import numpy as np

class GENPATH:
    def __init__(self) -> None:
        rospy.init_node('path_generator_node')
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.rate = rospy.Rate(10)  # publish at 1 Hz
        rospy.Subscriber('/marker2', MarkerArray, self.callback)
        rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.callback_stat)
        self.xx = None 
        self.yy = None
        self.zz = None
        self.robx = None
        self.roby = None
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(5)    
        self.run = False
        self.once = False
        self.path_msg = None
        self.xx = 1.0
        self.yy = 1.0
        self.zz = 0
        while not self.run: 
            if self.robx != None and self.roby != None and self.xx != None and self.yy != None:
                print("HI")
                self.generate_path()
                self.rate.sleep()
            else:
                self.rate.sleep()
        while not rospy.is_shutdown() and self.run:
            self.path_pub.publish(self.path_msg)
            
    def generate_path(self):
        

        self.run = True
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = 'map'   
        #num_waypoints = 121  # 60 points spaced 5 cm apart, plus the starting point at 0.0
        distance_between_waypoints = 0.05
        tot_X = self.xx - self.robx  
        tot_Y = self.yy - self.roby
        theta = np.arctan2(tot_Y,tot_X)
        i = 0
        x_ = 0
        y_ = 0
        temp_x = tot_X
        temp_y = tot_Y
        print(self.xx, x_, self.yy, y_)

        while np.sqrt(temp_x**2 + temp_y**2) > 0.15 and not rospy.is_shutdown():
        #for i in range(0,121):
        #for i in range(num_waypoints):
            print(np.sqrt(temp_x**2 + temp_y**2))
            waypoint = PoseStamped()
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = 'map'
            x_ = i*distance_between_waypoints*np.cos(theta)*tot_X
            y_ = i*distance_between_waypoints*np.sin(theta)*tot_Y
            temp_x = self.xx - x_
            temp_y = self.yy - y_
            print(self.xx, x_)
            #waypoint.pose.position = Point(x=0.0 , y=i*distance_between_waypoints-3.0, z=0.0)
            waypoint.pose.position = Point(x= x_, y= y_, z=0.0)
            self.path_msg.poses.append(waypoint)
            i += 1
        self.path_pub.publish(self.path_msg)
    
        """
        for i in range(60,121):
            waypoint = PoseStamped()
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position = Point(x=i*distance_between_waypoints-3.0, y=0.0, z=0.0)
            path_msg.poses.append(waypoint)
        return path_msg"""
            

    def callback_stat(self,msg):
        self.robx = msg.pose.pose.position.x
        self.roby = msg.pose.pose.position.y     
        

    def callback(self, msg):
        if not self.once:
            for marker in msg.markers:
                #print("marker", marker)
                try:
                    trans = self.tfBuffer.lookup_transform("map",marker.header.frame_id, rospy.Time.now(), rospy.Duration(0.5))
                    rospy.loginfo("here")
                    do_trans = tf2_geometry_msgs.do_transform_pose(marker, trans)
                    #self.xx = marker.pose.position.x
                    #self.yy = marker.pose.position.y
                    #self.zz = marker.pose.position.z
                    self.xx = do_trans.pose.position.x
                    self.yy = do_trans.pose.position.y
                    self.zz = do_trans.pose.position.z
                    
                    #print(self.xx)
                    self.once = True

                except:
                    rospy.loginfo("No transform aruco")

if __name__ == '__main__':
    
    path_msg = GENPATH()
    while not rospy.is_shutdown():
        # Generate and publish the path
        rospy.spin()


        #rate.sleep()
