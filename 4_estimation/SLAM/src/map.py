#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
#from math import abs

# Create class that updates the occupancy grid
class Grid:
    '''This class updates the occupancy grid, does no calculations'''
    def __init__(self):
        self.rate = rospy.Rate(10)
        #init subscriber to /marker topic
        self.pub = rospy.Publisher('/occupancygrid', OccupancyGrid, queue_size=1)
        self.sub = rospy.Subscriber('/occupancyGridUpdate', OccupancyGrid)
        self.workspace =[]
        self.frame_id = None
        self.stamp = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None
        self.data = None
        self.orientation = None
        self.info = False
        self.grid = None

    def callback(self,msg):

        self.info = True
        self.grid= msg.data
        self.frame_id= msg.header.frame_id
        self.stamp= msg.header.stamp
        self.resolution= msg.info.resolution
        self.width= msg.info.width
        self.height= msg.info.height
        self.origin= msg.info.origin
        self.orientation= msg.info.origin.orientation



    def publishMap(self):
        if self.info:
            occupancy_grid = OccupancyGrid()
            occupancy_grid.header.frame_id = self.frame_id
            occupancy_grid.header.stamp = self.stamp
            occupancy_grid.info.resolution = self.resolution
            occupancy_grid.info.width = self.width
            occupancy_grid.info.height = self.height
            occupancy_grid.info.origin.position = self.origin.position
            occupancy_grid.info.origin.orientation = self.orientation
    
            occupancy_grid.data = self.grid
            # publish occupancy grid
            self.pub.publish(occupancy_grid)
        
        
        
    








if __name__ == '__main__':
    rospy.init_node('map', anonymous=True)
    occupancygrid = Grid()
    while not rospy.is_shutdown():
        occupancygrid.publishMap()
        occupancygrid.rate.sleep()
    rospy.spin()