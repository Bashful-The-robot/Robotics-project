#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
from Astar import *
import tf2_ros
import tf2_geometry_msgs
import ros_numpy
import numpy as np
import math
from robp_msgs.msg import flags
import time
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class PathControl:

    def __init__(self):
        #Initialize variables
        self.mission =None
        self.goal = None
        self.targetPos = None       #Can be goal or something else (explorer, origin)
        self.current_pos = None

        self.geofence_points = None
        self.grid_geofence_points = None
        self.unreachableExplorerPoints = []     #Points that the explorer mode will avoid
        self.poly = None

        #Astar variables
        self.astarObj = None
        self.grid_msg = None        #msg used incallback
        self.new_grid = None        #the grid being sent to A*
        self.xmin = None
        self.ymin = None

        #Used to transform the current position to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #Subscribers
        rospy.Subscriber('/occupancyGridUpdate',OccupancyGrid, self.grid_callback,queue_size=1)
        rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.current_callback,queue_size=1)
        self.geofenceSub = rospy.Subscriber('/geofence', MarkerArray, self.geofence_callback,queue_size=1)
        rospy.Subscriber('/system_flags', flags, self.target_mission_callback,queue_size=1)

        #Publisher
        self.pubPath = rospy.Publisher('/path',Path,queue_size=1)
        self.pubSignal = rospy.Publisher('/system_flags', flags, queue_size=1)

    ###     Callbacks   ###
    def grid_callback(self,msg):
        self.grid_msg = msg
        self.resolution = msg.info.resolution
        self.xmin = msg.info.origin.position.x
        self.ymin = msg.info.origin.position.y


    def target_mission_callback(self,msg):
        self.mission = msg.path_gen_mission
        self.goal =msg.target.position

        if (self.grid_msg != None) and (self.mission != None):
            new_grid = self.process_grid()
            data = [self.resolution,self.xmin,self.ymin]

            self.astarObj = Astar(new_grid,data)
            if self.mission == int(-1):   #Explorer mode
                self.gotoUnexplored()

            elif self.mission == int(1):    #Receive goal
                self.path_planning()

            elif self.mission == int(2):       #Landmark
                self.path_planning()


    def geofence_callback(self,msg):
        if self.xmin != None and self.ymin != None:
            self.geofence_points = [(point.x, point.y) for point in msg.markers[0].points]
            self.poly = Polygon(self.geofence_points)                   #The workspace we should stay within (in map coordinates)
    
            # self.grid_geofence_points = [self.convert_to_grid(Node(point.x,point.y)) for point in msg.markers[0].points]  #geofence points in grid_coordinates
            # self.grid_geofence_points = list(sum(self.grid_geofence_points, ()))

            self.grid_geofence_points = [self.convert_to_grid(Node(point.x, point.y)) for point in msg.markers[0].points]  # geofence points in grid_coordinates
            self.grid_geofence_points = [self.convert_to_grid(Node(point.x, point.y)) for point in msg.markers[0].points]  # geofence points in grid_coordinates
            concatenated_points = []
            for sublist in self.grid_geofence_points:
                concatenated_points.extend(sublist)
            self.grid_geofence_points = concatenated_points

            min_x, min_y, max_x, max_y = self.poly.bounds
    
            #In order to create a smaller grid
            polyWidth = int((max_x - min_x)) + 1
            polyHeight = int((max_y - min_y)) + 1
            self.new_grid = np.full((polyWidth,polyHeight),100)
            self.geofenceSub.unregister()


    def current_callback(self,msg):
        #Transforms the current position to map frame
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", msg.header.stamp,rospy.Duration(2))
            self.current_pos = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform).pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: {e}")
            rospy.logwarn("Robot pose: {}".format(self.current_pos))

## Grid functions
    def isInsideWS(self,point):
        ##Checks if a point in map is inside the polygon

        #Flip coordinates since the coordinate systems are different
        y_m = point.x
        x_m = point.y
        point = Point(x_m,y_m)
        if self.poly.contains(point):
            return point
        else:
            return None

    def convert_to_grid(self,pos):
        #Converts the position in map frame into nodes in the gridmap
        xgrid = int((pos.x -self.xmin) / self.resolution)
        ygrid = math.ceil((pos.y - self.ymin) / self.resolution)
        return xgrid,ygrid

    # def process_grid(self):
    #     #Take in a grid map
    #     self.grid = ros_numpy.occupancy_grid.occupancygrid_to_numpy(self.grid_msg)
    #     try:
    #         if self.geofence_points != None:
    #             new_grid = np.full(self.grid.shape,100)
    #             r, c = polygon(self.grid_geofence_points[1::2], self.grid_geofence_points[::2])
    #             new_grid[r,c] = self.grid[r, c]
    #         return new_grid
    #     except:
    #         rospy.loginfo("The points representing the geofence are missing!")


    def process_grid(self):
        self.grid = ros_numpy.occupancy_grid.occupancygrid_to_numpy(self.grid_msg)
        # try:
        if self.grid_geofence_points != None:
            r, c = Polygon(self.grid_geofence_points[1::2], self.grid_geofence_points[::2])
            self.new_grid[r,c] = self.grid[r, c]
            return self.new_grid

        # except:
        #     rospy.loginfo("The points representing the geofence are missing!")

    ###     Path functions       ###
    def generate_pathMsg(self,path):
        pathMsg = Path()
        pathMsg.header.frame_id = 'map'
        pathMsg.header.stamp = rospy.Time.now()

        for node in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            #working
            pose.pose.position.y = (node.x * self.resolution) + self.xmin
            pose.pose.position.x = (node.y *self.resolution) + self.ymin
            pose.pose.position.z =0
            pathMsg.poses.append(pose)
        return pathMsg

    def path_planning(self,explorerLandmarkGoal=None):
        if self.astarObj != None:
            self.targetPos = explorerLandmarkGoal

            #If no goal given from the exploration or landmark-check, then we're pathplanning to given goal position
            if explorerLandmarkGoal == None:
                self.targetPos = Node(self.goal.x,self.goal.y)    #Target position is the goal position
                self.targetPos = self.astarObj.convert_to_grid(self.targetPos)

                if not self.isInsideWS(self.targetPos):
                    rospy.loginfo("Goal position is outside the workspace")
                    self.targetPos = None

            if self.current_pos != None and self.targetPos != None:    #Checks if it's possible to create a path
                path = self.astarObj.get_trajectory(self.current_pos, self.targetPos)
                if path != None:    #it was possible to find a path
                    print("-----------------------")
                    print("PATH CREATED TOWARDS " + str(self.targetPos.x) + ", " + str(self.targetPos.y))
                    print("-----------------------")

                    pathMsg = self.generate_pathMsg(path)
                    self.pubPath.publish(pathMsg)
                elif explorerLandmarkGoal != None:                         #If no path and in explorer mode
                    print("No path created, but unvalid explorer goal detected")
                    self.unreachableExplorerPoints.append(self.targetPos)  #Append the unreachable goal position
                else:
                    rospy.loginfo("Goal position was unreachable")


        ####        Exploring functions      ###
    def gotoUnexplored(self):
        node = self.astarObj.get_explorerNode()
        if node is not None:
            explorerGoal = node
            self.path_planning(explorerGoal)
        else:   #done with exploring -> publish a signal
            print("Done with exploring")
            self.publishMsg()

    def publishMsg(self):
        #Publishes a msg when done with exploring
        msg = flags()
        msg.explorer = True
        self.pubSignal.publish(msg)

    def explorer_mode(self):
        element = self.astarObj.get_explorerNode()


if __name__ == '__main__':
    rospy.init_node('path_control')
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():
        pathObj = PathControl()
        rate.sleep()
