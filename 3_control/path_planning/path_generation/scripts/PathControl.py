#!/usr/bin/env python3


import matplotlib.pyplot as plt
from skimage.draw import polygon

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
# How to test exploring:
#Publish a -1 to topic \pathMission. 
#In the callback we'll then call the function 'gotoUnexplored'
    #This function will call A* to get the index of the node that is -1
    #This will be our goal position (in grid map)
    #Then we'll call the function 'path_planning', which will plan a path using A*
#If everything has been explored, we will publish a -1 to the topic taskHandler

# Help:
# Use the filer publisher.py
# These will make sure that we end up in the explorer mode
# Note that the target position is not needed for testing the explorer

class PathControl:

    def __init__(self):
        #rospy.init_node('path_control')
        
        #Initialize variables
        self.mission =None
        self.goal = None
        self.targetPos = None       #Can be goal or something else (explorer, origin)
        self.current_pos = None
        self.dist = 0.15            #Current position should be at least 15 cm from target position after path planning
        self.ToDoMsg = int(-1)      #Keep track of which tasks to do, explorer mode by default
        self.stopMotors =False
        self.geofence_points = None
        self.unreachableExplorerPoints = []     #Points that the explorer mode will avoid
        
        #Astar variables
        self.astarObj = None
        self.width = None
        self.height = None
        self.grid_msg = None        #msg used incallback
        
        self.xmax =4.42
        self.ymax =9.79

        #Used to transform the current position to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        #Subscribers
        rospy.Subscriber('/occupancyGridUpdate',OccupancyGrid, self.grid_callback,queue_size=1)
        rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.current_callback,queue_size=1)  
        rospy.Subscriber('/geofence', MarkerArray, self.geofence_callback,queue_size=1)
        rospy.Subscriber('/system_flags', flags, self.target_mission_callback,queue_size=1)

        #Publisher
        self.pubPath = rospy.Publisher('/path',Path,queue_size=1)
        self.pubSignal = rospy.Publisher('/system_flags', flags, queue_size=1) 

    ###     Callbacks   ###
    def process_grid(self):
        self.grid = ros_numpy.occupancy_grid.occupancygrid_to_numpy(self.grid_msg)

        if self.geofence_points != None:
            new_grid = np.full(self.grid.shape,100)
            self.grid_geofence_points = [self.convert_to_grid(Node(point.x,point.y)) for point in self.geofence_msg.markers[0].points]  #geofence points in grid_coordinates
            self.grid_geofence_points = list(sum(self.grid_geofence_points, ()))  
            r, c = polygon(self.grid_geofence_points[1::2], self.grid_geofence_points[::2])
            new_grid[r,c] = self.grid[r, c]
        else:
            new_grid =  None
        return new_grid
            
    
    def grid_callback(self,msg):
        self.grid_msg = msg
        self.resolution = msg.info.resolution
        self.xmin = msg.info.origin.position.x
        self.ymin = msg.info.origin.position.y
    

    def target_mission_callback(self,msg):
        self.mission = msg.path_gen_mission
        print("LA MISION ES",self.mission)
        self.goal =msg.target.position

        if (self.grid_msg != None) and (self.mission != None):
            new_grid = self.process_grid()
            if new_grid is not None:
                data = [self.resolution,self.xmin,self.ymin]

                self.astarObj = Astar(new_grid,data,self.geofence_points)
                if self.mission == int(-1):   #Explorer mode
                    self.gotoUnexplored()
                    self.mission = None
                    print("he entrado en explorer mode")

                elif self.mission == int(1):    #Receive goal
                    print("he entrado en receive goal")
                    self.path_planning()
                    self.mission = None
                elif self.mission == int(2):       #Landmark
                    self.path_planning()
                    self.mission = None


    def geofence_callback(self,msg):
        self.geofence_msg = msg
        self.geofence_points = [(point.x, point.y) for point in msg.markers[0].points]
    
   
    def convert_to_grid(self,pos):
        #Converts the position in map frame into nodes in the gridmap
        xgrid = int((pos.x -self.xmin) / self.resolution)
        ygrid = math.ceil((pos.y - self.ymin) / self.resolution)
        return xgrid,ygrid
    
    def convert_to_map(self,pos):
    #Converts the position from grid map into coordinates in the map frame
        x_g = pos.x
        y_g = pos.y
        x_m = x_g*self.resolution+ self.xmin
        y_m = y_g*self.resolution+ self.ymin
        return (x_m, y_m)
    

    def current_callback(self,msg):      
        #Transforms the current position to map frame       
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", msg.header.stamp,rospy.Duration(2))
            self.current_pos = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform).pose.position
            #print(self.current_pos)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: {e}")
            rospy.logwarn("Robot pose: {}".format(self.current_pos))
    

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
            pose.pose.position.y = (node.x * self.resolution) + self.ymin
            pose.pose.position.x = (node.y *self.resolution) + self.xmin
            pose.pose.position.z =0
            pathMsg.poses.append(pose)    
        return pathMsg    

    def path_planning(self,explorerLandmarkGoal=None):
        if self.astarObj != None:
            self.targetPos = explorerLandmarkGoal
            

            #If no goal given from the exploration or landmark-check, then we're pathplanning to given goal position
            if explorerLandmarkGoal == None:                                             
                self.targetPos = self.goal    #Target position is the goal position
                self.targetPos = self.astarObj.convert_to_grid(self.targetPos)
                self.targetPos = self.astarObj.flip(self.targetPos)
                if not self.astarObj.isInsideWS(self.targetPos):
                    rospy.loginfo("Goal position is outside the workspace")
                    self.targetPos = None
            if self.targetPos == explorerLandmarkGoal:
                print("la posicion es una mierda")
            

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
                

    # def atGoal(self,goal):      
    #     isClose = False
    #     if goal == 0:
    #         goal = Node(0,0)
    #     closeInX = math.isclose(goal.x,self.current_pos.x,abs_tol=self.dist)
    #     closeInY = math.isclose(goal.y,self.current_pos.y,abs_tol=self.dist)
    #     if closeInX and closeInY:
    #         isClose = True                                              #True if at goal
    #     return isClose
    
        ####        Exploring functions      ###
    def gotoUnexplored(self):
        node = self.astarObj.get_explorerNode()
        if node is not None:
            explorerGoal = node
            print("se ha ejecutado esta mierda")
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

        
        
     


        

     
   

            
