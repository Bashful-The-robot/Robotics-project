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
from robp_msgs.msg import DutyCycles        #CHANGE

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
        self.goal = None
        self.targetPos = None       #Can be goal or something else (explorer, origin)
        self.current_pos = None
        self.dist = 0.15            #Current position should be at least 15 cm from target position after path planning
        self.ToDoMsg = int(-1)     #Keep track of which tasks to do, explorer mode by default
        self.stopMotors =False
        self.geofence_points = None
        self.unreachableExplorerPoints = []     #Points that the explorer mode will avoid
        #Astar variables
        self.grid = None
        self.astarObj = None
        self.width = None
        self.height = None
        
        #Used to transform the current position to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        #Subscribers
        rospy.Subscriber('/occupancyGridUpdate',OccupancyGrid, self.grid_callback)
        rospy.Subscriber('/target_pos',PoseStamped, self.target_callback)     
        rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.current_callback)
        rospy.Subscriber('/pathMission', Int8, self.mission_callback)
        rospy.Subscriber('/geofence', MarkerArray, self.geofence_callback)
        rospy.Subscriber('/runMotors', Bool, self.runMotor_callback)
        
        #Publisher
        self.pubPath = rospy.Publisher('/path',Path,queue_size=10)
        self.pubDelta = rospy.Publisher('motor/duty_cycles', DutyCycles, queue_size=10)
        self.pubSignal = rospy.Publisher('explorerDone', Bool, queue_size=2)

    ###     Callbacks   ###
    def geofence_callback(self,msg):
        self.geofence_points = [(point.x, point.y, point.z) for point in msg.markers[0].points]
        # points = []
        # for marker in msg:
        #     for point in msg.points:
        #         points.append((point.x, point.y))


    def grid_callback(self,msg):
        width = msg.info.width
        height = msg.info.height
        self.grid = self.data_to_numpy(msg)
        #self.grid = ros_numpy.occupancy_grid.occupancygrid_to_numpy(msg)
        #print(self.grid)
        self.resolution = msg.info.resolution
        self.xmin = msg.info.origin.position.x
        self.ymin = msg.info.origin.position.y
        
        if len(self.unreachableExplorerPoints) >0:
            for node in self.unreachableExplorerPoints:
                x,y = node.x,node.y
                self.grid[x][y] = 100           #Making sure that unreachable points won't be explored

        if self.geofence_points != None:
            self.astarObj = Astar(self.grid,msg,self.geofence_points)
            
    def target_callback(self,msg):        
        self.goal =msg.pose.position

    def mission_callback(self,msg):    
        self.mission = msg.data
        
        if self.mission != None and self.astarObj != None:
            if self.mission == int(-1):   #Explorer mode
                print("Mission received")
                self.gotoUnexplored()
                
            elif self.mission == int(1):    #Receive goal
                self.path_planning()

            elif self.mission == 0:       #Landmark
                goal = Node(0,0)          #Goal is origin in map
                self.path_planning(goal)

            else:
                print("Should not end up here")
    

    def runMotor_callback(self,msg= True):
        delta_left_right = 0
        stopMsg = DutyCycles(None,delta_left_right, delta_left_right)

        if msg == False:                  
            self.pubDelta.publish(stopMsg)



    def current_callback(self,msg):      
        #Transforms the current position to map frame       
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(2))
            self.current_pos = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform).pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: {e}")
            rospy.logwarn("Robot pose: {}".format(self.current_pos))

    #Converts the gridmap into a numpy array
    def data_to_numpy(self,data):
        width = data.info.width
        height = data.info.height
        occupancy_data = np.zeros((height, width), dtype=np.float32) # Create an empty NumPy array
        for i, element in enumerate(data.data):
            if element == int(-1):
                occupancy_data[i // width, i % width] = int(-1)     # If element is -1, replace it with -1 again (without this step, everything disappears)
            else:
                occupancy_data[i // width, i % width] = element     # Otherwise, set the value directly
        return occupancy_data
    

    ###     Path functions       ###
    def generate_pathMsg(self,path):
        pathMsg = Path()
        pathMsg.header.frame_id = 'map'
        pathMsg.header.stamp = rospy.Time.now()
    
        for node in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x =node.x * self.resolution + self.xmin
            pose.pose.position.y =node.y *self.resolution + self.ymin
            pose.pose.position.z =0
            pathMsg.poses.append(pose)    
        return pathMsg    

    def path_planning(self,explorerLandmarkGoal=None):
        if self.astarObj != None:
            self.targetPos = explorerLandmarkGoal
            #If no goal given from the exploration or landmark-check
            if explorerLandmarkGoal == None:                                             
                self.targetPos = self.goal    #Target position is the goal position
                self.targetPos = self.astarObj.convert_to_grid(self.targetPos)
                if not self.astarObj.isInsideWS(self.targetPos):
                    self.targetPos = None
                    rospy.loginfo("Goal position is outside the workspace")

            if self.current_pos != None and self.targetPos != None:    #Checks if it's possible to create a path
                #print("gonna call the path planner")
                path = self.astarObj.get_trajectory(self.current_pos, self.targetPos)
                print("returned from path planner")
                if path != None:    #it was possible to find a path
                    print("path has been created")
                    pathMsg = self.generate_pathMsg(path)
                    self.pubPath.publish(pathMsg)
                elif explorerLandmarkGoal != None:                         #If no path and in explorer mode
                    print("No path created, but unvalid explorer goal detected")
                    self.unreachableExplorerPoints.append(self.targetPos)  #Append the unreachable goal position
                else:
                    rospy.loginfo("Goal position was unreachable")
                

    def atGoal(self,goal):      
        isClose = False
        if goal == 0:
            goal = Node(0,0)
        closeInX = math.isclose(goal.x,self.current_pos.x,abs_tol=self.dist)
        closeInY = math.isclose(goal.y,self.current_pos.y,abs_tol=self.dist)
        if closeInX and closeInY:
            isClose = True                                              #True if at goal
        return isClose
    
        ####        Exploring functions      ###
    def gotoUnexplored(self):
        node = self.astarObj.get_explorerNode()
        if node is not None:
            print("FOUND GOAL NODE")
            explorerGoal = node
            self.path_planning(explorerGoal)
        else:   #done with exploring -> publish a signal
            print("Done with exploring")
            self.publishMsg()

    def publishMsg(self):   
        #Publishes a msg when done with exploring
        msg = Bool()
        msg.data = True
        self.pubSignal.publish(msg)
        
    def explorer_mode(self):
        element = self.astarObj.get_explorerNode()

                        
if __name__ == '__main__':
    rospy.init_node('path_control')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pathObj = PathControl()
        rate.sleep() 

        
        
     


        

     
   

            
