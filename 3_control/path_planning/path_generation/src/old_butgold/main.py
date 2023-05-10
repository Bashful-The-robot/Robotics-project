#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from Astar import Astar
import tf2_ros
import tf2_geometry_msgs
import ros_numpy
import numpy as np
import math
from robp_msgs.msg import flags

# How to test exploring:
#Publish a -1 to taskHandler. 
#In toDo_callback we'll then call the function 'gotoUnexplored'
    #This function will call A* to get the index of the node that is -1
    #This will be our goal position (in grid map)
    #Then we'll call the function 'path_planning', which will plan a path using A*
#If everything has been explored, we will publish a -1 to the topic taskHandler

# Help, use the filer publisher.py
# These will make sure that we end up in the explorer mode
# Note that the target position is not needed for testing the explorer


class PathControl:

    def __init__(self):
        #Initialize variables
        self.goal = None
        self.grid = None
        self.astarObj = None
        self.width = None
        self.height = None
        self.current_pos = None
        self.ToDoMsg = None
        #Used to transform the current position to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        #Subscribers
        rospy.Subscriber('/occupancyGridUpdate',OccupancyGrid, self.grid_callback)
        #rospy.Subscriber('/target_pos',PoseStamped, self.target_callback)     
        rospy.Subscriber('/state/cov', PoseWithCovarianceStamped, self.current_callback)
        rospy.Subscriber('/taskHandler', Int8, self.toDo_callback)
        rospy.Subscriber('/system_flags', flags, self.toDo_callback)

        #Publisher
        self.pubPath = rospy.Publisher('/path',Path,queue_size=10)
        self.pubSignal = rospy.Publisher('/taskHandler',Int8,queue_size=2)

    ###     Callbacks   ###
    def grid_callback(self,msg):
        width = msg.info.width
        height = msg.info.height
        self.grid = self.data_to_numpy(msg)
        #self.grid = ros_numpy.occupancy_grid.occupancygrid_to_numpy(msg)
        #print(self.grid)
        self.resolution = msg.info.resolution
        self.xmin = msg.info.origin.position.x
        self.ymin = msg.info.origin.position.y
        self.astarObj = Astar(self.grid,msg)
            
    # def target_callback(self,msg):        
    #     self.goal =msg.pose.position

    # def toDo_callback(self,msg):    
    #     #Differentiates between explorer mode and ordinary path planning
    #     self.msg = msg
    #     if msg == -1:   #Explorer mode
    #         self.gotoUnexplored()
    #     if msg == 1:    #Receive goal
    #         self.path_planning()
    #     elif msg == 0:   #landmark
    #         pass
    #         #self.path_planning(origin)
    #     else:
    #         pass
    def toDo_callback(self,msg):    
        #Differentiates between explorer mode and ordinary path planning
        self.ToDoMsg = msg.path_gen_mission
        self.goal =msg.target.position

    
    def current_callback(self,msg):      
        #Transforms the current position to map frame       
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0))
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
    

    ####        Exploring functions      ###
    def gotoUnexplored(self):
        node = self.astarObj.get_explorerNode()
        print(node)
        if node is not None:
            goalPosition = node
            self.path_planning(goalPosition)
        else:   #done with exploring -> publish a signal
            self.publishMsg()


    def publishMsg(self):   
        #Publishes a msg when done with exploring
        msg = Int8()
        msg.data = -1
        self.pubSignal.publish(msg)
        
    def explorer_mode(self):
        element = self.astarObj.get_explorerNode()


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
        

    def path_planning(self,goalPosition=None):
        if self.astarObj != None:
            if goalPosition == None:                                                #If No goal position has been received -> We're in exploring mode
                goalPosition = self.goal    
                goalPosition = self.astarObj.convert_to_grid(goalPosition)
            if self.current_pos != None and self.goal != None:    #Checks if it's possible to create a path
                path = self.astarObj.get_trajectory(self.current_pos, goalPosition)
                pathMsg = self.generate_pathMsg(path)
                self.pubPath.publish(pathMsg)

    def select_mode(self):
        #rospy.wait_for_message('toDo_callback',Int8)
        if self.ToDoMsg != None and self.astarObj != None:
            if self.ToDoMsg == int(-1):   #Explorer mode
                self.gotoUnexplored()
            if self.ToDoMsg == int(1):    #Receive goal
                self.path_planning()
            if self.ToDoMsg == int(2):    #Receive goal
                self.path_planning()
            elif self.ToDoMsg == 0:   #landmark
                pass
            else:
                pass

#Don't remove, might be used later
if __name__ == '__main__':
    rospy.init_node('path_generation')
    pathObj = PathControl()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        #athObj.path_planning()
        pathObj.select_mode()
        rate.sleep() 

        
     


        

     
   

            
