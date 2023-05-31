#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped,PoseStamped
from sensor_msgs.msg import PointCloud2,LaserScan
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import numpy as np
import math
from math import atan2,cos,sin,fabs
from numpy import sign
import tf2_geometry_msgs
import sys
#from scipy.ndimage.morphology import binary_dilation
from scipy.ndimage import binary_dilation, gaussian_filter
import matplotlib.pyplot as plt
# create class that discretizes the workspace 
class Workspace:
    def __init__(self):

        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)

    # Publisher for the occupancy grid

        self.pub = rospy.Publisher('/occupancyGridUpdate', OccupancyGrid, queue_size=10)

    # Subscribers

        rospy.Subscriber('/scan', LaserScan, self.laser_callback,queue_size=1)
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.cov_cb)
        rospy.Subscriber('/geofence',MarkerArray,self.marker_callback)
        rospy.Subscriber('/marker2', MarkerArray, self.objectCallback)
        rospy.Subscriber('/marker', MarkerArray, self.aruco_cb)

    # Variables
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.xmin = None
        self.ymin = None

        
        self.marker = None
        self.grid_data = None
        self.oldGrid = None
        self.angleMapOdom = None
        self.prevGrid = None
        self.occGrid = None
        self.occMask = None
        self.RobPose = PoseStamped()

        self.workspace_seen = False
        self.boundary = False
        self.ok2pubList = False
        self.grid_available = False
        self.givenRobotPose = False 
        self.prevGridExist = False
        
        self.objNameArray = []
        self.workspacePoints = []
        self.cubePose = []
        self.markerPose = []
    
        self.rate = rospy.Rate(2)
        
        self.init_ard_rob = True


    def aruco_cb(self, msg):
        # get the position of the aruco marker
        if not self.grid_available:
            rospy.loginfo("NO GRID (IN CB ARUCO)")
            return
        self.markerPose = []
        self.marker = msg.markers
        for marker in self.marker:
            if marker.id != 500 and marker.id != 0:
                
                x = marker.pose.position.x
                y = marker.pose.position.y
                
                xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
                ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                
                self.markerPose.append([xgrid,ygrid])
        rospy.loginfo(f"The marker pose: {self.markerPose},xmin,ymin ,({self.xmin},{self.ymin} & maxes: ({self.xmax},{self.ymax}))")



    def objectCallback(self,msg):
        #WILL NOT HAVE TO HANDLE MUPLITIPLES OF SAME OBJECT!!  MarkerArray

        #THIS WILL HAVE TO CHANGE, CORNELIA IS SENDING IN A MARKERARRAY WITH ALL OBJECTS
        
        if not self.grid_available: # We have not initialized all parameters yet
            rospy.loginfo("NO GRID, Object_cb")
            return
        
        self.cubePose = []

        for marker in msg.markers:

            try:
                #mapView = self.buffer.lookup_transform("map","camera_color_optical_frame",marker.header.stamp, rospy.Duration(0.5))
                #do_trans = tf2_geometry_msgs.do_transform_pose(marker, mapView)

                x = marker.pose.position.x#do_trans.pose.position.x
                y = marker.pose.position.y#do_trans.pose.position.y
                
                xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
                ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

                #if [xgrid,ygrid] not in self.cubePose:# and marker.id not in self.objNameArray:
                #    self.objNameArray.append(marker.id)
                #    self.cubePose.append([xgrid,ygrid])

                self.cubePose.append([xgrid,ygrid])
                self.ok2pubList = True
                
            except:

                print("No transform found")


    def cov_cb(self,msg):
        '''Gets angle and position from robot in base_link, have to convert to map.'''
        try:
            trans = self.buffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
            do_trans = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
            self.RobPose = do_trans
            self.givenRobotPose = True
        except Exception as e:
            rospy.logerr(f"{e}")
#        self.RobPose = msg.pose



    def laser_callback(self,msg):
        '''Think it works as a windshield wiper, scanning the range. Need initial angle'''
        
    #Make sure self.width and heigh seen and defined!
        if not self.grid_available or not self.givenRobotPose:
            rospy.loginfo(f"laser cb grid;{self.grid_available}, Robot:{self.givenRobotPose}")
            return
        maxrange = 1.5 #meters
        ranges = msg.ranges
        
        dTheta = msg.angle_increment
        angMin = msg.angle_min
        rangeMin = msg.range_min
        rangeMax = msg.range_max
        
        
    # Looking up position of robot and orientation
        try:
            if self.givenRobotPose:

                mapView = self.buffer.lookup_transform("map","camera_depth_frame",msg.header.stamp, rospy.Duration(0.5))
                #rot = [mapView.rotation.x,mapView.transform.rotation,mapView.transform.rotation.z,mapView.rotation.w]

                rot = [self.RobPose.pose.orientation.x,self.RobPose.pose.orientation.y,self.RobPose.pose.orientation.z,self.RobPose.pose.orientation.w]

                (rollrob,pitchrob,yawrob) = euler_from_quaternion(rot)
                maprot = [mapView.transform.rotation.x, mapView.transform.rotation.y, mapView.transform.rotation.z, mapView.transform.rotation.w]
                (rollmap,pitchmap,yawmap) = euler_from_quaternion(maprot)

                newrot = [rollrob + rollmap, pitchrob + pitchmap, yawrob+yawmap]                                                          
                
                self.angleMapOdom = yawrob#newrot[2]

                #xRob = mapView.transform.translation.x   # self.robX # do_trans.pose.position.x
                #yRob = mapView.transform.translation.y   #self.robY # do_trans.pose.position.y
                xRob = self.RobPose.pose.position.x
                
                yRob = self.RobPose.pose.position.y
                 

                #xgrid = math.ceil((xtemp-self.xmin)/(self.xmax-self.xmin)*self.width)
                #ygrid = math.ceil((yRob-self.ymin)/(self.ymax-self.ymin)*self.height)
                #gridrayStart
                # xRayStart = math.ceil((xRob+0.3-self.xmin)/(self.xmax-self.xmin)*self.width)
                # yRayStart = math.ceil((yRob+0.3-self.ymin)/(self.ymax-self.ymin)*self.height)
                #xRayStart = math.ceil((xtemp-self.xmin)/(self.xmax-self.xmin)*self.width)
                #yRayStart = math.ceil((yRob-self.ymin)/(self.ymax-self.ymin)*self.height)

                self.rangeMaxed = False

                ranges=np.asarray(ranges)

                for i in range(len(ranges)):
                    if math.isnan(ranges[i]):
                        continue

                    if ranges[i] > rangeMin and ranges[i] < rangeMax:
                        if ranges[i]> maxrange:
                            self.rangeMaxed = True
                            ranges[i] = maxrange

                        x = xRob + ranges[i]*cos(angMin + (dTheta*i) + self.angleMapOdom)
                        xRayStart = xRob + 0.3*cos(angMin + (dTheta*i) + self.angleMapOdom)
                        xRay = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)

                        y = yRob + ranges[i]*sin(angMin + (dTheta*i) + self.angleMapOdom)
                        yRayStart = yRob + 0.3*sin(angMin + (dTheta*i) + self.angleMapOdom)
                        yRay = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

                        xRayStart = math.ceil((xRayStart-self.xmin)/(self.xmax-self.xmin)*self.width)
                        yRayStart = math.ceil((yRayStart-self.ymin)/(self.ymax-self.ymin)*self.height)

                        if not self.rangeMaxed and self.grid_data[xRay,yRay] != 101 :
                            self.grid_data[xRay,yRay] = 100

                        elif self.rangeMaxed and self.grid_data[xRay,yRay] != 101:
                            self.grid_data[xRay,yRay] = 0
                            self.rangeMaxed = False

                        #self.raytrace((xgrid,ygrid),(xRay,yRay))
                        self.raytrace((xRayStart,yRayStart),(xRay,yRay))


        except Exception as e:
            print(f"Error:  {e}")

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))
            #if self.grid_data[x,y] != 101:
            #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
            
            #if [x,y] not in self.cubePose and self.grid_data[x,y]<10:
            #if [x,y] not in self.cubePose:
            if [x,y] not in self.cubePose and self.prevGrid[x,y]<10 and [x,y] not in self.markerPose:

                self.grid_data[x,y] = 0

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                    #if self.grid_data[x+x_inc,y] != 101:
                    #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
                    if [x,y] not in self.cubePose and self.prevGrid[x,y]<10 and [x,y] not in self.markerPose:
                    #if [x,y] not in self.cubePose:
                    
                        self.grid_data[x + x_inc,y] = 0
                y += y_inc
                error += dx
        return traversed


    def marker_callback(self,msg):
        '''This function gets the height and width of the workspace'''
    # If seen, no need to continue!
        if self.workspace_seen:
            return
        if len(msg.markers) == 0:
            rospy.loginfo("marker_cb empty!")
            return
        
        self.marker=msg.markers[0]
    
    # Look for the different coordianates of the marker
        maxX = -np.inf
        xmin = np.inf
        maxY = -np.inf
        ymin = np.inf
        if msg.markers[0].ns == "workspace":
            for point in msg.markers[0].points:
                x = point.x
                y = point.y
                self.workspacePoints.append([x,y])
                if x > maxX:
                    maxX = x
                if y > maxY:
                    maxY = y
                if x < xmin:
                    xmin = x
                if y < ymin:
                    ymin = y
          
    # Save the height and width and other parameters of the workspace.
            self.height = math.ceil((maxY+3.2-ymin)/self.resolution)
            self.width = math.ceil((maxX+3.2-xmin)/self.resolution)
            self.ymax = maxY+1.6
            self.xmax = maxX+1.6
            self.xmin = xmin-1.6
            self.ymin = ymin-1.6
            self.workspace_seen = True
            print(f"MAx,MIN: {self.xmax},{self.ymax}")

    

    def lineAlgorithm(self,x,y,xstop,ystop,grid_data):
        ''' Bresenham's line algorithm.'''
        xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
        ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
        xstopgrid = math.ceil((xstop-self.xmin)/(self.xmax-self.xmin)*self.width)
        ystopgrid = math.ceil((ystop-self.ymin)/(self.ymax-self.ymin)*self.height)

        if xgrid >=self.width:
            xgrid = self.width - 1
        if ygrid >=self.height:
            ygrid = self.height - 1
        
        if xstopgrid >=self.width:
            xstopgrid = self.width - 1
        if ystopgrid >=self.height:
            ystopgrid = self.height - 1

        delta_x = xstopgrid - xgrid
        delta_y = ystopgrid - ygrid

    # Line is vertical
        
        if delta_x == 0:  
            if xstopgrid < xgrid or ystopgrid < ygrid:
            # swap the values
                xtemp,ytemp = xgrid, ygrid
                xgrid, ygrid = xstopgrid, ystopgrid
                xstopgrid, ystopgrid = xtemp, ytemp

            for y in range(ygrid, ystopgrid + 1):
                #print("We are now in the vertical loop")
                x = xgrid
                grid_data[x][y] = 100
    
    # Line is horizontal 
       
        elif delta_y == 0:  
            if xstopgrid < xgrid or ystopgrid < ygrid:
            # swap the values
                xtemp,ytemp = xgrid, ygrid
                xgrid, ygrid = xstopgrid, ystopgrid
                xstopgrid, ystopgrid = xtemp, ytemp

            for x in range(xgrid, xstopgrid + 1):
                #print("We are now in the horizontal loop")
                y = ygrid
                grid_data[x][y] = 100
        
    # Line is diagonal or at an angle
        
        else:  
            is_steep = abs(delta_y) > abs(delta_x)
            error = 0 
            for step in range(min(abs(delta_x), abs(delta_y))):
            
            #Calculate grid coordinates of current cell
                if is_steep:
                    x = xgrid + step * sign(delta_x)
                    y = ygrid + round(step * delta_y / delta_x)
                else:
                    x = xgrid + round(step * delta_x / delta_y)
                    y = xgrid.y + step * sign(delta_y)

                grid_data[x,y] = 100
                error += abs(delta_y) if is_steep else abs(delta_x)

                for adj_x in range(x - 1, x + 2):
                    for adj_y in range(y - 1, y + 2):
                        if adj_x >= 0 and adj_x < grid_data.shape[0] and adj_y >= 0 and adj_y < grid_data.shape[1]:
                            if grid_data[adj_x, adj_y] != 100:
                                grid_data[adj_x, adj_y] = 100

                if error * 2 >= (abs(delta_y) if is_steep else abs(delta_x)):
                    if is_steep:
                        y += sign(delta_y)
                    else:
                        x += sign(delta_x)
                    error -= abs(delta_y) if is_steep else abs(delta_x)



        return grid_data

    def pubWorkspace(self):
        '''This function publishes the workspace as an occupancy grid.
         Have to reshape to 1D array, as the msg requires.'''
        
        
    # Create occupancy grid
        if not self.boundary and self.workspace_seen:

            grid_data = np.zeros((self.width, self.height))
            grid_data -= 1
            self.occGrid = np.zeros((self.width, self.height))
            self.grid_available = True
            self.boundary = True
            self.grid_data = grid_data
            for i in range(len(self.workspacePoints) -1):

                self.occGrid = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.occGrid)    
            self.occMask = self.occGrid!= 0
    
        #if self.grid_available:
        #        for i in range(len(self.workspacePoints) -1):
#
        #                self.grid_data = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.grid_data)
        
        self.grid_data[self.occMask]=100
        '''for i in range(len(self.cubePose)):
            try:
                self.grid_data[int(self.cubePose[i][0]),int(self.cubePose[i][1])] = 100
            except:
                print("Detection outside workspace")'''





        xRob = math.ceil((self.RobPose.pose.position.x-self.xmin)/(self.xmax-self.xmin)*self.width)
        yRob = math.ceil((self.RobPose.pose.position.y-self.ymin)/(self.ymax-self.ymin)*self.height)
#                 #seenGrid[xRob,yRob] = 1
        
        if self.init_ard_rob and self.workspace_seen and self.givenRobotPose:
            self.grid_data[(xRob-7):(xRob+8),(yRob-7):(yRob+8)] = 0
            self.init_ard_rob = False
        oldGrid = self.grid_data
        
        if self.prevGridExist:
            gridMask = oldGrid!=self.prevGrid
            self.prevGrid[gridMask] = oldGrid[gridMask]
        
        sigma = 2
        gaussian_grid = gaussian_filter(oldGrid.astype(float), sigma=sigma)

    
       
        
    # Normalize the values to range from 0 to 1
        gaussian_grid = gaussian_grid / np.max(gaussian_grid)

    # Set the mean value to a high value
        mean_value = 200
        gaussian_grid = gaussian_grid * mean_value + (1 - gaussian_grid) * 1.5

    # Cap the maximum value to 101
        #gaussian_grid = self.grid_data + gaussian_grid

          
        gaussian_grid = np.clip(gaussian_grid, -1, 100)
        gaussian_grid = np.nan_to_num(gaussian_grid, nan=0.0)

        if self.prevGridExist:
            self.prevGrid[gridMask] = gaussian_grid[gridMask]
            gaussian_grid= self.prevGrid

        self.prevGrid = gaussian_grid       # used to compare new and old grid
        self.pubGrid = gaussian_grid.reshape(-1,order='F').astype(int).tolist()

        occupancy_grid = OccupancyGrid()

        occupancy_grid.header.frame_id = "map" 
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.width
        occupancy_grid.info.height = self.height

        occupancy_grid.info.origin.position.x =  self.xmin
        occupancy_grid.info.origin.position.y =  self.ymin
        occupancy_grid.info.origin.position.z = 0   #self.marker.pose.position.z

        occupancy_grid.info.origin.orientation.x = 0    #self.marker.pose.orientation.x
        occupancy_grid.info.origin.orientation.y = 0    #self.marker.pose.orientation.y
        occupancy_grid.info.origin.orientation.z = 0    #self.marker.pose.orientation.z
        occupancy_grid.info.origin.orientation.w = 1    #self.marker.pose.orientation.w

        occupancy_grid.data = self.pubGrid

    # Publish occupancy grid
        

        self.pub.publish(occupancy_grid)
        
        if not self.prevGridExist:
            self.prevGridExist= True
                


def main():
    rospy.init_node('discreteWorkspace')
    workspace = Workspace()
    while not rospy.is_shutdown():
        if workspace.workspace_seen:
            workspace.pubWorkspace()
        workspace.rate.sleep()
    rospy.spin()









if __name__ == '__main__':
    main()



# ''#!/usr/bin/env python3
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import MarkerArray
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped,PoseStamped
# from sensor_msgs.msg import PointCloud2,LaserScan
# from open3d import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as o3drh
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import tf2_ros
# import numpy as np
# import math
# from math import atan2,cos,sin,fabs
# from numpy import sign
# import tf2_geometry_msgs
# import sys
# #from scipy.ndimage.morphology import binary_dilation
# from scipy.ndimage import binary_dilation, gaussian_filter
# import matplotlib.pyplot as plt
# # create class that discretizes the workspace 
# class Workspace:
#     def __init__(self):

#         self.buffer = tf2_ros.Buffer(rospy.Duration(10))
#         self.listener = tf2_ros.TransformListener(self.buffer)

#     # Publisher for the occupancy grid

#         self.pub = rospy.Publisher('/occupancyGridUpdate', OccupancyGrid, queue_size=10)

#     # Subscribers

#         rospy.Subscriber('/scan', LaserScan, self.laser_callback,queue_size=1)
#         rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.cov_cb)
#         rospy.Subscriber('/geofence',MarkerArray,self.marker_callback)
#         rospy.Subscriber('/marker2', MarkerArray, self.objectCallback)
#         rospy.Subscriber('/marker', MarkerArray, self.aruco_cb)

#     # Variables
#         self.resolution = 0.05
#         self.width = 0
#         self.height = 0
#         self.xmin = None
#         self.ymin = None

        
#         self.marker = None
#         self.grid_data = None
#         self.oldGrid = None
#         self.angleMapOdom = None
#         self.prevGrid = None
#         self.occGrid = None
#         self.occMask = None
        
#         self.addedSeenSpace = False
#         self.workspace_seen = False
#         self.boundary = False
#         self.ok2pubList = False
#         self.grid_available = False
#         self.givenRobotPose = False 
#         self.prevGridExist = False
        
#         self.objNameArray = []
#         self.workspacePoints = []
#         self.cubePose = []
#         self.cubePose = []
        
    
#         self.rate = rospy.Rate(5)
        



#     def aruco_cb(self, msg):
#         # get the position of the aruco marker
#         if not self.grid_available:
#             return
        
#         self.marker = msg.markers
#         for marker in self.marker:
#             if marker.id != 500 and marker.id != 0:
                
#                 x = marker.pose.position.x
#                 y = marker.pose.position.y
                
#                 xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#                 ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                
#                 self.cubePose.append([xgrid,ygrid])
        



#     def objectCallback(self,msg):
#         #WILL NOT HAVE TO HANDLE MUPLITIPLES OF SAME OBJECT!!  MarkerArray

#         #THIS WILL HAVE TO CHANGE, CORNELIA IS SENDING IN A MARKERARRAY WITH ALL OBJECTS
        
#         if not self.grid_available: # We have not initialized all parameters yet
#             return
        
#         self.cubePose = []

#         for marker in msg.markers:

#             try:
#                 #mapView = self.buffer.lookup_transform("map","camera_color_optical_frame",marker.header.stamp, rospy.Duration(0.5))
#                 #do_trans = tf2_geometry_msgs.do_transform_pose(marker, mapView)

#                 x = marker.pose.position.x#do_trans.pose.position.x
#                 y = marker.pose.position.y#do_trans.pose.position.y
                
#                 xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#                 ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

#                 #if [xgrid,ygrid] not in self.cubePose:# and marker.id not in self.objNameArray:
#                 #    self.objNameArray.append(marker.id)
#                 #    self.cubePose.append([xgrid,ygrid])

#                 self.cubePose.append([xgrid,ygrid])
#                 self.ok2pubList = True
                
#             except:

#                 print("No transform found")


#     def cov_cb(self,msg):
#         '''Gets angle and position from robot in base_link, have to convert to map.'''
        
#         self.givenRobotPose = True



#     def laser_callback(self,msg):
#         '''Think it works as a windshield wiper, scanning the range. Need initial angle'''
        
#     #Make sure self.width and heigh seen and defined!
#         if not self.grid_available or not self.givenRobotPose:
#             return
#         maxrange = 1.5 #meters
#         ranges = msg.ranges
        
#         dTheta = msg.angle_increment
#         angMin = msg.angle_min
#         rangeMin = msg.range_min
#         rangeMax = msg.range_max
        
        
#     # Looking up position of robot and orientation
#         try:
#             mapView = self.buffer.lookup_transform("map","camera_depth_frame",msg.header.stamp, rospy.Duration(0.5))
#             rot = [mapView.transform.rotation.x,mapView.transform.rotation.y,mapView.transform.rotation.z,mapView.transform.rotation.w]

#             (roll,pitch,yaw) = euler_from_quaternion(rot)
#             self.angleMapOdom = yaw

#             xRob = mapView.transform.translation.x   # self.robX # do_trans.pose.position.x
#             yRob = mapView.transform.translation.y   #self.robY # do_trans.pose.position.y

#             xgrid = math.ceil((xRob-self.xmin)/(self.xmax-self.xmin)*self.width)
#             ygrid = math.ceil((yRob-self.ymin)/(self.ymax-self.ymin)*self.height)
#             #gridrayStart
#             xRayStart = math.ceil((xRob+0.2-self.xmin)/(self.xmax-self.xmin)*self.width)
#             yRayStart = math.ceil((yRob+0.2-self.ymin)/(self.ymax-self.ymin)*self.height)

#             self.rangeMaxed = False
            
#             ranges=np.asarray(ranges)

#             for i in range(len(ranges)):
#                 if math.isnan(ranges[i]):
#                     continue

#                 if ranges[i] > rangeMin and ranges[i] < rangeMax:
#                     if ranges[i]> maxrange:
#                         self.rangeMaxed = True
#                         ranges[i] = maxrange

#                     x = xRob + ranges[i]*cos(angMin + (dTheta*i) + self.angleMapOdom)
#                     xRayStart = xRob + 0.3*cos(angMin + (dTheta*i) + self.angleMapOdom)
#                     xRay = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)

#                     y = yRob + ranges[i]*sin(angMin + (dTheta*i) + self.angleMapOdom)
#                     yRayStart = yRob + 0.3*sin(angMin + (dTheta*i) + self.angleMapOdom)
#                     yRay = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

#                     xRayStart = math.ceil((xRayStart-self.xmin)/(self.xmax-self.xmin)*self.width)
#                     yRayStart = math.ceil((yRayStart-self.ymin)/(self.ymax-self.ymin)*self.height)

#                     if not self.rangeMaxed and self.grid_data[xRay,yRay] != 101 :
#                         self.grid_data[xRay,yRay] = 100
                        
#                     elif self.rangeMaxed and self.grid_data[xRay,yRay] != 101:
#                         self.grid_data[xRay,yRay] = 0
#                         self.rangeMaxed = False

#                     #self.raytrace((xgrid,ygrid),(xRay,yRay))
#                     self.raytrace((xRayStart,yRayStart),(xRay,yRay))


#         except:
#             print("Error: ", sys.exc_info()[0])

#     def raytrace(self, start, end):
#         """Returns all cells in the grid map that has been traversed
#         from start to end, including start and excluding end.
#         start = (x, y) grid map index
#         end = (x, y) grid map index
#         """
#         (start_x, start_y) = start
#         (end_x, end_y) = end
#         x = start_x
#         y = start_y
#         (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
#         n = dx + dy
#         x_inc = 1
#         if end_x <= start_x:
#             x_inc = -1
#         y_inc = 1
#         if end_y <= start_y:
#             y_inc = -1
#         error = dx - dy
#         dx *= 2
#         dy *= 2

#         traversed = []
#         for i in range(0, int(n)):
#             traversed.append((int(x), int(y)))
#             #if self.grid_data[x,y] != 101:
#             #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
#             #if [x,y] not in self.cubePose and self.grid_data[x,y]<10:
#             if [x,y] not in self.cubePose:
            
#                 self.grid_data[x,y] = 0

#             if error > 0:
#                 x += x_inc
#                 error -= dy
#             else:
#                 if error == 0:
#                     traversed.append((int(x + x_inc), int(y)))
#                     #if self.grid_data[x+x_inc,y] != 101:
#                     #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
#                     #if [x,y] not in self.cubePose and self.grid_data[x,y]<10:
#                     if [x,y] not in self.cubePose:

#                         self.grid_data[x + x_inc,y] = 0
#                 y += y_inc
#                 error += dx
#         return traversed


#     def marker_callback(self,msg):
#         '''This function gets the height and width of the workspace'''
#     # If seen, no need to continue!
#         if self.workspace_seen:
#             return
#         if len(msg.markers) == 0:
#             return
        
#         self.marker=msg.markers[0]
    
#     # Look for the different coordianates of the marker
#         maxX = -np.inf
#         xmin = np.inf
#         maxY = -np.inf
#         ymin = np.inf
#         if msg.markers[0].ns == "workspace":
#             for point in msg.markers[0].points:
#                 x = point.x
#                 y = point.y
#                 self.workspacePoints.append([x,y])
#                 if x > maxX:
#                     maxX = x
#                 if y > maxY:
#                     maxY = y
#                 if x < xmin:
#                     xmin = x
#                 if y < ymin:
#                     ymin = y
          
#     # Save the height and width and other parameters of the workspace.
#             self.height = math.ceil((maxY+4-ymin)/self.resolution)
#             self.width = math.ceil((maxX+4-xmin)/self.resolution)
#             self.ymax = maxY+2
#             self.xmax = maxX+2
#             self.xmin = xmin-2
#             self.ymin = ymin-2
#             self.workspace_seen = True
#             print(f"MAx,MIN: {self.xmax},{self.ymax}")

    

#     def lineAlgorithm(self,x,y,xstop,ystop,grid_data):
#         ''' Bresenham's line algorithm.'''
#         xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#         ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
#         xstopgrid = math.ceil((xstop-self.xmin)/(self.xmax-self.xmin)*self.width)
#         ystopgrid = math.ceil((ystop-self.ymin)/(self.ymax-self.ymin)*self.height)

#         if xgrid >=self.width:
#             xgrid = self.width - 1
#         if ygrid >=self.height:
#             ygrid = self.height - 1
        
#         if xstopgrid >=self.width:
#             xstopgrid = self.width - 1
#         if ystopgrid >=self.height:
#             ystopgrid = self.height - 1

#         delta_x = xstopgrid - xgrid
#         delta_y = ystopgrid - ygrid

#     # Line is vertical
        
#         if delta_x == 0:  
#             if xstopgrid < xgrid or ystopgrid < ygrid:
#             # swap the values
#                 xtemp,ytemp = xgrid, ygrid
#                 xgrid, ygrid = xstopgrid, ystopgrid
#                 xstopgrid, ystopgrid = xtemp, ytemp

#             for y in range(ygrid, ystopgrid + 1):
#                 #print("We are now in the vertical loop")
#                 x = xgrid
#                 grid_data[x][y] = 100
    
#     # Line is horizontal 
       
#         elif delta_y == 0:  
#             if xstopgrid < xgrid or ystopgrid < ygrid:
#             # swap the values
#                 xtemp,ytemp = xgrid, ygrid
#                 xgrid, ygrid = xstopgrid, ystopgrid
#                 xstopgrid, ystopgrid = xtemp, ytemp

#             for x in range(xgrid, xstopgrid + 1):
#                 #print("We are now in the horizontal loop")
#                 y = ygrid
#                 grid_data[x][y] = 100
        
#     # Line is diagonal or at an angle
        
#         else:  
#             is_steep = abs(delta_y) > abs(delta_x)
#             error = 0 
#             for step in range(min(abs(delta_x), abs(delta_y))):
            
#             #Calculate grid coordinates of current cell
#                 if is_steep:
#                     x = xgrid + step * sign(delta_x)
#                     y = ygrid + round(step * delta_y / delta_x)
#                 else:
#                     x = xgrid + round(step * delta_x / delta_y)
#                     y = xgrid.y + step * sign(delta_y)

#                 grid_data[x,y] = 100
#                 error += abs(delta_y) if is_steep else abs(delta_x)

#                 for adj_x in range(x - 1, x + 2):
#                     for adj_y in range(y - 1, y + 2):
#                         if adj_x >= 0 and adj_x < grid_data.shape[0] and adj_y >= 0 and adj_y < grid_data.shape[1]:
#                             if grid_data[adj_x, adj_y] != 100:
#                                 grid_data[adj_x, adj_y] = 100

#                 if error * 2 >= (abs(delta_y) if is_steep else abs(delta_x)):
#                     if is_steep:
#                         y += sign(delta_y)
#                     else:
#                         x += sign(delta_x)
#                     error -= abs(delta_y) if is_steep else abs(delta_x)



#         return grid_data

#     def pubWorkspace(self):
#         '''This function publishes the workspace as an occupancy grid.
#          Have to reshape to 1D array, as the msg requires.'''
        
        
#     # Create occupancy grid
#         if not self.boundary and self.workspace_seen:

#             grid_data = np.zeros((self.width, self.height))
#             grid_data -= 1
#             self.occGrid = np.zeros((self.width, self.height))
#             self.grid_available = True
#             self.boundary = True
#             self.grid_data = grid_data
            
#             for i in range(len(self.workspacePoints) -1):

#                 self.occGrid = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.occGrid)    
#             self.occMask = self.occGrid!= 0
    
#         #if self.grid_available:
#         #        for i in range(len(self.workspacePoints) -1):
# #
#         #                self.grid_data = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.grid_data)
        
#         self.grid_data[self.occMask]=100
#         for i in range(len(self.cubePose)):
#             try:
#                 self.grid_data[int(self.cubePose[i][0]),int(self.cubePose[i][1])] = 100
#             except Exception as e:
                
#                 rospy.loginfo(f"ERROR THROWN!, {e}")
        

#         if self.givenRobotPose and not self.addedSeenSpace:
#             try:
#                 self.rate.sleep()
#                 self.rate.sleep()
#                 mapView = self.buffer.lookup_transform("map","odom",rospy.Time(0), rospy.Duration(3))
            
#                 x=mapView.transform.translation.x
#                 y=mapView.transform.translation.y
#                 xRob = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#                 yRob = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
#                 #seenGrid[xRob,yRob] = 1
#                 self.grid_data[(xRob-7):(xRob+8),(yRob-7):(yRob+8)] = 0
#                 self.addedSeenSpace = True
#             except Exception as e:
#                  rospy.logwarn(f"MAP NOT INIT!, Error: {e}")

        
        
        
        
        
        
        
        
        
        
#         oldGrid = self.grid_data
        
#         if self.prevGridExist:
#             gridMask = oldGrid!=self.prevGrid
#             self.prevGrid[gridMask] = oldGrid[gridMask]
        
#         sigma = 2
#         gaussian_grid = gaussian_filter(oldGrid.astype(float), sigma=sigma)

    
       
        
#     # Normalize the values to range from 0 to 1
#         gaussian_grid = gaussian_grid / np.max(gaussian_grid)

#     # Set the mean value to a high value
#         mean_value = 200
#         gaussian_grid = gaussian_grid * mean_value + (1 - gaussian_grid) * 1.5

#     # Cap the maximum value to 101
#         gaussian_grid= self.grid_data + gaussian_grid

          
#         gaussian_grid = np.clip(gaussian_grid, -1, 100)
#         gaussian_grid = np.nan_to_num(gaussian_grid, nan=0.0)

#         if self.prevGridExist:
#             self.prevGrid[gridMask] = gaussian_grid[gridMask]
#             gaussian_grid= self.prevGrid

#         self.prevGrid = gaussian_grid       # used to compare new and old grid
#         self.pubGrid = gaussian_grid.reshape(-1,order='F').astype(int).tolist()

#         occupancy_grid = OccupancyGrid()

#         occupancy_grid.header.frame_id = "map" 
#         occupancy_grid.header.stamp = rospy.Time.now()
#         occupancy_grid.info.resolution = self.resolution
#         occupancy_grid.info.width = self.width
#         occupancy_grid.info.height = self.height

#         occupancy_grid.info.origin.position.x =  self.xmin
#         occupancy_grid.info.origin.position.y =  self.ymin
#         occupancy_grid.info.origin.position.z = 0   #self.marker.pose.position.z

#         occupancy_grid.info.origin.orientation.x = 0    #self.marker.pose.orientation.x
#         occupancy_grid.info.origin.orientation.y = 0    #self.marker.pose.orientation.y
#         occupancy_grid.info.origin.orientation.z = 0    #self.marker.pose.orientation.z
#         occupancy_grid.info.origin.orientation.w = 1    #self.marker.pose.orientation.w

#         occupancy_grid.data = self.pubGrid

#     # Publish occupancy grid
        

#         self.pub.publish(occupancy_grid)
        
#         if not self.prevGridExist:
#             self.prevGridExist= True
                


# def main():
#     rospy.init_node('discreteWorkspace')
#     workspace = Workspace()
#     while not rospy.is_shutdown():
#         if workspace.workspace_seen:
#             workspace.pubWorkspace()
#         workspace.rate.sleep()
#     rospy.spin()









# if __name__ == '__main__':
#     main()

''


# #!/usr/bin/env python3
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import MarkerArray
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped,PoseStamped
# from sensor_msgs.msg import PointCloud2,LaserScan
# from open3d import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as o3drh
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import tf2_ros
# import numpy as np
# import math
# from math import atan2,cos,sin,fabs
# from numpy import sign
# import tf2_geometry_msgs
# import sys
# #from scipy.ndimage.morphology import binary_dilation
# from scipy.ndimage import binary_dilation, gaussian_filter
# import matplotlib.pyplot as plt
# import copy
# # create class that discretizes the workspace 
# class Workspace:
#     def __init__(self):

#         self.buffer = tf2_ros.Buffer(rospy.Duration(10))
#         self.listener = tf2_ros.TransformListener(self.buffer)

#     # Publisher for the occupancy grid

#         self.pub = rospy.Publisher('/occupancyGridUpdate', OccupancyGrid, queue_size=1)

#     # Subscribers

#         rospy.Subscriber('/scan', LaserScan, self.laser_callback,queue_size=1)
#         rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.cov_cb)
#         rospy.Subscriber('/geofence',MarkerArray,self.marker_callback)
#         rospy.Subscriber('/marker2', MarkerArray, self.objectCallback)
#         rospy.Subscriber('/marker', MarkerArray, self.aruco_cb)

#     # Variables
#         self.resolution = 0.05
#         self.width = 0
#         self.height = 0
#         self.xmin = None
#         self.ymin = None

        
#         self.marker = None
#         self.grid_data = None
#         self.oldGrid = None
#         self.angleMapOdom = None
#         self.prevGrid = None
#         self.occGrid = None
#         self.occMask = None
        
#         self.workspace_seen = False
#         self.boundary = False
#         self.ok2pubList = False
#         self.grid_available = False
#         self.givenRobotPose = False 
#         self.prevGridExist = False
        
#         self.objNameArray = []
#         self.workspacePoints = []
#         self.cubePose = []
#         self.cubePose = []
    
#         self.rate = rospy.Rate(2)
        



#     def aruco_cb(self, msg):
#         # get the position of the aruco marker
#         if not self.grid_available:
#             return
        
#         self.marker = msg.markers
#         for marker in self.marker:
#             if marker.id != 500 and marker.id != 0:
                
#                 x = marker.pose.position.x
#                 y = marker.pose.position.y
                
#                 xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#                 ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                
#                 self.cubePose.append([xgrid,ygrid])
        



#     def objectCallback(self,msg):
#         #WILL NOT HAVE TO HANDLE MUPLITIPLES OF SAME OBJECT!!  MarkerArray

#         #THIS WILL HAVE TO CHANGE, CORNELIA IS SENDING IN A MARKERARRAY WITH ALL OBJECTS
        
#         if not self.grid_available: # We have not initialized all parameters yet
#             return
        
#         self.cubePose = []

#         for marker in msg.markers:

#             try:
#                 #mapView = self.buffer.lookup_transform("map","camera_color_optical_frame",marker.header.stamp, rospy.Duration(0.5))
#                 #do_trans = tf2_geometry_msgs.do_transform_pose(marker, mapView)

#                 x = marker.pose.position.x#do_trans.pose.position.x
#                 y = marker.pose.position.y#do_trans.pose.position.y
                
#                 xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#                 ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

#                 #if [xgrid,ygrid] not in self.cubePose:# and marker.id not in self.objNameArray:
#                 #    self.objNameArray.append(marker.id)
#                 #    self.cubePose.append([xgrid,ygrid])

#                 self.cubePose.append([xgrid,ygrid])
#                 self.ok2pubList = True
                
#             except:

#                 print("No transform found")


#     def cov_cb(self,msg):
#         '''Gets angle and position from robot in base_link, have to convert to map.'''
        
#         self.givenRobotPose = True



#     def laser_callback(self,msg):
#         '''Think it works as a windshield wiper, scanning the range. Need initial angle'''
        
#     #Make sure self.width and heigh seen and defined!
#         if not self.grid_available or not self.givenRobotPose:
#             return
#         maxrange = 1.5 #meters
#         ranges = msg.ranges
        
#         dTheta = msg.angle_increment
#         angMin = msg.angle_min
#         rangeMin = msg.range_min
#         rangeMax = msg.range_max
        
        
#     # Looking up position of robot and orientation
#         try:
#             mapView = self.buffer.lookup_transform("map","camera_depth_frame",msg.header.stamp, rospy.Duration(0.5))
#             rot = [mapView.transform.rotation.x,mapView.transform.rotation.y,mapView.transform.rotation.z,mapView.transform.rotation.w]

#             (roll,pitch,yaw) = euler_from_quaternion(rot)
#             self.angleMapOdom = yaw

#             xRob = mapView.transform.translation.x   # self.robX # do_trans.pose.position.x
#             yRob = mapView.transform.translation.y   #self.robY # do_trans.pose.position.y

#             xgrid = math.ceil((xRob-self.xmin)/(self.xmax-self.xmin)*self.width)
#             ygrid = math.ceil((yRob-self.ymin)/(self.ymax-self.ymin)*self.height)
#             #gridrayStart
#             xRayStart = math.ceil((xRob+0.2-self.xmin)/(self.xmax-self.xmin)*self.width)
#             yRayStart = math.ceil((yRob+0.2-self.ymin)/(self.ymax-self.ymin)*self.height)

#             self.rangeMaxed = False
            
#             ranges=np.asarray(ranges)

#             for i in range(len(ranges)):
#                 if math.isnan(ranges[i]):
#                     continue

#                 if ranges[i] > rangeMin and ranges[i] < rangeMax:
#                     if ranges[i]> maxrange:
#                         self.rangeMaxed = True
#                         ranges[i] = maxrange

#                     x = xRob + ranges[i]*cos(angMin + (dTheta*i) + self.angleMapOdom)
#                     xRayStart = xRob + 0.3*cos(angMin + (dTheta*i) + self.angleMapOdom)
#                     xRay = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)

#                     y = yRob + ranges[i]*sin(angMin + (dTheta*i) + self.angleMapOdom)
#                     yRayStart = yRob + 0.3*sin(angMin + (dTheta*i) + self.angleMapOdom)
#                     yRay = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)

#                     xRayStart = math.ceil((xRayStart-self.xmin)/(self.xmax-self.xmin)*self.width)
#                     yRayStart = math.ceil((yRayStart-self.ymin)/(self.ymax-self.ymin)*self.height)

#                     if not self.rangeMaxed and self.grid_data[xRay,yRay] != 101 :
#                         self.grid_data[xRay,yRay] = 100
                        
#                     elif self.rangeMaxed and self.grid_data[xRay,yRay] != 101:
#                         self.grid_data[xRay,yRay] = 0
#                         self.rangeMaxed = False

#                     #self.raytrace((xgrid,ygrid),(xRay,yRay))
#                     self.raytrace((xRayStart,yRayStart),(xRay,yRay))


#         except:
#             print("Error: ", sys.exc_info()[0])

#     def raytrace(self, start, end):
#         """Returns all cells in the grid map that has been traversed
#         from start to end, including start and excluding end.
#         start = (x, y) grid map index
#         end = (x, y) grid map index
#         """
#         (start_x, start_y) = start
#         (end_x, end_y) = end
#         x = start_x
#         y = start_y
#         (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
#         n = dx + dy
#         x_inc = 1
#         if end_x <= start_x:
#             x_inc = -1
#         y_inc = 1
#         if end_y <= start_y:
#             y_inc = -1
#         error = dx - dy
#         dx *= 2
#         dy *= 2

#         traversed = []
#         for i in range(0, int(n)):
#             traversed.append((int(x), int(y)))
#             #if self.grid_data[x,y] != 101:
#             #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
#             #if [x,y] not in self.cubePose and self.grid_data[x,y]<10:
            
#             if [x,y] not in self.cubePose:
#                 self.grid_data[x,y] = 0

#             if error > 0:
#                 x += x_inc
#                 error -= dy
#             else:
#                 if error == 0:
#                     traversed.append((int(x + x_inc), int(y)))
#                     #if self.grid_data[x+x_inc,y] != 101:
#                     #if self.grid_data[x,y] < 1 and [x,y] not in self.cubePose:
#                     #if [x,y] not in self.cubePose and self.grid_data[x,y]<10:
#                     if [x,y] not in self.cubePose:

#                         self.grid_data[x + x_inc,y] = 0
#                 y += y_inc
#                 error += dx
#         return traversed


#     def marker_callback(self,msg):
#         '''This function gets the height and width of the workspace'''
#     # If seen, no need to continue!
#         if self.workspace_seen:
#             return
#         if len(msg.markers) == 0:
#             return
        
#         self.marker=msg.markers[0]
    
#     # Look for the different coordianates of the marker
#         maxX = -np.inf
#         xmin = np.inf
#         maxY = -np.inf
#         ymin = np.inf
#         if msg.markers[0].ns == "workspace":
#             for point in msg.markers[0].points:
#                 x = point.x
#                 y = point.y
#                 self.workspacePoints.append([x,y])
#                 if x > maxX:
#                     maxX = x
#                 if y > maxY:
#                     maxY = y
#                 if x < xmin:
#                     xmin = x
#                 if y < ymin:
#                     ymin = y
          
#     # Save the height and width and other parameters of the workspace.
#             '''self.height = math.ceil((maxY+4-ymin)/self.resolution)
#             self.width = math.ceil((maxX+4-xmin)/self.resolution)
#             self.ymax = maxY+2

#             self.xmax = maxX+2
#             self.xmin = xmin-2
#             self.ymin = ymin-2'''
#             self.height = math.ceil((maxY+3.2-ymin)/self.resolution)
#             self.width = math.ceil((maxX+3.2-xmin)/self.resolution)
#             self.ymax = maxY+1.6
#             self.xmax = maxX+1.6
#             self.xmin = xmin-1.6
#             self.ymin = ymin-1.6
#             self.workspace_seen = True
#             print(f"MAx,MIN: {self.xmax},{self.ymax}")

    

#     def lineAlgorithm(self,x,y,xstop,ystop,grid_data):
#         ''' Bresenham's line algorithm.'''
#         xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
#         ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
#         xstopgrid = math.ceil((xstop-self.xmin)/(self.xmax-self.xmin)*self.width)
#         ystopgrid = math.ceil((ystop-self.ymin)/(self.ymax-self.ymin)*self.height)

#         if xgrid >=self.width:
#             xgrid = self.width - 1
#         if ygrid >=self.height:
#             ygrid = self.height - 1
        
#         if xstopgrid >=self.width:
#             xstopgrid = self.width - 1
#         if ystopgrid >=self.height:
#             ystopgrid = self.height - 1

#         delta_x = xstopgrid - xgrid
#         delta_y = ystopgrid - ygrid

#     # Line is vertical
        
#         if delta_x == 0:  
#             if xstopgrid < xgrid or ystopgrid < ygrid:
#             # swap the values
#                 xtemp,ytemp = xgrid, ygrid
#                 xgrid, ygrid = xstopgrid, ystopgrid
#                 xstopgrid, ystopgrid = xtemp, ytemp

#             for y in range(ygrid, ystopgrid + 1):
#                 #print("We are now in the vertical loop")
#                 x = xgrid
#                 grid_data[x][y] = 100
    
#     # Line is horizontal 
       
#         elif delta_y == 0:  
#             if xstopgrid < xgrid or ystopgrid < ygrid:
#             # swap the values
#                 xtemp,ytemp = xgrid, ygrid
#                 xgrid, ygrid = xstopgrid, ystopgrid
#                 xstopgrid, ystopgrid = xtemp, ytemp

#             for x in range(xgrid, xstopgrid + 1):
#                 #print("We are now in the horizontal loop")
#                 y = ygrid
#                 grid_data[x][y] = 100
        
#     # Line is diagonal or at an angle
        
#         else:  
#             is_steep = abs(delta_y) > abs(delta_x)
#             error = 0 
#             for step in range(min(abs(delta_x), abs(delta_y))):
            
#             #Calculate grid coordinates of current cell
#                 if is_steep:
#                     x = xgrid + step * sign(delta_x)
#                     y = ygrid + round(step * delta_y / delta_x)
#                 else:
#                     x = xgrid + round(step * delta_x / delta_y)
#                     y = xgrid.y + step * sign(delta_y)

#                 grid_data[x,y] = 100
#                 error += abs(delta_y) if is_steep else abs(delta_x)

#                 for adj_x in range(x - 1, x + 2):
#                     for adj_y in range(y - 1, y + 2):
#                         if adj_x >= 0 and adj_x < grid_data.shape[0] and adj_y >= 0 and adj_y < grid_data.shape[1]:
#                             if grid_data[adj_x, adj_y] != 100:
#                                 grid_data[adj_x, adj_y] = 100

#                 if error * 2 >= (abs(delta_y) if is_steep else abs(delta_x)):
#                     if is_steep:
#                         y += sign(delta_y)
#                     else:
#                         x += sign(delta_x)
#                     error -= abs(delta_y) if is_steep else abs(delta_x)



#         return grid_data

#     def pubWorkspace(self):
#         '''This function publishes the workspace as an occupancy grid.
#          Have to reshape to 1D array, as the msg requires.'''
        
        
#     # Create occupancy grid
#         if not self.boundary and self.workspace_seen:

#             grid_data = np.zeros((self.width, self.height))
#             grid_data -= 1
#             self.occGrid = np.zeros((self.width, self.height))
#             self.grid_available = True
#             self.boundary = True
#             self.grid_data = grid_data
#             for i in range(len(self.workspacePoints) -1):

#                         self.occGrid = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.occGrid)    
#             self.occMask = self.occGrid!= 0
    
#         #if self.grid_available:
#         #        for i in range(len(self.workspacePoints) -1):
# #
#         #                self.grid_data = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.grid_data)
        
#         self.grid_data[self.occMask]=100
#         cubepose = copy.deepcopy(self.cubePose)
#         for i in range(len(cubepose)):
            
#             try:
#                 self.grid_data[int(cubepose[i][0]),int(cubepose[i][1])] = 100
#             except:
#                 print(f"Error causing position")   
#         oldGrid = self.grid_data
        
#         if self.prevGridExist:
#             gridMask = oldGrid!=self.prevGrid
#             self.prevGrid[gridMask] = oldGrid[gridMask]
        
#         sigma = 2
#         gaussian_grid = gaussian_filter(oldGrid.astype(float), sigma=sigma)

    
       
        
#     # Normalize the values to range from 0 to 1
#         gaussian_grid = gaussian_grid / np.max(gaussian_grid)

#     # Set the mean value to a high value
#         mean_value = 200
#         #gaussian_grid = gaussian_grid * mean_value + (1 - gaussian_grid) * 1.5
#         gaussian_grid = gaussian_grid * mean_value + (1 - gaussian_grid) * 1.5


#     # Cap the maximum value to 101
#         gaussian_grid= self.grid_data + gaussian_grid

          
#         gaussian_grid = np.clip(gaussian_grid, -1, 100)
#         gaussian_grid = np.nan_to_num(gaussian_grid, nan=0.0)

#         if self.prevGridExist:
#             self.prevGrid[gridMask] = gaussian_grid[gridMask]
#             gaussian_grid= self.prevGrid

#         self.prevGrid = gaussian_grid       # used to compare new and old grid
#         self.pubGrid = gaussian_grid.reshape(-1,order='F').astype(int).tolist()

#         occupancy_grid = OccupancyGrid()

#         occupancy_grid.header.frame_id = "map" 
#         occupancy_grid.header.stamp = rospy.Time.now()
#         occupancy_grid.info.resolution = self.resolution
#         occupancy_grid.info.width = self.width
#         occupancy_grid.info.height = self.height

#         occupancy_grid.info.origin.position.x =  self.xmin
#         occupancy_grid.info.origin.position.y =  self.ymin
#         occupancy_grid.info.origin.position.z = 0   #self.marker.pose.position.z

#         occupancy_grid.info.origin.orientation.x = 0    #self.marker.pose.orientation.x
#         occupancy_grid.info.origin.orientation.y = 0    #self.marker.pose.orientation.y
#         occupancy_grid.info.origin.orientation.z = 0    #self.marker.pose.orientation.z
#         occupancy_grid.info.origin.orientation.w = 1    #self.marker.pose.orientation.w

#         occupancy_grid.data = self.pubGrid

#     # Publish occupancy grid
        

#         self.pub.publish(occupancy_grid)
        
#         if not self.prevGridExist:
#             self.prevGridExist= True
                


# def main():
#     rospy.init_node('discreteWorkspace')
#     workspace = Workspace()
#     while not rospy.is_shutdown():
#         if workspace.workspace_seen:
#             workspace.pubWorkspace()
#         workspace.rate.sleep()
#     rospy.spin()









# if __name__ == '__main__':
#     main()


