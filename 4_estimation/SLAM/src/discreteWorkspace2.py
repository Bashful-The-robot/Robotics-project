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
#from math import abs

# create class that discretizes the workspace 
class Workspace:
    def __init__(self):
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pubMarkerArray = rospy.Publisher('/ObjectList', MarkerArray, queue_size=10)
        self.pub = rospy.Publisher('/occupancyGridUpdate', OccupancyGrid, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback,queue_size=1)
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.cov_cb)
        rospy.Subscriber('/marker',MarkerArray,self.marker_callback)
        rospy.Subscriber('/pospub', PointStamped, self.objectCallback)
        rospy.Subscriber('/marker', MarkerArray, self.aruco_cb)
        self.resolution = 0.05
        self.width = 0
        self.rate = rospy.Rate(10)
        self.height = 0
        self.workspace_seen = False
        self.marker = None
        self.xmin = None
        self.ymin = None
        self.boundary = False
        self.marker_array = MarkerArray()
        self.objectArray = []
        self.objNameArray = []
        self.objPose = []
        self.workspacePoints = []
        self.grid_data = None
        self.oldGrid = None
        self.ok2pubList = False
        self.grid_available = False
        self.angleMapOdom = None
        self.poseRobot = None
        self.givenRobotPose = False 
        self.cubePose = []
        self.RobOrientation = None
        self.RobPose = None
        self.Robmsg = None
        self.lock = False
        self.robX = None
        self.robY = None
    def aruco_cb(self, msg):
        # get the position of the aruco marker
        if not self.grid_available:
            return
        
        self.marker = msg.markers
        for marker in self.marker:
            if marker.id != 500:
                x = marker.pose.position.x
                y = marker.pose.position.y
                xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
                ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                self.cubePose.append([xgrid,ygrid])
        
        



    def objectCallback(self,msg):
        #rospy.loginfo(f"Object seen at: {msg}")


        # THIS DOES NOT HANDLE UPDATES TO THE OBJECTS POSITION DUE TO COVARIANCE!!!
        if not self.grid_available: # We have not initialized all parameters yet
            return
        
        if msg.header.frame_id not in self.objNameArray:
            self.objectArray.append(msg)
            self.objNameArray.append(msg.header.frame_id)
            # call draw function
            #self.draw(msg.point)
            x = msg.point.x
            y = msg.point.y
            xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
            ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
            # reshape the grid to a 2D array
            self.cubePose.append([xgrid,ygrid])
            
            
            #rospy.loginfo(f"Object array len: {self.objectArray}")
            self.marker = Marker()
        
            self.marker.header.frame_id = msg.header.frame_id
            self.marker.header.stamp = msg.header.stamp
            self.marker.points = [msg.point]
            self.marker_array.markers.append(self.marker)
            self.ok2pubList = True




    def cov_cb(self,msg):
        '''Gets angle and position from robot in base_link, have to convert to map.'''
        self.Robmsg = PoseStamped()
        self.Robmsg.pose.position = msg.pose.pose.position
        self.Robmsg.pose.orientation = msg.pose.pose.orientation
        #mapView = self.buffer.lookup_transform("map","base_link",msg.header.stamp, rospy.Duration(0.5))
        #rot = [mapView.transform.rotation.x,mapView.transform.rotation.y,mapView.transform.rotation.z,mapView.transform.rotation.w]
        #(roll,pitch,yaw) = euler_from_quaternion(rot)
        #self.angleMapOdom = yaw
        #self.robX= mapView.transform.translation.x#do_trans.pose.position.x
        #self.robY= mapView.transform.translation.y#do_trans.pose.position.y

        #mapView = self.buffer.lookup_transform("map","odom",msg.header.stamp, rospy.Duration(3.0))
        #self.poseRobot = mapView.transform.translation
        self.givenRobotPose = True



    def laser_callback(self,msg):
        '''Think it works as a windshield wiper, scanning the range. Need initial angle'''
        if self.lock:
            return
        #self.lock = True
        if not self.grid_available or not self.givenRobotPose:
            #Make sure self.width and heigh seen and defined!
            return
        maxrange = 1.5#1.5
        ranges = msg.ranges
        
        dTheta = msg.angle_increment
        angMin = msg.angle_min
        rangeMin = msg.range_min
        rangeMax = msg.range_max
        
        start = rospy.Time.now()
        
        # Looking up position of robot and orientation
        try:
            mapView = self.buffer.lookup_transform("map","base_link",msg.header.stamp, rospy.Duration(0.5))
            rot = [mapView.transform.rotation.x,mapView.transform.rotation.y,mapView.transform.rotation.z,mapView.transform.rotation.w]
            (roll,pitch,yaw) = euler_from_quaternion(rot)
            self.angleMapOdom = yaw
            xRob =mapView.transform.translation.x# self.robX # do_trans.pose.position.x
            yRob =mapView.transform.translation.y#self.robY # do_trans.pose.position.y
            xgrid = math.ceil((xRob-self.xmin)/(self.xmax-self.xmin)*self.width)
            ygrid = math.ceil((yRob-self.ymin)/(self.ymax-self.ymin)*self.height)
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
                    xRay = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)

                    y = yRob + ranges[i]*sin(angMin + (dTheta*i) + self.angleMapOdom)
                    yRay = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                    if not self.rangeMaxed and self.grid_data[xRay,yRay] != 101 :
                        self.grid_data[xRay,yRay] = 100
                    elif self.rangeMaxed and self.grid_data[xRay,yRay] != 101:

                        self.grid_data[xRay,yRay] = 0
                        self.rangeMaxed = False
                    self.raytrace((xgrid,ygrid),(xRay,yRay))
            end=rospy.Time.now()
            #print(f"Time:{(end-start).to_sec()}")
            self.lock=False

        except:
            print("There is a problem with the lookup, hopefully it is not forever!")
    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        ss = rospy.Time.now()
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
            if self.grid_data[x,y] != 101:
                self.grid_data[x,y] = 0

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                    if self.grid_data[x+x_inc,y] != 101:
                        self.grid_data[x + x_inc,y] = 0
                y += y_inc
                error += dx
        ee=rospy.Time.now()
        #print(f"raytime: {(ee-ss).to_sec()}")
        return traversed


    def drawGaussian(self,oldgrid):
        '''This function draws the Gaussian belonging to the object in the occupancy grid'''
        grid = oldgrid
        try:

            for obj in self.objectArray:
                x = obj.point.x
                y = obj.point.y
                xgrid = math.ceil((x-self.xmin)/(self.xmax-self.xmin)*self.width)
                ygrid = math.ceil((y-self.ymin)/(self.ymax-self.ymin)*self.height)
                # reshape the grid to a 2D array
                grid[xgrid,ygrid] = 100
                # draw the gaussian

                #grid_data[xgrid,ygrid] = 100
            return grid
        
        except: # This should only occure if there are no objects in the workspace!
            return grid



    def marker_callback(self,msg):
        '''This function gets the height and width of the workspace'''
        if self.workspace_seen:# If seen, no need to continue!
            return
        if len(msg.markers) == 0:
            return
        
        self.marker=msg.markers[0]
        # look for the different coordianates of the marker
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
          
          
            self.height = math.ceil((maxY+4-ymin)/self.resolution)
            self.width = math.ceil((maxX+4-xmin)/self.resolution)
            self.ymax = maxY+2
            self.xmax = maxX+2
            self.xmin = xmin-2
            self.ymin = ymin-2
            self.workspace_seen = True

    

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
        
        #print(f"starting point is {xgrid},{ygrid} and ending point is {xstopgrid},{ystopgrid}")
        if delta_x == 0:  # Line is vertical
            if xstopgrid < xgrid or ystopgrid < ygrid:
            # swap the values
                xtemp,ytemp = xgrid, ygrid
                xgrid, ygrid = xstopgrid, ystopgrid
                xstopgrid, ystopgrid = xtemp, ytemp

            for y in range(ygrid, ystopgrid + 1):
                #print("We are now in the vertical loop")
                x = xgrid
                grid_data[x][y] = 101
        elif delta_y == 0:  # Line is horizontal
            if xstopgrid < xgrid or ystopgrid < ygrid:
            # swap the values
                xtemp,ytemp = xgrid, ygrid
                xgrid, ygrid = xstopgrid, ystopgrid
                xstopgrid, ystopgrid = xtemp, ytemp

            for x in range(xgrid, xstopgrid + 1):
                #print("We are now in the horizontal loop")
                y = ygrid
                grid_data[x][y] = 101
        else:  # Line is diagonal or at an angle
            is_steep = abs(delta_y) > abs(delta_x)
            #print(f"is steep is {is_steep}")
            error = 0 
            #print(f"we in here")
            #print(f" min is {min(abs(delta_x), abs(delta_y))}")
            for step in range(min(abs(delta_x), abs(delta_y))):
            
                #Calculate grid coordinates of current cell
                #print(f"step is {step}")
                if is_steep:
                    x = xgrid + step * sign(delta_x)
                    y = ygrid + round(step * delta_y / delta_x)
                else:
                    x = xgrid + round(step * delta_x / delta_y)
                    y = xgrid.y + step * sign(delta_y)
                # Update error term
                # correct the errors in the code below
                #print(f"x,y are {x},{y} ")
                grid_data[x,y] = 101
                error += abs(delta_y) if is_steep else abs(delta_x)
                if error * 2 >= (abs(delta_y) if is_steep else abs(delta_x)):
                    if is_steep:
                        y += sign(delta_y)
                    else:
                        x += sign(delta_x)
                    error -= abs(delta_y) if is_steep else abs(delta_x)



        return grid_data

    def pubWorkspace(self):
        '''This function publishes the workspace as an occupancy grid'''
        
        ss = rospy.Time.now()
        
        # create occupancy grid
        if not self.boundary and self.workspace_seen:
            grid_data = np.zeros((self.width, self.height))
            grid_data -= 1
            self.grid_available = True
            self.boundary = True
            self.oldGrid = grid_data            # THIS IS USED TO Save the old grid so that we draw on the base of the old grid each time.
            self.grid_data = grid_data    # reshape to 1D array, as the msg requires.
            if self.grid_available:
                for i in range(len(self.workspacePoints) -1):

                        self.grid_data = self.lineAlgorithm(self.workspacePoints[i][0],self.workspacePoints[i][1],self.workspacePoints[i+1][0],self.workspacePoints[i+1][1],self.grid_data)


        if self.grid_available:
            self.grid_data = self.drawGaussian(self.oldGrid)   #Draw gaussian
        

        for i in range(len(self.cubePose)):
            self.grid_data[int(self.cubePose[i][0]),int(self.cubePose[i][1])] = 50
       
        self.pubGrid = self.grid_data.reshape(-1,order='F').astype(int)
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
        # publish occupancy grid
        if self.ok2pubList:
            self.pubMarkerArray.publish(self.marker_array)
        self.pub.publish(occupancy_grid)    
        ee = rospy.Time.now()
        #print(f"This is the pub: {(ee-ss).to_sec()}")
                


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
