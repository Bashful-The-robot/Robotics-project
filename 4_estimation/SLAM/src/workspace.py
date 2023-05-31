#!/usr/bin/env python3
import rospy
import csv
#from aruco_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point,Pose,Quaternion
from visualization_msgs.msg import MarkerArray, Marker
#read in the tsv file example_workspace.csv 


# define a class that creates a publisher and a subscriber
class Workspace:
    def __init__(self):
        '''Initialize the node'''
        # create a publisher
        self.rate = rospy.Rate(1)
        self.marker_pub = rospy.Publisher('/geofence', MarkerArray, queue_size=10)
        self.marker = Marker()       
        
        self.workspace = self.readCSV()
        self.workspace = self.workspace[1:]
        self.intWorkspace = self.makefloat(self.workspace)
        #self.pubWorkspace()
        self.pubLines()

    def makefloat(self, list):
        for i in range(len(list)):
            list[i] = [float(list[i][0]), float(list[i][1])]
        return list


    def readCSV(self):
        #read in the tsv file example_workspace.csv

        with open('/home/robot/BASHFUL_WS/src/4_estimation/SLAM/src/test.tsv', 'r') as f: #dd2419_ws/src/robot/SLAM/src
        #with open('/home/robot/dd2419_ws/src/robot/SLAM/src/example_workspace.tsv', 'r') as f: #dd2419_ws/src/robot/SLAM/src
        
            reader = csv.reader(f, delimiter='\t')
            workspace = list(reader)
            
        return workspace

    def pubLines(self):
        self.marker_array = MarkerArray()

        line_color = ColorRGBA()       # a nice color for my line (royalblue)
        line_color.r = 0.254902
        line_color.g = 0.411765
        line_color.b = 0.882353
        line_color.a = 1.0

        pointsArray = []
        for i in range(len(self.workspace)):
            pointsArray.append(Point())
#        p1 = Point()
#        p2 = Point()
#        p3 = Point()
#        p4 = Point()
        
#       pointsArray = [p1, p2, p3, p4]
        for i in range(len(pointsArray)):
            pointsArray[i].x = self.intWorkspace[i][0]
            pointsArray[i].y = self.intWorkspace[i][1]
            pointsArray[i].z = 0


        line_marker = Marker()
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "workspace"
        line_marker.header.frame_id = "map"
        line_marker.type =Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color = line_color
        for i in range(len(pointsArray)):
            line_marker.points.append(pointsArray[i])
        line_marker.points.append(pointsArray[0])# This is for adding the last point to the first point

        
        self.marker_array.markers.append(line_marker)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.marker_array)
            self.rate.sleep()
            #rospy.loginfo("Published workspace")
        


   
def main():
    # initialize the node
    rospy.init_node('workspace')
    # create an object of the class
    workspace = Workspace()
    # keep the node running
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("node terminated.")
if __name__ == '__main__':
    main()