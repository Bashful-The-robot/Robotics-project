#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Path
from robp_msgs.msg import flags

#########################################################
# Pure Pursuit Controller 
#     Determine the current location of the robot 
#     Find the path point closest to the robot
#     Find the target point 
#     Transform the target point to the robot coordinates
#     Calculate the curvature and the necessary twist 
#########################################################

class PurePursuitController:

    def __init__(self):
        # Define some constants
        self.look_ahead_distance = 0.15  #meters
        self.target_velocity = 0.05 #meters/second
        self.robot_pose = PoseStamped()
        self.path = Path()
        self.closest_point = [None,None]
        self.look_ahead_point = None
        self.action = False

        # Create Subscribers and publishers
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/path',Path, self.path_callback,queue_size=1)
        rospy.Subscriber('/system_flags',flags,self.flags_callback)

        self.twist_pub = rospy.Publisher('/motor_controller/twist', Twist, queue_size=1)

        # Define the transform buffer and listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def flags_callback(self,msg):
        self.action = msg.path_control

    def pose_callback(self, msg):
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0))
            self.robot_pose = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform)
            print("The robot position is ", self.robot_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass
            #rospy.logwarn("Failed to transform pose: {e}")
            #rospy.logwarn("Robot pose: {}".format(self.robot_pose))
        
    def path_callback(self, msg):
        self.path = msg
        print()

    def get_closest_point(self):
        
        if len(self.path.poses) == 0 or self.robot_pose is None:
            #rospy.logwarn("Path or robot pose not received!")
            return
        
        min_dist= math.inf 
        for i, waypoint in enumerate(self.path.poses):
            dist = math.sqrt((waypoint.pose.position.x - self.robot_pose.pose.position.x)**2 + 
                             (waypoint.pose.position.y - self.robot_pose.pose.position.y)**2)
            
            if dist < min_dist:
                min_dist = dist
                self.closest_point = [i,waypoint]
        #print("The closest point is:",self.closest_point)
    
    
    def get_lookahead_point(self):

        if len(self.path.poses) == 0 or self.robot_pose is None or self.closest_point is None:
            #rospy.logwarn("Path, robot pose, or closest point not received!")
            return
        
        start_index = self.closest_point[0]
        #print(start_index)
        min_error = math.inf
        dist_to_last = math.sqrt((self.path.poses[-1].pose.position.x - self.robot_pose.pose.position.x)**2 + 
                              (self.path.poses[-1].pose.position.y - self.robot_pose.pose.position.y)**2)
       
        if dist_to_last < self.look_ahead_distance:
            self.look_ahead_point = self.path.poses[-1]
            return
        
        
        for waypoint in self.path.poses[start_index:]:
            dist = math.sqrt((waypoint.pose.position.x - self.robot_pose.pose.position.x)**2 + 
                             (waypoint.pose.position.y - self.robot_pose.pose.position.y)**2)
            
            if dist > self.look_ahead_distance + 0.2:
                break
            else:
                
                error = abs(self.look_ahead_distance - dist)
                if error < min_error:
                    min_error = error
                    self.look_ahead_point = waypoint
            print(error)
        if self.look_ahead_point is None:
            #rospy.logwarn("Lookahead point not found!")
            pass
        print("The lookahead point is:",self.look_ahead_point)

    def get_twist(self):
        
        if self.look_ahead_point is None or self.robot_pose is None:
            #rospy.logwarn("Lookahead point or robot pose not received!")
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            return twist
        
        # Extract the x and y coordinates of the robot pose and the look-ahead point
        x_r = self.robot_pose.pose.position.x
        y_r = self.robot_pose.pose.position.y
        x_L = self.look_ahead_point.pose.position.x
        y_L = self.look_ahead_point.pose.position.y
               
        # Calculate the distance between the robot pose and the look-ahead point
        L = math.sqrt((x_L - x_r) ** 2 + (y_L - y_r) ** 2)
        
        # We transform the goal point from /map to /odom
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "map",rospy.Time(0),rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #rospy.logwarn(f"Failed to transform goal point: {e}")
            pass
        

        target_pose_transformed = tf2_geometry_msgs.do_transform_pose(self.look_ahead_point,transform)
        
        # Calculate the curvature of the path
        if L == 0:
            k = 0
        else:
            k = 2 * target_pose_transformed.pose.position.y / (L ** 2)
        
        # Calculate the desired angular velocity and linear velocity
        w = self.target_velocity * k
        v = self.target_velocity
        
        # Create a Twist message with the desired angular and linear velocities
        twist = Twist()
        if self.action == True:
            twist.angular.z = w
            twist.linear.x = v
            print(twist)
        elif self.action == False:
            twist.angular.z = 0
            twist.linear.x = 0

        return twist

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    controller=PurePursuitController()
    rate=rospy.Rate(10)
    
    while not rospy.is_shutdown():
        controller.get_closest_point()
        controller.get_lookahead_point()
        twist = controller.get_twist()
        controller.twist_pub.publish(twist)
        rate.sleep()        
        
        
    
    
    
    



    








