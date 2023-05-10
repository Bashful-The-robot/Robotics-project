#!/usr/bin/env python3

import rospy
from robp_msgs.msg import flags
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Pose
from visualization_msgs.msg import MarkerArray as vMarkerArray
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from std_msgs.msg import Int8
import tf2_ros
class Brain:
    def __init__(self):
        self.arm_mission = 0
        self.path_gen_mission = 0
        self.perception = False
        self.path_control = False
        #self.target = 0
        self.target = Pose()   
        self.target.position.x = 0 #Pose
        self.target.position.y = 0
        self.target.position.z = 0
        self.object_id = -1

        self.robot_pose = None

        self.obj_list = []
        self.box_list = [None]*3
        self.PairFoundlist = [] #obj, box

        #General inits
        self.rate = rospy.Rate(10)
        self.PairFoundlist = [] #obj, box
        self.pairFound = False

        self.arm_motion_status = -1

        # Buffer    
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


        # Subscribers
        
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/marker2',vMarkerArray,self.perception_cb,queue_size=1)
        rospy.Subscriber('/marker', vMarkerArray, self.boxes_callback)
        rospy.Subscriber('/arm_motion_complete', Int8, self.arm_motion_callback, queue_size=1) #0 dioing, 1 picked, 2 placed
        # Publishers
        self.pub = rospy.Publisher('/system_flags',flags,queue_size=10)
        self.pub_to_path = rospy.Publisher('/target_pos', PoseStamped, queue_size=10)

    # notAtgoal = True
    # pathObj = PathControl()
    # while notAtgoal and not rospy.is_shutdown():
    #     notAtgoal = pathObj.astarObj.atGoal()

    def pose_callback(self,msg):
        try:
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(1.5))
            self.robot_pose = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform)
            #print("The robot position is ", self.robot_pose)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: {e}")
            rospy.logwarn("Robot pose: {}".format(self.robot_pose))

    
        if self.target.position.x != None and self.robot_pose!= None:

            #distance_to_target = math.isclose(self.target.position,self.robot_pose.pose.position,abs_tol=0.1)
            distance_to_target = math.sqrt((self.target.position.x - self.robot_pose.pose.position.x)**2 + 
                                 (self.target.position.y - self.robot_pose.pose.position.y)**2)
            print(distance_to_target)
            if distance_to_target < 0.20:
                self.arm_mission = 0
                self.path_gen_mission = 0
                self.perception = False
                self.path_control = False
                #rospy.sleep(2)
                current_robot_loc = self.PairFoundlist.index(self.target)
                if current_robot_loc == 0 and self.arm_motion_status != 1:
                    self.arm_mission = 1 #to pick up 
                elif current_robot_loc == 1:
                    self.arm_mission = 2 #to place

                

    def arm_motion_callback(self, msg):
        self.arm_motion_status = msg.data

    def perception_cb(self,msg):
        self.obj_list = []
        for marker in msg.markers:
            #6 cube
            #7 ball
            #0-5 animal
            self.obj_list.append([marker.id, marker.pose])
            #try:
            #    #mapView = self.tf_buffer.lookup_transform("map","camera_color_optical_frame",marker.header.stamp, rospy.Duration(0.5))
            #    #do_trans = tf2_geometry_msgs.do_transform_pose(marker, mapView)
            #    #do_trans.pose.position.z = 0   
            #    #self.obj_list.append([marker.id, do_trans.pose])
            #except:
            #    continue
        self.check_pair()

    def boxes_callback(self, msg):
        
        for marker in msg.markers:
            if marker.id != 500 and marker.id != 0 and marker.id not in self.box_list:
                #print(marker.id)
                self.box_list[marker.id - 1] = marker.pose
        self.check_pair()

    def check_pair(self):
        #cube: 1
        #ball: 2
        #animal: 3

        
        a = len(self.PairFoundlist)
        for obj in self.obj_list:
            #modulus
            if 0 <= obj[0]%8 <= 5:
                if self.box_list[2] != None:
                    #print("found a pair - animal")
                    self.PairFoundlist = [obj[1], self.box_list[2]] #pose of obj and box
                    self.pairFound = True
                    break
            elif obj[0]%8 == 6:
                if self.box_list[0] != None:
                    #print("found a pair - cube")
                    self.PairFoundlist = [obj[1], self.box_list[0]] #pose of obj and box
                    self.pairFound = True
                    break
            elif obj[0]%8 == 7:   
                if self.box_list[1] != None:
                   # print("found a pair - ball")
                    self.PairFoundlist = [obj[1], self.box_list[1]] #pose of obj and box
                    self.pairFound = True
                    break
        

    def send_loc(self):
        if self.arm_motion_status == 1:
            self.arm_mission = 0
            self.path_gen_mission = 1
            self.perception = False
            self.path_control = True
            try:
                self.target = self.PairFoundlist[0]
            except IndexError as e :
                print(f"The value of self.PairFoundlist[0] is: {self.PairFoundlist}")

            loc = PoseStamped()
            loc.header.stamp = rospy.Time.now()
            loc.header.frame_id = "map"#??
            try:
                loc.pose = self.PairFoundlist[0]
            except:
                pass
        #print(loc.pose)
        #sending obj pose
            self.pub_to_path.publish(loc)
        #get a flag from the system after the obj is picked up
        # while not self.pickedup and not rospy.is_shutdown():
        #     self.rate.sleep()
        if self.arm_motion_status == 1:
            self.path_control = True
            try:
                self.target = self.PairFoundlist[1]
            except IndexError as e :
                print("The value of self.PairFoundlist[0] is: {self.PairFoundlist}")
        
            loc = PoseStamped()
            loc.header.stamp = rospy.Time.now()
            loc.header.frame_id = "map"#??
            try:
                loc.pose = self.PairFoundlist[1]
            except:
                pass
            
            self.pub_to_path.publish(loc)
            self.arm_motion_status = 0


    def publisher(self):
        flag = flags()
        flag.header.stamp = rospy.Time.now()
        flag.arm_mission = self.arm_mission 
        flag.path_gen_mission = self.path_gen_mission 
        flag.perception = self.perception 
        flag.path_control = self.path_control 
        if self.target!= None:
            flag.target = self.target 
        flag.object_id = str(self.object_id)
        #publish

        self.pub.publish(flag)        
        self.rate.sleep()
    


if __name__ == '__main__':
    rospy.init_node('brain')
    brain=Brain()
    while not rospy.is_shutdown():
        if brain.pairFound and brain.arm_mission  == 0:
            brain.send_loc()
        brain.publisher()
        