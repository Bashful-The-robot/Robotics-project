#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from robp_msgs.msg import flags
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray as vMarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Pose
import tf2_ros
import tf2_geometry_msgs
import math
#from reactive_sequence import RSequence


class Perception(pt.behaviour.Behaviour):
    '''This class finds object pairs'''
    def __init__(self):
        self.objectFound = False
        self.boxFound = False
        self.ObjRemoved= False 
        self.id = "Perception"
        #Subscribers
        rospy.Subscriber('/marker2',vMarkerArray,self.perception_cb,queue_size=1)   #Perception
        rospy.Subscriber('/marker', vMarkerArray, self.boxes_callback)              #SLAM am_detector
        
        # Attributes
        self.obj_list = []  
        self.box_list = [None]*3
        self.PairFoundlist = [] #obj, box
        self.pairFound = False
        
        self.blackboard = pt.blackboard.Blackboard()
        
        super(Perception,self).__init__()
    def perception_cb(self,msg):
        self.obj_list = []
        for marker in msg.markers:
            #6 cube
            #7 ball
            #0-5 animal
            self.obj_list.append([marker.id, marker.pose])
        
    def boxes_callback(self, msg):
        for marker in msg.markers:
            if marker.id != 500 and marker.id != 0 and marker.id not in self.box_list:
                self.box_list[marker.id - 1] = marker.pose
        

    def update(self):
        #cube: 1
        #ball: 2
        #animal: 3
        print("Per")
        bdict["flags"] = 1000
        self.blackboard.odenisanidiot = "oden"
        for obj in self.obj_list:
            #modulus

            if 0 <= obj[0]%8 <= 5:
                if self.box_list[2] != None:
                    #print("found a pair - animal")
                    self.PairFoundlist = [obj[1], self.box_list[2]] #pose of obj and box
                    self.pairFound = True
                    return pt.common.Status.SUCCESS
            elif obj[0]%8 == 6:
                if self.box_list[0] != None:
                    #print("found a pair - cube")
                    self.PairFoundlist = [obj[1], self.box_list[0]] #pose of obj and box
                    self.pairFound = True

                    return pt.common.Status.SUCCESS
                    
            elif obj[0]%8 == 7:   
                if self.box_list[1] != None:
                    # print("found a pair - ball")
                    self.PairFoundlist = [obj[1], self.box_list[1]] #pose of obj and box
                    self.pairFound = True
                    return pt.common.Status.SUCCESS
        # Have not seen any pairs
        self.pairFound = False
        return pt.common.Status.FAILURE
    
        
class AtGoal(pt.behaviour.Behaviour):
    def __init__(self):
        self.id = "PathPlanning"
        
        # Attribute for publisher
        self.target = Pose()   
        self.target.position.x = None #Pose
        self.target.position.y = None
        self.target.position.z = 0
        self.robot_pose = None
        self.lastDist2Goal = None
        self.robotFound = False
        # Buffer    
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.blackboard = pt.blackboard.Blackboard()

        # #Subscriber
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.state_cov_callback)

        super(AtGoal,self).__init__()

    def state_cov_callback(self,msg):
        try:
            self.robotFound = True
            aux_pose = PoseStamped()
            aux_pose.pose= msg.pose.pose
            pose_transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(1.5))
            self.robot_pose = tf2_geometry_msgs.do_transform_pose(aux_pose, pose_transform)
            #print("The robot position is ", self.robot_pose)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform pose: {e}")
            rospy.logwarn("Robot pose: {}".format(self.robot_pose))

    def update(self):
        '''Generic function to see if we are close to target'''
        target = PoseStamped()
        target.pose.position.x = 3
        target.pose.position.y = 3
        target.pose.position.z = 0 
        print("hi", self.blackboard.odenisanidiot)
        if not self.robotFound:
            print("waiting for robot to be found and positioned")
            return pt.common.Status.RUNNING
        distance_to_target = math.sqrt((target.pose.position.x - self.robot_pose.pose.position.x)**2 + 
                                 (target.pose.position.y - self.robot_pose.pose.position.y)**2)
        if self.lastDist2Goal == None:  #not had any delta
            self.lastDist2Goal = distance_to_target
            return pt.common.Status.RUNNING
        else:

            delta = distance_to_target - self.lastDist2Goal 

        #if delta < 0.01:                    # Check if we are moving
        #    return pt.common.Status.FAILURE

        if distance_to_target > 0.25:       # Moving but not close enough
            return pt.common.Status.RUNNING
        elif distance_to_target <= 0.25:    # Close enough, reset dis2Goal
            self.lastDist2Goal = None
            #reset target position value
            self.target.position.x = None 
            self.target.position.y = None
            return  pt.common.Status.SUCCESS   

class SayHello(pt.behaviour.Behaviour):
    def __init__(self):
        super(SayHello, self).__init__()

    def update(self):
        print("Hello!")
        return pt.common.Status.SUCCESS
    

def bpublish(bdict):
    print("blackbaord", bdict["flags"])
    flag = flags()
    flag.header.stamp = rospy.Time.now()

    flag.arm_mission = arm_mission 
    flag.path_gen_mission = path_gen_mission 
    flag.perception = perception 
    flag.path_control = path_control 
    if target!= None:
        flag.target = target 
    flag.object_id = str(object_id)
    pub_flags.publish(flag) 

if __name__ == "__main__":
    rospy.init_node('Brain')
    arm_mission = -2
    path_gen_mission = -2
    perception = False
    path_control = False
    target = Pose()
    object_id = None

    bdict = {
        "flags": 1,
        "hi": 2
    }

    #publisher
    pub_flags = rospy.Publisher('/system_flags', flags, queue_size=1)

    root = pt.composites.Sequence()
    place_Tree = pt.composites.Selector(
            children=[SayHello()]
        )
    root.add_children([Perception(),AtGoal(),place_Tree])
    tree = pt.trees.BehaviourTree(root)
    tree.setup(timeout=10000)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        bpublish(bdict)
        rate.sleep()
        tree.tick()
    #    print("ANNIKA HOE ")
#
    rospy.spin()