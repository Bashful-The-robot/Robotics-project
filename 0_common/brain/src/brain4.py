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
from aruco_msgs.msg import MarkerArray, Marker

#from reactive_sequence import RSequence

##TO BE CHANGED #CHANGE!!!!! change everything here before starting!!!!!!!!!!!!!!
stop_distance = 0.2

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
        for obj in self.obj_list:
            #modulus
            if 0 <= obj[0]%8 <= 5:
                if self.box_list[2] != None:
                    rospy.loginfo("success preception - animal")
                    #print("found a pair - animal")
                    self.PairFoundlist = [obj[1], self.box_list[2]] #pose of obj and box
                    self.pairFound = True
                    common_dict['path_gen_mission']=1
                    common_dict['arm_mission']=0
                    common_dict['perception']=False
                    common_dict['path_control']=True
                    common_dict['pickPose'].position = self.PairFoundlist[0].position
                    common_dict['placePose'].position = self.PairFoundlist[1].position
                    common_dict['target'].position = self.PairFoundlist[0].position  
                    common_dict['object_id']=obj[0]
                    common_dict['Status'] = -1
                    return pt.common.Status.SUCCESS
            elif obj[0]%8 == 6:
                if self.box_list[0] != None:
                    #print("found a pair - cube")
                    #rospy.loginfo("found a pair - cube")
                    self.PairFoundlist = [obj[1], self.box_list[0]] #pose of obj and box
                    self.pairFound = True
                    common_dict['path_gen_mission']=1
                    common_dict['arm_mission']=0
                    common_dict['perception']=False
                    common_dict['path_control']=True
                    common_dict['pickPose'].position = self.PairFoundlist[0].position
                    common_dict['placePose'].position = self.PairFoundlist[1].position
                    common_dict['target'].position = self.PairFoundlist[0].position   
                    print("the target is:", common_dict['target'].position)
                    common_dict['object_id']=obj[0]
                    common_dict['Status'] = -1
                    #print(f"Updated dict as: {common_dict}")
                    rospy.loginfo("success preception - cube")
                    #print("success preception - cube")
                    return pt.common.Status.SUCCESS
                    
            elif obj[0]%8 == 7:   
                if self.box_list[1] != None:
                    # print("found a pair - ball")
                    self.PairFoundlist = [obj[1], self.box_list[1]] #pose of obj and box
                    self.pairFound = True
                    common_dict['path_gen_mission']=1
                    common_dict['arm_mission']=0
                    common_dict['perception']=False
                    common_dict['path_control']=True
                    common_dict['pickPose'].position = self.PairFoundlist[0].position
                    common_dict['placePose'].position = self.PairFoundlist[1].position
                    common_dict['target'].position = self.PairFoundlist[0].position   
                    common_dict['object_id']=obj[0]
                    rospy.loginfo("success preception - ball")
                    #print("success preception - ball")
                    return pt.common.Status.SUCCESS
        # Have not seen any pairs
        self.pairFound = False
        rospy.logerr("failure preception")
        #print("failure preception")
        return pt.common.Status.FAILURE
    
        
class AtGoal(pt.behaviour.Behaviour):
    def __init__(self):
        self.id = "PathPlanning"
        
        # Attribute for publisher
        self.target = Pose()   
   
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
        print("AtGoal update")
        target = PoseStamped()
        #common_dict['target'] = common_dict['pickPose']        
        common_dict['perception'] = False
        target.pose = common_dict['target']

        if not self.robotFound:
            rospy.logwarn("Running Atgoal - waiting for robot to be found and positioned")
            return pt.common.Status.RUNNING
        distance_to_target = math.sqrt((target.pose.position.x - self.robot_pose.pose.position.x)**2 + 
                                 (target.pose.position.y - self.robot_pose.pose.position.y)**2)
        #print("distance to target is ", distance_to_target)
        if self.lastDist2Goal == None:  #not had any delta
            self.lastDist2Goal = distance_to_target
            #print("running atgoal None")
            rospy.logwarn("running Atgoal - None")
            return pt.common.Status.RUNNING
        else:

            delta = distance_to_target - self.lastDist2Goal 

        #if delta < 0.01:                    # Check if we are moving
        #    return pt.common.Status.FAILURE

        if distance_to_target > stop_distance: #0.25:       # Moving but not close enough 
            #print("running atgoal > stop_distance")
            rospy.logwarn("running atgoal - > stop_distance")
            return pt.common.Status.RUNNING
        elif distance_to_target <= stop_distance: #0.25:    # Close enough, reset dis2Goal
            self.lastDist2Goal = None
            #reset target position value
        
            common_dict['path_gen_mission']=0
            common_dict['arm_mission']=0
            common_dict['perception']=False
            common_dict['path_control']=False
            common_dict['target'] = common_dict['placePose']
            #rospy.loginfo(f"Updated dict as: {common_dict}")
            rospy.loginfo("success Atgoal")
            return  pt.common.Status.SUCCESS   
    
class Pick(pt.behaviour.Behaviour):
    '''Generic pick behaviour'''
    def __init__(self):
        super(Pick,self).__init__()
        rospy.Subscriber('/arm_motion_complete', Int8, self.arm_status_callback)
        self.arm_status = False

    def arm_status_callback(self,msg):
        self.arm_status = msg.data

    def update(self):
        '''Generic function to pick'''
        if self.arm_status == 0:
            common_dict['path_gen_mission']=0
            common_dict['arm_mission']=1
            common_dict['perception']=False
            common_dict['path_control']=False
            common_dict['target'] = common_dict['pickPose']
            #rospy.loginfo(f"Updated dict as: {common_dict}")
            rospy.logwarn("Running pick")
            return pt.common.Status.RUNNING
        elif self.arm_status == 1:
            common_dict['path_gen_mission']=1
            common_dict['arm_mission']=0
            common_dict['perception']=False
            common_dict['path_control']=True
            common_dict['target'] = common_dict['placePose']
            rospy.loginfo("success pick")
            #print("success pick")
            return pt.common.Status.SUCCESS
        else:
            rospy.logerr("failure pick")
            #print("failure pick")
            return pt.common.Status.FAILURE

class Place(pt.behaviour.Behaviour):
    '''Generic place behaviour'''
    def __init__(self):
        super(Place,self).__init__()
        rospy.Subscriber('/arm_motion_complete', Int8, self.arm_status_callback)
        self.arm_status = False

    def arm_status_callback(self,msg):
        self.arm_status = msg.data

    def update(self):
        '''Generic function to place'''
        if self.arm_status == 0: 
            common_dict['path_gen_mission']=0
            common_dict['arm_mission']=2
            common_dict['perception']=False
            common_dict['path_control']=False
            common_dict['target'] = common_dict['placePose']
            
            #rospy.logwarn(f"Updated dict as: {common_dict}")
            rospy.logwarn("Running place")
            return pt.common.Status.RUNNING
        elif self.arm_status == 2:
            common_dict['path_gen_mission']=0
            common_dict['arm_mission']=0
            common_dict['perception']=True
            common_dict['path_control']=False
            common_dict['target'] = Pose() #common_dict['pickPose']
            common_dict['pickPose'] = Pose()
            common_dict['placePose'] = Pose()
#            common_dict['object_id']=None
            rospy.loginfo("Success place")
            #print("success palce")
            return pt.common.Status.SUCCESS
        else:
            rospy.logerr("Failure place")
            #print("failure palce")
            return pt.common.Status.FAILURE

class DetectedAnchorAruco(pt.behaviour.Behaviour):
    def __init__(self):
        super(DetectedAnchorAruco, self).__init__()
        rospy.Subscriber('/aruco_single/pose',PoseStamped, self.anchor_aruco_callback)
        self.detected_pub = rospy.Publisher('/detected_anchor_aruco', Bool, queue_size=1)
        self.detected_anchor_aruco = False

    def anchor_aruco_callback(self, data):
        self.detected_anchor_aruco = True

    def update(self):
        if self.detected_anchor_aruco:
            common_dict['arm_mission'] = 0
            common_dict['path_gen_mission'] = 0
            common_dict['perception'] = False
            common_dict['path_control'] = False
            common_dict['target'] = Pose()
            common_dict['pickPose'] = Pose()
            common_dict['placePose'] = Pose()
            common_dict['object_id'] = None
            common_dict['Status'] = -1
            rospy.loginfo("success detected anchor aruco")
            #print("success detected anchor aruco")
            self.detected_pub.publish(self.detected_anchor_aruco)
            self.detected_anchor_aruco = False
            return pt.common.Status.SUCCESS
        else:
            #print("failure detected anchor aruco")
            rospy.logerr("failure detected anchor aruco")
            self.detected_pub.publish(self.detected_anchor_aruco)
            return pt.common.Status.FAILURE

class Exploration(pt.behaviour.Behaviour):
    def __init__(self):
        super(Exploration, self).__init__()
        
    def update(self):
        common_dict['arm_mission'] = 0
        common_dict['path_gen_mission'] = -1
        common_dict['perception'] = False
        common_dict['path_control'] = True
        common_dict['target'] = Pose()
        common_dict['pickPose'] = Pose()
        common_dict['placePose'] = Pose()
        common_dict['object_id'] = None
        rospy.loginfo("success exploration")
        #print("success exploration")
        return pt.common.Status.SUCCESS ## check with Filippa
    
class DetectNewLandmark(pt.behaviour.Behaviour):
    def __init__(self):
        super(DetectNewLandmark, self).__init__()
        rospy.Subscriber('/camera/aruco/markers', MarkerArray, self.aruco_callback)
        self.detected_previously_list = [0,0,0] #idx:: boxes 1,2,3 #item:: 0: not detected before, 1:new detection, 2:old detection

    def aruco_callback(self, msg):
        for marker in msg.markers:
            if 1 <= marker.id <= 3:
                if self.detected_previously_list[marker.id - 1] == 0:
                    self.detected_previously_list[marker.id - 1] = 1 
                    
    def update(self):
        for idx, item in enumerate(self.detected_previously_list):
            if item == 1:
                self.detected_previously_list[idx] = 2
                #print("success detected new landmark")
                common_dict['arm_mission'] = 0
                common_dict['path_gen_mission'] = 2 ##check with Filippa
                common_dict['perception'] = False
                common_dict['path_control'] = True
                target_pos = Pose()
                target_pos.position.x = 0 #CHANGE
                target_pos.position.y = 0 #CHANGE
                target_pos.position.z = 0
                common_dict['target'] = target_pos ##Anchor pose 0,0,0
                common_dict['pickPose'] = Pose()
                common_dict['placePose'] = Pose()
                common_dict['object_id'] = None
                #print("success detected new landmark")
                rospy.loginfo("success detected new landmark")
                return pt.common.Status.SUCCESS
        rospy.logerr("failure detected new landmark")
        #print("failure detected new landmark")
        return pt.common.Status.FAILURE

class CompletedPlaceOrExploration(pt.behaviour.Behaviour):
    def __init__(self):
        super(CompletedPlaceOrExploration, self).__init__()
#        rospy.Subscriber('/arm_motion_complete', Int8, self.arm_status_callback)
#        self.arm_status = -1
#
#    def arm_status_callback(self,msg):
#        self.arm_status = msg.data

    def update(self):
        if common_dict['Status'] == 0 or common_dict['Status'] == 1:
#            self.arm_status = -1
            common_dict['arm_mission'] = 0
            common_dict['path_gen_mission'] = 0
            common_dict['perception'] = False
            common_dict['path_control'] = FalseB
            common_dict['target'] = Pose()
            common_dict['pickPose'] = Pose()
            common_dict['placePose'] = Pose()
            common_dict['object_id'] = None
            common_dict['Status'] = -1
            #print("success completed place")
            rospy.loginfo("success completed place")
            return pt.common.Status.SUCCESS
        else:
            #print("failure completed place")
            rospy.logerr("failure completed place")
            return pt.common.Status.FAILURE

class TimeControl(pt.behaviour.Behaviour):
    '''This is for the time limit!'''
    def __init__(self):
        super(TimeControl, self).__init__()
        self.timeLimit = 2*60 #2 minutes ##TO BE CHANGED
        self.maxTime = rospy.Time.now() + rospy.Duration(self.timeLimit)
    
    def update(self):
        if rospy.Time.now() >= self.maxTime:    #Timelimit has been exceeded
            #print("success timecontrol")
            rospy.loginfo("success timecontrol")
            return pt.common.Status.SUCCESS
        else:
            #print("failure timecontrol")
            rospy.logerr("failure timecontrol")
            return pt.common.Status.FAILURE
    
def bpublish():
    flag = flags()
    flag.header.stamp = rospy.Time.now()

    flag.arm_mission = common_dict['arm_mission']
    flag.path_gen_mission = common_dict['path_gen_mission']
    flag.perception = common_dict['perception']
    flag.path_control =  common_dict['path_control']
    if target!= None:
        flag.target = common_dict['target'] 
    flag.object_id = str(common_dict['object_id'])
    pub_flags.publish(flag) 

if __name__ == "__main__":
    # Init ros node
    rospy.init_node('Brain')
    # Init flags
    arm_mission = -2
    path_gen_mission = -2
    perception = False
    path_control = False
    target = Pose()
    pickPose = Pose()
    placePose = Pose()
    object_id = None
    Status = -1 #0: completed place, 1:was doing exploration
    pub_flags = rospy.Publisher('/system_flags', flags, queue_size=1)
    common_dict = {
        "arm_mission": arm_mission,
        "path_gen_mission": path_gen_mission,
        "perception": perception,
        "path_control": path_control,
        "target": target,
        "object_id": object_id,
        "pickPose": pickPose,
        "placePose": placePose,
        "Status": Status,
    }
  
    #pub_flags = rospy.Publisher('/system_flags', flags, queue_size=1)
    
    root = pt.composites.Selector()
    mainfunction = pt.composites.Sequence()
    anchor_or_placed = pt.composites.Selector()
    explore_or_newlandmark_or_pair = pt.composites.Selector()
    perception_pair = pt.composites.Sequence()
    #explore_or_see_landmark = pt.composites.Selector()
    new_landmark_procedure = pt.composites.Sequence()
    mission = pt.composites.Sequence()

    mission.add_children([AtGoal(),Pick(),AtGoal(),Place()])
    new_landmark_procedure.add_children([DetectNewLandmark(), AtGoal()])
    #explore_or_see_landmark.add_children([new_landmark_procedure, Exploration()])
    perception_pair.add_children([Perception(), mission])
    explore_or_newlandmark_or_pair.add_children([perception_pair, new_landmark_procedure, Exploration()])
    anchor_or_placed.add_children([DetectedAnchorAruco(), CompletedPlaceOrExploration()])
    mainfunction.add_children([anchor_or_placed, explore_or_newlandmark_or_pair])
    root.add_children([TimeControl(), mainfunction])
#    root = pt.composites.Sequence()
#    root.add_children([Perception(),AtGoal(),Pick(),AtGoal(),Place()])
    tree = pt.trees.BehaviourTree(root)
    tree.setup(timeout=10000)
    print(f"Init dict as: {common_dict}")

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        bpublish()
        rate.sleep()
        tree.tick()

    rospy.spin()