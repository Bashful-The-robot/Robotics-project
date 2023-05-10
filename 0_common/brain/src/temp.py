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

class TimeControl(pt.behaviour.Behaviour):
    '''This is for the time limit!'''
    def __init__(self):
        self.timeLimit = 2*60 #2 minutes
        self.maxTime = rospy.Time.now() + rospy.Duration(self.timeLimit)
    
    def timerIsNotDone(self):
        NotDone = True                         #Timer is not Done in the beginning
        if rospy.Time.now() >= self.maxTime:    #Timelimit has been exceeded
            NotDone = True                      #Timer is done
        return NotDone

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
            super(Perception,self).__init__("Perception")
        def perception_cb(self,msg):
            self.obj_list = []
            for marker in msg.markers:
                #6 cube
                #7 ball
                #0-5 animal
                self.obj_list.append([marker.id, marker.pose])
            #self.check_pair()


        def boxes_callback(self, msg):
        
            for marker in msg.markers:
                if marker.id != 500 and marker.id != 0 and marker.id not in self.box_list:
                    self.box_list[marker.id - 1] = marker.pose
            #self.check_pair()

        def update(self):
            #cube: 1
            #ball: 2
            #animal: 3
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
        
        def tellWorld(self):
            print("WE FOUND A PAIR")
            return pt.common.Status.SUCCESS
        def allObjsPlaced(self):
            return False


class PathPlanning():
    def __init__(self):
        self.id = "PathPlanning"
        self.pathMission = Int8()
        self.notAtGoal = True 
        self.goal = None
        self.robot_pose = None
        self.targetx = None
        self.targety = None
        self.DoneExploring = False
        self.lastDist2Goal = None
        self.robotFound = False
        # Attribute for publisher
        self.target = Pose()   
        self.target.position.x = None #Pose
        self.target.position.y = None
        self.target.position.z = 0

        
        # Buffer    
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # #Subscriber
         
        rospy.Subscriber('/state/cov',PoseWithCovarianceStamped, self.state_cov_callback)

        
    


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


    def atGoal(self,data):
        '''Generic function to see if we are close to target'''
            
        target = PoseStamped()
        target.pose.position.x = data[0]
        target.pose.position.y = data[1]
        target.pose.position.z = 0 
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

        if delta < 0.01:                    # Check if we are moving
            return pt.common.Status.FAILURE

        if distance_to_target > 0.25:       # Moving but not close enough
            return pt.common.Status.RUNNING
        elif distance_to_target <= 0.25:    # Close enough, reset dis2Goal
            self.lastDist2Goal = None
            #reset target position value
            self.target.position.x = None 
            self.target.position.y = None
            return  pt.common.Status.SUCCESS
        
    def explorerDone_callback(self,msg):
        self.DoneExploring = msg

    def isExplorerDone(self):           #Check in behaviour tree
        return self.DoneExploring

    def flags_callback(self,msg):
        self.armMission = msg.arm_mission
        self.pathMission = msg.path_mission
        self.objectRemoved = msg.perception
        self.object_id = msg.object_id
        self.goal = msg.Pose.target

        if self.pathMission.data == 0:      #Can only be zero in this callback
            self.sendMissions(0)    

    def landmarkDetected(self):
        isDetected = False
        if self.pathMission.data ==0:       #Will be 0 if a landmark has been detected
            isDetected = True
        return isDetected
    
     #Missions


    #def sendMissions(self,mission):
    #    self.pathMission.data = mission 
    #    rate = rospy.rate(10)
    #    NotAtGoal = True
    #    if self.pathControl.astarObj.isInsideWS(self.goal) is not None: #if inside WS
    #        while NotAtGoal and not rospy.is_shutdown():
    #            if mission != 1 or (mission == 1 and self.goal is not None):    #To avoid errors with path planning to undefined goal position
    #                NotAtGoal = not self.pathControl.atGoal(self.goal)          #False if at goal
    #                self.pubPathMission.publish(self.pathMission)
    #                self.pubGoal.publish(self.goal)
    #                rospy.sleep()
    #    self.pubMotors.publish(False)

    def sendMissions(self,mission):

        self.target.position.x = mission[0] #Pose
        self.target.position.y = mission[1]

        return True


class ArmActions(pt.behaviour.Behaviour):
    def __init__(self):
        self.id = "ArmAction"
        self.action = Int8()
        self.action.data = 0        #Do nothing
        #self.pubSignal = rospy.Publisher('/flags',Int8,queue_size=1)    #Need to change to correct format

    def pickUp(self):
        self.action.data = 1
        self.publishAction()

    def place(self):
        self.action.data = 2
        self.publishAction()

    def publishAction(self):
        rate = rospy.rate(10)
        while not rospy.is_shutdown:
            self.pubSignal.publish(self.action.data)
            rate.sleep()


class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self,name):
        
        rospy.loginfo("Initialising behaviour tree")
        self.timeControl = TimeControl()     #Time starts counting from now 
        self.pathplanning = PathPlanning()
        self.armActions = ArmActions()   
        self.perception = Perception()
        # Publisher
        self.pub = rospy.Publisher('/system_flags',flags,queue_size=10)
        done = self.pathplanning.DoneExploring and self.perception.allObjsPlaced()      #Need to add
        # Attributes

        self.arm_mission = 0
        self.path_gen_mission = 0
        self.perception_mission = False
        self.path_control = False

        self.target = Pose()   
        self.target.position.x = None #Pose
        self.target.position.y = None
        self.target.position.z = None
        self.object_id = -1





        #Time out conditions
        
        '''killROS = pt.composites.Sequence(
            children=[self.pathplanning.atGoal([0,0]), rospy.on_shutdown(self.shutdown)]
        )
        move2origoNkill = pt.composites.Sequence(
            children=[self.pathplanning.sendMissions([0,0]),self.pathplanning.atGoal([0,0]), rospy.on_shutdown(self.shutdown)]
        )

        timeTree = pt.composites.Sequence(
            children=[self.timeControl.timerIsNotDone(),timeoutTree]
        )
        timeoutTree = pt.composites.Selector(
        
            children=[killROS, move2origoNkill]
        )'''
        '''
        #Task tree
        pickUp_Tree = pt.composites.Sequence(
            children=[armActions.pickUp(),Perception.checkIfPickedUp()]
        )
        place_Tree = pt.composites.Sequence(
            children=[perception.objIsBox(), armActions.place()]
        )

        handleObj_Tree = pt.composites.Selector(
            children=[place_Tree,pickUp_Tree]
        )
        
        armMovementsTree = pt.composites.Sequence(
        children=[perception.inFrontOfObj(),handleObj_Tree]
        )

        landmarkTree = pt.composites.Sequence(
        children=[pathActions.landmarkDetected(),pathActions.sendMissions(0)]
        )

        pairingTree = pt.composites.Sequence(
        children=[perception.pairFound(),pathActions.sendMissions(1),armMovementsTree]
        )
        
        tasksTree = pt.composites.Selector(
            children=[pairingTree, pathActions.sendMissions(-1), landmarkTree]
        )
        isDone = pt.composites.Sequence(
        children=[done,self.pathplanning.sendMissions([0,0]),timeoutTree] # if explored everything unknown and placed object goTo 0,0
        )
        '''
        #pair = pt.composites.Sequence(name='look4pair',
        #children=[])
        #Main tree
        '''mainTree = pt.composites.Selector(
        children=[timeTree,isDone,tasksTree])'''
        mainTree = pt.composites.Selector()
        mainTree.add_children([self.perception])
        #tree = ptr.trees.BehaviourTree(mainTree)
        tree = pt.trees.BehaviourTree(root=mainTree)
        
        super(BehaviourTree,self).__init__(tree)
        # execute the behaviour tree
        #rospy.sleep(5)
        self.setup(timeout=15)
        while not rospy.is_shutdown():
            tree.tick_tock(1)
            if self.perception.pairFound:
                self.send_loc()   
            self.publish()

    
    def send_loc(self):
        '''Send location to path gen and control'''

        #Set flags for flags publisher
        self.arm_mission = 0
        self.path_gen_mission = 1
        self.perception = False
        self.path_control = True
        self.target = self.PairFoundlist[0]
        loc = PoseStamped()
        loc.header.stamp = rospy.Time.now()
        loc.header.frame_id = "map"
        loc.pose = self.PairFoundlist[0]
        pass
        
    def shutdown(self):
        rospy.logwarn("SHUTTING DOWN!")


    def publish(self):

        flag = flags()
        flag.header.stamp = rospy.Time.now()

        flag.arm_mission = self.arm_mission 
        flag.path_gen_mission = self.path_gen_mission 
        flag.perception = self.perception 
        flag.path_control = self.path_control 
        if self.target!= None:
            flag.target = self.target 
        flag.object_id = str(self.object_id)
        self.pub.publish(flag) 

if __name__ == "__main__":
    rospy.init_node('Brain')
    try:
        BehaviourTree("DT")
    except rospy.ROSInterruptException:
        pass

    rospy.spin()