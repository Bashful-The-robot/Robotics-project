#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from robp_msgs.msg import flags
from geometry_msgs.msg import PoseStamped

class TimeControl():
    def __init__(self):
        self.timeLimit = 2*60 #2 minutes
        self.maxTime = rospy.Time.now() + rospy.Duration(self.timeLimit)
    
    def timerIsNotDone(self):
        NotDone = True                         #Timer is not Done in the beginning
        if rospy.Time.now() >= self.maxTime:    #Timelimit has been exceeded
            NotDone = True                      #Timer is done
        return NotDone
    
class Pairing():
        def __init__(self):
            self.objectFound = False
            self.boxFound = False
            self.ObjRemoved= False 

            #Subscribers
            rospy.Subscriber('/topic',x, self.object_callback)  #Get type of obj
            rospy.Subscriber('/topic',flags, self.flags_callback)  
        
        def flags_callback(self,msg):
            self.ObjRemoved = msg.perception
            self.object_id = msg.object_id
            self.goal = msg.Pose.target

        def checkIfPickedUp(self):
            #return self.ObjRemoved
            return True
        
        def objIsBox(self):
            return False
        
        def pairFound(self):
            return False
        
        def inFrontOfObj(self):
            return False
        
        def allObjsFound(self):
            return False
        

class PathActions():
    def __init__(self):
        self.pathMission = Int8()
        self.notAtGoal = True 
        self.goal = None
        self.pathControl = PathControl()
        self.DoneExploring = False
        # #Subscriber
        rospy.Subscriber('/topic',flag, self.flags_callback)  
        rospy.Subscriber('/ExplorerDone',Bool, self.explorerDone_callback)  

        #Publishers
        self.pubPathMission= rospy.Publisher('/pathMission',Int8,queue_size=10) #Publish the mission to PathControl
        self.pubGoal = rospy.Publisher('/target_pos',PoseStamped,queue_size=10)
        self.pubMotors= rospy.Publisher('/runMotors',Bool,queue_size=10) #Publish the mission to PathControl
    
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
    def sendMissions(self,mission):
        self.pathMission.data = mission 
        rate = rospy.rate(10)
        NotAtGoal = True
        if self.pathControl.astarObj.isInsideWS(self.goal) is not None: #if inside WS
            while NotAtGoal and not rospy.is_shutdown():
                if mission != 1 or (mission == 1 and self.goal is not None):    #To avoid errors with path planning to undefined goal position
                    NotAtGoal = not self.pathControl.atGoal(self.goal)          #False if at goal
                    self.pubPathMission.publish(self.pathMission)
                    self.pubGoal.publish(self.goal)
                    rospy.sleep()
        self.pubMotors.publish(False)


class ArmActions():
    def __init__(self):
        self.action = Int8()
        self.action.data = 0        #Do nothing
        self.pubSignal = rospy.Publisher('/flags',Int8,queue_size=1)    #Need to change to correct format

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



class BehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        timeControl = TimeControl()     #Time starts counting from now 
        pathActions = PathActions()
        armActions = ArmActions()   
        pairingActions = Pairing()
        done = pathActions.DoneExploring and pairingActions.allObjsFound()      #Need to add

        #Time out conditions
        timeoutTree = pt.composites.Selector(
            children=[killROS, pathActions.sendMissions(0)]
        )
        killROS = pt.composites.Sequence(
            children=[pathActions.pathControl.atGoal(0), rospy.on_shutdown()]
        )

        timeTree = pt.composites.Sequence(
            children=[timeControl.timerIsNotDone(),timeoutTree]
        )

        #Task tree
        pickUp_Tree = pt.composites.Sequence(
            children=[armActions.pickUp(),Pairing.checkIfPickedUp()]
        )
        place_Tree = pt.composites.Sequence(
            children=[pairingActions.objIsBox(), armActions.place()]
        )

        handleObj_Tree = pt.composites.Selector(
            children=[place_Tree,pickUp_Tree]
        )
        
        armMovementsTree = pt.composites.Sequence(
        children=[pairingActions.inFrontOfObj(),handleObj_Tree]
        )

        landmarkTree = pt.composites.Sequence(
        children=[pathActions.landmarkDetected(),pathActions.sendMissions(0)]
        )

        pairingTree = pt.composites.Sequence(
        children=[pairingActions.pairFound(),pathActions.sendMissions(1),armMovementsTree]
        )
        
        tasksTree = pt.composites.Selector(
            children=[pairingTree, pathActions.sendMissions(-1), landmarkTree]
        )

        isDone = pt.composites.Sequence(
        children=[done,pathActions.sendMissions(1),timeoutTree]
        )
        #Main tree
        mainTree = pt.composites.Selector(
        children=[timeTree,isDone,tasksTree])
        
        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): 
            self.tick_tock(1)


if __name__ == "__main__":
    rospy.init_node('Brain')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
