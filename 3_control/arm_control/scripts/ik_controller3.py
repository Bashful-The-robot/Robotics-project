#!/usr/bin/env python

import rospy
import math 
import tf2_ros
import tf2_geometry_msgs
from hiwonder_servo_msgs.msg import CommandDuration
from hiwonder_servo_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64
from robp_msgs.msg import flags
from std_msgs.msg import Int8


class RoboticArm:

    def __init__(self):

        # Define some flags ( We will subscribe to these flags)
        self.START_MISSION = False
        self.PICK = False
        self.LEAVE = False
        self.SUCCESS = False
       
        # Define the lengths of the three links of the robotic arm
        self.length1 = 101.0/1000
        self.length2 = 95.0/1000
        self.length3 = 165.0/1000

        # Initialize the orientation of the end effector 
        self.orientation = 0.0 

        # Velocity of the servos
        self.time_factor=(2500)/((math.pi)/2)

        # Initialize the joint angles to 0
        self.joint1_angle = 0.0
        self.joint2_angle = 0.0
        self.joint3_angle = 0.0
        self.joint4_angle = 0.0

        # Initialize the current states of the joints to 0
        self.joint1_state = 0.0
        self.joint2_state = 0.0
        self.joint3_state = 0.0
        self.joint4_state = 0.0

        # Initialize the command duration to zero 
        self.command1_duration = 0.0
        self.command2_duration = 0.0
        self.command3_duration = 0.0
        self.command4_duration = 0.0

        # initializa the target position to 0
        self.target_x = None
        self.target_y = None
        self.target_z = None

        # Define the publishers for the command of each joint
        self.joint1_command_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size=10)
        self.joint2_command_pub = rospy.Publisher('/joint2_controller/command_duration', CommandDuration, queue_size=10)
        self.joint3_command_pub = rospy.Publisher('/joint3_controller/command_duration', CommandDuration, queue_size=10)
        self.joint4_command_pub = rospy.Publisher('/joint4_controller/command_duration', CommandDuration, queue_size=10)
        self.gripper_pub = rospy.Publisher('/r_joint_controller/command', Float64, queue_size=10)
        self.arm_info_pub = rospy.Publisher('/arm_info', Int8, queue_size=1)
        self.arm_info = Int8()
        #self.arm_info.data = 0

        # Define the subscriber to get the joint states.
        self.state1_subs = rospy.Subscriber('/joint2_controller/state',JointState, self.state1_callback)
        self.state2_subs = rospy.Subscriber('/joint2_controller/state',JointState, self.state2_callback)
        self.state3_subs = rospy.Subscriber('/joint3_controller/state',JointState, self.state3_callback)
        self.state4_subs = rospy.Subscriber('/joint4_controller/state',JointState, self.state4_callback)
        
        # Other subscribers
        self.flag_sub = rospy.Subscriber('/system_flags',flags, self.flag_callback)
        self.object_pos_sub = rospy.Subscriber('arm_cam_pos',PointStamped)
       
       # Define the transform buffer and listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
        # extra
        self.transition = 0
    def state1_callback(self,msg):
        self.joint1_state = msg.current_pos

    def state2_callback(self,msg):
        self.joint2_state = msg.current_pos
    
    def state3_callback(self,msg):
        self.joint3_state = msg.current_pos
            
    def state4_callback(self,msg):
        self.joint4_state = msg.current_pos
    


    def flag_callback(self,msg):

        self.target_x = msg.target.position.x
        self.target_y = msg.target.position.y
        self.target_z = msg.target.position.z
        
        if msg.arm_mission == 1:
            self.START_MISSION = True 
            self.PICK = True
            self.LEAVE = False
        elif msg.arm_mission == 2:
            self.START_MISSION = True 
            self.PICK = False
            self.LEAVE = True
        else:
            self.START_MISSION = False 
            self.PICK = False
            self.LEAVE = False     
        
    
    def calculate_commands(self, x, y , z):
        
        #Align the arm with the target point and  correct the coordinates
        yaw = math.asin(y/x) 
        x = math.sqrt(y**2 + x**2)
        y = 0
        # Set different orientations of the gripper according to the distance to the object.
        if x>=300.0/1000:
            self.orientation = -0.785
        elif x>250.0/1000 and x<300.0/1000:
            self.orientation = -1.0
        elif x>=190.0/1000 and x<=250.0/1000:
            self.orientation = -1.22
        elif x>=160.0/1000 and x<190.0/1000:
            self.orientation = -1.5708
        elif x<160/1000:
            hola=0#exit()

        # Calculate the different commands    
        P_x = x - self.length3 * math.cos(self.orientation)
        P_z = z - self.length3 * math.sin(self.orientation)
        print(P_x)
        print(self.length3 * math.sin(self.orientation))
        print(z)
        print(P_z)
        print("second step")
        if yaw < 0:
            self.joint1_angle = yaw - 0.05
            
        else:
            self.joint1_angle = yaw - 0.085

        expr = ((P_z**2 + P_x**2) - (self.length1**2 + self.length2**2))/(2*self.length1*self.length2)
        
        if expr <=1:
            self.joint3_angle = - math.acos(((P_z**2 + P_x**2) - (self.length1**2 + self.length2**2))/(2*self.length1*self.length2))

            self.joint2_angle = math.atan2(P_z , P_x) -  math.atan2(self.length2 * math.sin(self.joint3_angle),(self.length1 + self.length2 * math.cos(self.joint3_angle)))

            self.joint4_angle = self.orientation - (self.joint2_angle + self.joint3_angle)

            self.joint2_angle -= math.pi/2
            
            self.SUCCESS = True

            self.command1_duration = abs(self.joint1_angle - self.joint1_state) * self.time_factor
            self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
            self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
            self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor 

        else:
            if yaw > 0:
                self.arm_info = -1
            elif yaw < 0:
                self.arm_info = -3
            
        
        print("command 1:",self.joint1_angle,"state:",self.command1_duration)
        print("command 2:",self.joint2_angle,"state:",self.command2_duration)
        print("command 3:",self.joint3_angle,"state:",self.command3_duration)
        print("command 4:",self.joint4_angle,"state:",self.command4_duration)
    
    
    
    def publish_commands(self):

        rospy.sleep(1)
        command1 = CommandDuration()
        command1.data = self.joint1_angle
        command1.duration = self.command1_duration
        self.joint1_command_pub.publish(command1)
        rospy.sleep(self.command1_duration/1000)
        
          
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)
                       
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)


        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)
        rospy.sleep(self.command2_duration/1000)
          
        if  self.PICK == True and self.LEAVE == False: # Pick object  sequence 
            rospy.sleep(0.5)
            gripper_command = Float64()
            gripper_command.data = -0.18
            self.gripper_pub.publish(gripper_command)
            rospy.sleep(1)
        elif self.PICK == False and self.LEAVE == True: # Leave object sequence 
            rospy.sleep(0.5)
            gripper_command = Float64()
            gripper_command.data = -1.3
            self.gripper_pub.publish(gripper_command)
            rospy.sleep(1)

    
    def return_to_origin (self):
        
        self.joint1_angle = -0.075
        self.joint2_angle = 0.5
        self.joint3_angle = -1.35
        self.joint4_angle = -1.76

        self.command1_duration = abs(self.joint1_angle - self.joint1_state) * self.time_factor
        self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
        self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
        self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor

        
        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)
        rospy.sleep(self.command2_duration/1000)
        
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)
        
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)

        command1 = CommandDuration()
        command1.data = self.joint1_angle
        command1.duration = self.command1_duration
        self.joint1_command_pub.publish(command1)
        rospy.sleep(self.command1_duration/1000)

    def go_detect_pos (self):
        
        self.joint1_angle = -0.075
        self.joint2_angle = -0.7
        self.joint3_angle = -0.8
        self.joint4_angle = -1.571

        self.command1_duration = abs(self.joint1_angle - self.joint1_state) * self.time_factor
        self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
        self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
        self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor

        gripper_command = Float64()
        gripper_command.data = -1.8
        self.gripper_pub.publish(gripper_command)
        rospy.sleep(1)
        
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)

        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)
        rospy.sleep(self.command2_duration/1000)
        
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)

        command1 = CommandDuration()
        command1.data = self.joint1_angle
        command1.duration = self.command1_duration
        self.joint1_command_pub.publish(command1)
        rospy.sleep(self.command1_duration/1000)

        

if __name__ == '__main__':
    
    rospy.init_node('ik_controller3') 
    
    controller = RoboticArm()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        if controller.START_MISSION == True:
            
            if controller.PICK == True:
                controller.go_detect_pos()
                msg = rospy.wait_for_message('arm_cam_pos',PointStamped)
                controller.calculate_commands(msg.point.x + 0.235, msg.point.y, -0.145)
            
            elif controller.LEAVE == True:   
                try:
                    Pose_aux = PoseStamped()
                    Pose_aux.header.frame_id = 'map'
                    Pose_aux.pose.position.x = controller.target_x
                    Pose_aux.pose.position.y = controller.target_y
                    Pose_aux.pose.position.z = 0
    
                    transform = controller.tf_buffer.lookup_transform('arm','map', rospy.Time(0), rospy.Duration(5.0))
                    pose_transformed = tf2_geometry_msgs.do_transform_pose(Pose_aux, transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Failed to transform pose: {e}")
                print("new position",pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z)
                controller.calculate_commands(pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z)
            
            if controller.SUCCESS == True:
                controller.publish_commands()

                controller.return_to_origin()

                if controller.PICK == True: 
                    controller.arm_info.data = 1
                elif controller.LEAVE == True:
                    controller.arm_info.data = 2

                controller.arm_info_pub.publish(controller.arm_info)

                controller.START_MISSION = False

                controller.SUCCESS = False

                rospy.sleep(1.5)
        
        controller.arm_info_pub.publish(controller.arm_info)
        rate.sleep() 

            
