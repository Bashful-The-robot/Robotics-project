#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

#This file publishes a -1 to the topic taskHandler and a position to the topic target_pos
#This will be used by the function main
###################
#Test
rospy.init_node('Testing')
pub = rospy.Publisher('/taskHandler', Int8, queue_size=10)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # create an Int8 message with a value of 42
    msg = Int8()
    msg.data = -1       #For exploring

    # publish the message
    pub.publish(msg)

    # sleep to maintain the loop rate
    rate.sleep()


#############
# def task_handler():
#     #rospy.init_node('TaskHandler', anonymous=True)
#     pub = rospy.Publisher('taskHandler', Int8, queue_size=10)
#     rate = rospy.Rate(10)

#     while not rospy.is_shutdown():
#         # create an Int8 message with a value of 42
#         msg = Int8()
#         msg.data = -1       #For exploring

#         # publish the message
#         pub.publish(msg)

#         # sleep to maintain the loop rate
#         rate.sleep()

# def target_pos_publisher():
#     rospy.init_node('TargetPosPublisher', anonymous=True)
#     pub = rospy.Publisher('target_pos', PoseStamped, queue_size=10)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         # create a PoseStamped message and publishes
#         msg = PoseStamped()
#         msg.pose.position.x = 0.5
#         msg.pose.position.y = 0.5
#         msg.pose.position.z = 0
#         msg.pose.orientation.x = 0.0
#         msg.pose.orientation.y = 0.0
#         msg.pose.orientation.z = 0.0
#         msg.pose.orientation.w = 0.0
#         pub.publish(msg)
#         rate.sleep()



# if __name__ == '__main__':
#     #rospy.init_node('Testing', anonymous=True)
#     rospy.init_node('Testing')
#     pub = rospy.Publisher('/taskHandler', Int8, queue_size=10)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         # create an Int8 message with a value of 42
#         msg = Int8()
#         msg.data = -1       #For exploring

#         # publish the message
#         pub.publish(msg)

#         # sleep to maintain the loop rate
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         task_handler()
#         target_pos_publisher()
#     except rospy.ROSInterruptException:
#         pass
