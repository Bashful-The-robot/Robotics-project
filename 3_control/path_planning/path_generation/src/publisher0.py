#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

#This file publishes a -1 to the topic taskHandler and a position to the topic target_pos
#This will be used by the function main
###################


rospy.init_node('toDo')
MissionPub = rospy.Publisher('/pathMission', Int8, queue_size=10)
targetPub = rospy.Publisher('/target_pos', PoseStamped, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # create an Int8 message with a value of 42
    msgInt = Int8()
    msgInt.data = -1       #For exploring

    # create a PoseStamped message
    msgPose = PoseStamped()
    msgPose.pose.position.x = 0.5
    msgPose.pose.position.y = 0.5
    msgPose.pose.position.z = 0
    msgPose.pose.orientation.x = 0.0
    msgPose.pose.orientation.y = 0.0
    msgPose.pose.orientation.z = 0.0
    msgPose.pose.orientation.w = 0.0

    # publish the message
    MissionPub.publish(msgInt)
    targetPub.publish(msgPose)

    # sleep to maintain the loop rate
    rate.sleep()

# if __name__ == '__main__':
#     try:
#         task_handler()
#         target_pos_publisher()
#     except rospy.ROSInterruptException:
#         pass
