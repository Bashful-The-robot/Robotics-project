#!/usr/bin/env python3
import rospy
import tf
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
class AngleMapOdom:
    def __init__(self):
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.pub = rospy.Publisher('/angleMapOdom',PoseStamped,queue_size=10)
        self.angle = None
        self.time = None
        self.rate = rospy.Rate(10)



    def tf_callback(self,msg):

        if msg.transforms[0].header.frame_id == 'map':
            test=self.buffer.lookup_transform("map","odom",msg.transforms[0].header.stamp, rospy.Duration(3.0))             # might be wrong, maybe odom,map
            rot = [test.transform.rotation.x,test.transform.rotation.y,test.transform.rotation.z,test.transform.rotation.w]
            (roll,pitch,yaw) = euler_from_quaternion(rot)
            self.angle = yaw
            self.time = msg.transforms[0].header.stamp
        

    def publish(self):
        if self.angle!= None:
                pose = PoseStamped()
                pose.header.frame_id = "info in pose.position.x"
                pose.header.stamp = self.time
                pose.pose.position.x = self.angle
                pose.pose.position.y=0
                pose.pose.position.z=0
                self.pub.publish(pose)
                self.rate.sleep()

def main():
    rospy.init_node('angelMapOdom')
    angle = AngleMapOdom()
    while not rospy.is_shutdown():
        angle.publish()
    rospy.spin()


if __name__ == '__main__':
    main()