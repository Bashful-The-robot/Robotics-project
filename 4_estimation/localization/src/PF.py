#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion #, Pose
from sensor_msgs.msg import Imu
from aruco_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage

import tf_conversions
import tf2_ros

from tf.transformations import euler_from_quaternion, quaternion_from_euler


import matplotlib.pyplot as plt
import matplotlib.animation as animation

class localization:
    def __init__(self):
        #subscriber
        rospy.Subscriber('/imu/data', Imu , self.imu_callback)
        #rospy.Subscriber('/', #landmark, self.landmark_callback)
        rospy.Subscriber('/tf', TFMessage, self.odometry_callback)
        
        #variable-aruco
        self.marker_id = None
        self.marker_pos = None
        self.marker_ori = None
        self.new_measurement = False

        #variable-odometry
        self.update_odometry = False
        self.time_diff = 0.01
        self.transform_time_Stamp = None
        self.odometry_pos = [0,0,0]
        self.odometry_ori = [0,0,0]

        #avriable-twist
        self.motion_x = 0
        self.motion_y = 0
        self.motion_theta = 0

        #particle_filter_initialization
        self.number_of_particles = 50
        self.particles = np.zeros((self.number_of_particles, 4), dtype=np.float16)

        #variable-particle_filter
        self.Q = np.array([[0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.2]]) #measurement noice
        self.inv_Q = np.linalg.inv(self.Q)
        self.update_coefficient = 1/(2*np.pi*self.Q**(1/2))

        #broadcast_transform
        br = tf2_ros.TransformBroadcaster()

        #transform
        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "/state_estimation"
        t.header.stamp = self.transform_time_Stamp

#        while self.marker_id == None:
#            print("Waiting for the starting Aruco marker")
#            continue

        while self.update_odometry == False:
            continue

#        self.initialization()

        while not rospy.is_shutdown():
            if self.update_odometry:
                self.prediction()
                self.update_odometry = False
            
            self.update()
#            self.resample()
        
#    def initialization(self):
#        self.particles[:,0] = 
#        self.particles[:,1] = self.odometry_pos.y
#        self.particles[:,2] = self.odometry_ori[2]
#        self.particles[:,3] = 1/self.number_of_particles

    def prediction(self):
        self.particles[:,0] = self.odometry_pos[0] + self.motion_x/self.time_diff + np.random.normal(loc=0.0, scale=0.03, size=self.number_of_particles)
        self.particles[:,1] = self.odometry_pos[1] + self.motion_y/self.time_diff + np.random.normal(loc=0.0, scale=0.03, size=self.number_of_particles)
        self.particles[:,2] = self.odometry_ori[2] + self.motion_theta/self.time_diff + np.random.normal(loc=0.0, scale=0.03, size=self.number_of_particles)
        self.particles[:,2] = np.mod(self.particles[:,2]+np.pi,2*np.pi) - np.pi
        self.particles[:,3] = 1/self.number_of_particles
        #print(self.particles)
        #self.print_estimation()
        #check if the particles are within bound?

    def print_estimation(self):
        fig, ax = plt.subplots()
        for x in self.particles:
            ax.plot(x[0],x[1], 'o')

        plt.show()

    def update(self):
        #print("self.new_measurement", self.new_measurement)
        #if self.new_measurement:
        measurement = [0.06, -0.08, 0]
        temp_psi = np.zeros((self.number_of_particles, 1))
        innovation = np.zeros((self.number_of_particles, 2))
        innovation[:,0] = np.sqrt((self.particles[:,0]-measurement[0])**2 + (self.particles[:,1]-measurement[1])**2)
        innovation[:,1] = np.mod((np.atan2(map(2,j) - self.particcles[:,2], measurement - S(1,:)) - S(3,:)) + pi, 2*pi)-pi
        (self.particles[:,2]-measurement[2])
        for i in range(0, self.number_of_particles):
            temp_psi[i] = self.update_coefficient*np.exp((-1/2)*((innovation[i]*self.inv_Q*innovation[i]))/10000)
        self.particles[:,3] = temp_psi/np.sum(temp_psi)
    
        self.new_measurement = False
        print("-----------------------------------------------")
        print(self.particles)
        #measurement how far from the arcuo
        #particles how far from the aruco
        #closest win

    def resample(self):
        pass

    def imu_callback(self, msg):
#        self.motion_x = msg.linear_acceleration.x
#        self.motion_y = msg.linear_acceleration.y
#        self.motion_theta = msg.angular_velocity.z
        pass

    def landmark_callback(self, msg):
        self.new_measurement = True
        self.marker_id = msg.markers[0].id
        self.marker_pos = msg.markers[0].pose.pose.position
        self.marker_ori = msg.markers[0].pose.pose.orientation

    def odometry_callback(self, msg):
        if msg.transforms[0].child_frame_id == "base_link":
            self.update_odometry = True

            self.transform_time_Stamp = msg.transforms[0].header.stamp
            self.motion_x = self.odometry_pos[0] - msg.transforms[0].transform.translation.x
            self.motion_y = self.odometry_pos[1] - msg.transforms[0].transform.translation.y
            odom = euler_from_quaternion([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w])
            self.motion_theta = self.odometry_ori[2] - odom[2]

            self.odometry_pos = [msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z]
            self.odometry_ori = odom
            #print("odom", self.odometry_pos[0], self.odometry_pos[1], self.odometry_ori[2])

def main():
    rospy.init_node("localization hate me")
    loc = localization()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("BYEEEEEEEEE")

if __name__ == '__main__':
    main()
