#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as Img_msg
from sensor_msgs.msg import CameraInfo as Cam_info
from sensor_msgs.msg import Image as Img_msgu
from sensor_msgs.msg import CameraInfo as Cam_infou
#from sensor_msgs.msg import Image as Img_msg
#from open3d import open3d as o3d
#from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
import numpy.linalg as LA

from pathlib import Path
import tf2_ros
import tf2_geometry_msgs
import torch
import cv2
from robp_msgs.msg import flags

import utils
from detector import Detector
from operator import itemgetter, attrgetter
from typing import List, Optional, Tuple, TypedDict

import numpy as np
import torch
import torch.nn as nn
from PIL import Image
from torchvision import models, transforms
import torchvision.transforms
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

from visualization_msgs.msg import Marker, MarkerArray

pub = None

"""Baseline detector model.
Inspired by
You only look once: Unified, real-time object detection, Redmon, 2016.
"""

def projectPixelTo3dRayUSB(camerainfo, uv):
    """
    :param uv:        rectified pixel coordinates
    :type uv:         (u, v)
    Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
    using the camera :math:`P` matrix.
    This is the inverse of :meth:`project3dToPixel`.
    """
    #fx = P(0,0), fy = P(1,1) cx = P(0,2) cy = P(1,2)

    
    
    P = np.array(camerainfo.P).reshape(3,4)
    # P = np.array([[517.03632655, 0, 312.03052029],
    #      [0, 516.70216219, 252.01727667],
    #      [0, 0, 1]])
    
    fx = P[0,0]
    fy = P[1,1]
    cx = P[0,2] 
    cy = P[1,2]

    x = (uv[0] - cx) / fx
    y = (uv[1] - cy) / fy
    norm = np.sqrt(x*x + y*y + 1)
    x /= norm
    y /= norm
    z = 1.0 / norm
    return (x, y, z)

def projectPixelTo3dRay(camerainfo, uv):
    """
    :param uv:        rectified pixel coordinates
    :type uv:         (u, v)
    Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
    using the camera :math:`P` matrix.
    This is the inverse of :meth:`project3dToPixel`.
    """
    #fx = P(0,0), fy = P(1,1) cx = P(0,2) cy = P(1,2)
    
    P = np.array(camerainfo.P).reshape(3,4)
    fx = P[0,0]
    fy = P[1,1]
    cx = P[0,2] 
    cy = P[1,2]

    x = (uv[0] - cx) / fx
    y = (uv[1] - cy) / fy
    norm = np.sqrt(x*x + y*y + 1)
    x /= norm
    y /= norm
    z = 1.0 / norm
    return (x, y, z)

class object_detect:

    ROOT_PATH = Path(__file__).parent
    DEVICE = 'cuda'
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    THRESHOLD = 0.8
    temp = []
    memory = []
    memorycat = []
    j = -1

    catdict = {0:"Binky",
               1:"Hugo",
               2:"Slush",
               3:"Muddles",
               4:"Kiki",
               5:"Oakie",
               6:"Cube",
               7:"Ball"}

    def __init__(self):
        rospy.init_node('object_detect')

        self.puben = rospy.Publisher('boundingboxes', Img_msg, queue_size=1)
        self.pusben = rospy.Publisher('boundingboxes_usb', Img_msgu, queue_size=1)
        self.pub_pos = rospy.Publisher('pospub', PointStamped, queue_size=1) #my own for seing the latest detection
        self.pusb = rospy.Publisher('arm_cam_pos', PointStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/marker2', MarkerArray, queue_size=1)

        self.bridge = CvBridge()

        self.detector = utils.load_model(Detector(),
                                         '/home/robot/BASHFUL_WS/src/2_perception/perception/det_2023-04-01_12-49-43-223143.pt',
                                         device=self.DEVICE,
                                         compiled=True)
        self.detector.eval()
        
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.Pin = PinholeCameraModel()

        flagss = rospy.Subscriber('/system_flags',flags, self.callback_flag)

        tss = ApproximateTimeSynchronizer([Subscriber('/usb_cam/image_raw', Img_msgu),
                              Subscriber('usb_cam/camera_info', Cam_infou)],1,1)
        
        ts = ApproximateTimeSynchronizer([Subscriber('/camera/color/image_raw', Img_msg),
                              Subscriber('camera/depth/image_rect_raw', Img_msg),
                              Subscriber('camera/depth/camera_info', Cam_info)],1,1)
         
        ts.registerCallback(self.callback)
        tss.registerCallback(self.callback_usb)
        
        print("NEW IMAGE RECEIVED") 

    def callback_flag(self, msg: flags):
        if msg.perception == True:
            for marker in self.marker_array:
                if msg.object_id == marker.id:
                    self.marker_array.pop(marker)
                    self.marker_pub.publish(self.marker_array)
                

    def callback_usb(self, msg: Img_msgu, msgi:Cam_infou):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
        cv_image = cv2.resize(cv_image, (640,480))
        
        imgage = torch.from_numpy(cv_image) / 255
        image = imgage.to(self.DEVICE)
        
        image = image.permute(2, 0, 1)
        image = image[None, :, :, :] # add new dimension -> (1, C, H, W)

        with torch.inference_mode():
            out = self.detector(image)#.cpu()
        bbs = Detector.decode_output(out, self.THRESHOLD-0.3)[0]
                
        skor = 0

        out_image = cv_image
        
        for bb in bbs:
            skor += bb["score"] 
            ## Get box
            u1 = int(bb["x"])
            v1 = int(bb["y"])
            u2 = int(bb["x"]+bb["width"])
            v2 = int(bb["y"]+bb["height"])
            cat = bb["category_id"]
            u = (bb["x"]+bb["width"]//2)
            v = bb["y"]+bb["height"]//2

            if v1 > 380:
                continue

            ## Draw rectangle
            out_image = cv2.rectangle(out_image,
                                      (u1, v1),
                                      (u2, v2),
                                      (255, 255, 255),
                                      4)
            out_image = cv2.putText(out_image,
                                    f'CATEGORY {cat} X: {u1} Y: {v1}',
                                    (u1, v1-2),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.75,
                                    (0,0,0),
                                    2)


            pos_msg = projectPixelTo3dRayUSB(msgi, (u,v)) #now in x,y,z relative the camera. 
            
            #len_from_cen = np.sqrt((pos_msg[0]*pos_msg[0])+(pos_msg[1]*pos_msg[1]))
            len_from_cen = 0
            dd = 0.22
            height = np.sqrt((dd*dd)+(len_from_cen*len_from_cen))
            
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = "base_link" #msg.header.frame_id #"camera_link" 
            pm.point.x = -pos_msg[1]*height
            pm.point.y = -pos_msg[0]*height 

            
            pm.point.z = 0

            print([cat, [pm.point.x, pm.point.y]])

            self.pusb.publish(pm)

        self.pusben.publish(self.bridge.cv2_to_imgmsg(out_image, "rgb8"))  

        
    
    def callback(self, msg: Img_msg, msgd: Img_msg, msgi: Cam_info):
        
        try: 
            mapView = self.buffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp)
        except: 
            return 

        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
        cv_image = cv2.resize(cv_image, (640,480))
        
        imgage = torch.from_numpy(cv_image) / 255
        image = imgage.to(self.DEVICE)
         
        image = image.permute(2, 0, 1)
        image = image[None, :, :, :] # add new dimension -> (1, C, H, W)
        
        with torch.inference_mode():
            out = self.detector(image)#.cpu()
        bbs = Detector.decode_output(out, self.THRESHOLD)[0]
        
        skor = 0
        
        cv_imaged = self.bridge.imgmsg_to_cv2(msgd, desired_encoding='passthrough')
        cv_imaged = cv2.resize(cv_imaged, (640,480))
        depth_v = np.array(cv_imaged, dtype = np.float32)
        depth = np.reshape(depth_v, (480,640))
        
        depth = depth/1000 #DEPTH DISTANCE IS IN METER
        depth = depth
        out_image = cv_image
        new_list = []

        for bb in bbs:

            skor += bb["score"] 
            ## Get box
            u1 = int(bb["x"])
            v1 = int(bb["y"])
            u2 = int(bb["x"]+bb["width"])
            v2 = int(bb["y"]+bb["height"])
            cat = bb["category_id"]
            u = (bb["x"]+bb["width"]//2)
            v = bb["y"]+bb["height"]//2

            dlal = depth[(v1):(v2-3), (u1):(u2-3)]
            dlal = dlal[~np.isnan(dlal)]
            d = dlal.mean() if dlal.size else 0 
            #print(f'DEPTH {cat}: {d}')

            if d < 0.01 or np.isnan(d): 
                print("No depth")
                continue
            if d > 1:
                continue

            if self.j == 20:
                self.j = -1
                print('trying to restart list')  
            
            
            ## Draw rectangle
            out_image = cv2.rectangle(out_image,
                                      (u1, v1),
                                      (u2, v2),
                                      (255, 255, 255),
                                      4)
            out_image = cv2.putText(out_image,
                                    f'CATEGORY {cat} X: {u1} Y: {v1} Z:{d}',
                                    (u1, v1-2),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.75,
                                    (0,0,0),
                                    2)
            
            self.j = self.j+1
            print([u, v, d])
            d = (d+0.23)
            pos_msg = projectPixelTo3dRay(msgi, (u,v)) #now in x,y,z relative the camera. 
            #print(pos_msg)
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.point.x = pos_msg[0]*d+0.15
            pm.point.y = pos_msg[1]*d
            pm.point.z = pos_msg[2]*d

            
            do_trans = tf2_geometry_msgs.do_transform_point(pm, mapView)   
            x = do_trans.point.x
            y = do_trans.point.y
            z = 0 #do_trans.point.z

            if len(self.memory)>0: #len(self.memorycat)>0:
                for mim in range(len(self.memory)):
                    if (cat == self.memory[mim][0]): 
                        if (self.memory[mim][1][0] - x < 0.1) and (self.memory[mim][1][1] - y < 0.1):
                            continue 

            self.temp.insert(self.j,[cat, x, y, z, 0, self.j]) 

            if len(self.temp) > 20:
                self.temp.pop(self.j)        
            
            tempsort = sorted(self.temp,key=itemgetter(0)) 
            
            if len(tempsort) < 1:
                continue

            #go through list of sorted detected objects
            tempsort = self.condition_filter(tempsort, out_image)

            #self.memory.append([cat, [x,y,z]])
            self.pub_pos.publish(pm)

            self.rVizFunc(self.memory)

        self.puben.publish(self.bridge.cv2_to_imgmsg(out_image, "rgb8"))  

    def condition_filter(self, tempsort, out_image):
        for i in range(len(tempsort)):
                if i == 0:
                    continue
                
                #filter if categories are the same, look if x,y and then z is close enough. 
                # if tempsort[i][0] in self.memorycat:
                #     continue
                if tempsort[i][0] == tempsort[i-1][0]:
                    if np.logical_or(abs(tempsort[i][1] - tempsort[i-1][1])>1,
                                        abs(tempsort[i][2] - tempsort[i-1][2])>1):
                        continue

                    tempsort = self.filter(tempsort, i, out_image)
                    
                    if len(tempsort)>=i:
                        self.j = len(tempsort)
                        break #hm, this might be very bad
  
                else: 
                    continue        

    
    def filter(self, tempsort, i, out_image):
        temp_new_x = ((tempsort[i][1]-tempsort[i-1][1])/2)+tempsort[i-1][1]
        temp_new_y = ((tempsort[i][2]-tempsort[i-1][2])/2)+tempsort[i-1][2]
        temp_new_z = ((tempsort[i][3]-tempsort[i-1][3])/2)+tempsort[i-1][3]
        a = []
        
        temp_new_cat = tempsort[i][0]
        
        if len(a)> 0:
            if temp_new_cat == a[0]:
                print("same again?")
                return tempsort

        print("STILL same again???")
        tempsort[i-1] = [temp_new_cat,temp_new_x,temp_new_y,temp_new_z, max(tempsort[i][4], tempsort[i-1][4])+1,max(tempsort[i][5],tempsort[i-1][5])]
        
        tempsort.pop(i)

        self.temp = sorted(tempsort, key=itemgetter(5))
        print("STEP 1 DONE")

        if i < len(tempsort):     
            if tempsort[i][0] == tempsort[i-1][0]:
                if np.logical_or(abs(tempsort[i][1] - tempsort[i-1][1])>0.7,
                                abs(tempsort[i][2] - tempsort[i-1][2])>0.7):
                    return tempsort
                
            temp_new_x = ((tempsort[i][1]-tempsort[i-1][1])/2)+tempsort[i-1][1]
            temp_new_y = ((tempsort[i][2]-tempsort[i-1][2])/2)+tempsort[i-1][2]
            temp_new_cat = tempsort[i][0]
            temp_new_z = ((tempsort[i][3]-tempsort[i-1][3])/2)+tempsort[i-1][3]
            tempsort[i-1] = [temp_new_cat,temp_new_x,temp_new_y,temp_new_z, max(tempsort[i][4], tempsort[i-1][4])+1,max(tempsort[i][5],tempsort[i-1][5])]
            #tempsort.pop(i)
            self.temp = sorted(tempsort, key=itemgetter(5))
            print("STEP 2 DONE")  
            
        print(i)
        print(f'a new match? {tempsort[i-1][4]}')

        if tempsort[i-1][4] > 50:
            self.temp = []
            i = 0
            self.j = 0 
            tempsort.pop(i-1)
            return tempsort
        
        if tempsort[i-1][4] >= 4 and tempsort[i-1][4] < 6:
            print("OMG A SURE MATCH")

            if np.isnan(tempsort[i-1][3]):
                return tempsort
            
            if tempsort[i-1][0] in self.memorycat:    
                for mim in range(len(self.memory)):
                    if (tempsort[i-1][0] == self.memory[mim][0]):
                        
                        z = tempsort[i-1][3]
                        x = tempsort[i-1][1]
                        y = tempsort[i-1][2]
                        cat = tempsort[i-1][0]

                        if np.isnan(x) or np.isnan(self.memory[mim][1][0]):
                            return tempsort
                        
                        if np.logical_or(abs(self.memory[mim][1][0] - x)<0.04,
                                        abs(self.memory[mim][1][1] - y)<0.04):
                            print("already detected at this x,y")
                            a = [cat]
                            break
                     
                        else:
                            print("new of same category") #if more than 2???????
                            cv2.imwrite("{}_1.png".format(self.catdict[cat]), out_image)
                        
                            self.memory.append([cat, [x,y,z]])

                            self.memorycat.append(cat)
                            return tempsort
            else:
                x = tempsort[i-1][1]
                y = tempsort[i-1][2]
                z = tempsort[i-1][3]
                cat = tempsort[i-1][0]

                #pos_msg = self.Pin.projectPixelTo3dRay((u,v)) #for map 

                catname = None
                if cat >= 6:
                    catname = self.catdict[cat]
                    col = self.process_color(out_image)

                    catname = catname + col

                else: 
                    catname = self.catdict[cat]
                    
                cv2.imwrite("{}_0.png".format(catname), cv2.cvtColor(out_image, cv2.COLOR_RGB2BGR))

                #new_pos = np.array([pos_msg[0]*d,pos_msg[1]*d, pos_msg[2]*(d*1.5)], dtype=float)

                self.memory.append([cat, [x,y,z]])
                self.memorycat.append(cat)
                tempsort.pop(i-1)
                
        return tempsort
    
    def process_color(self, out_image):
        hsv = cv2.cvtColor(out_image, cv2.COLOR_RGB2HSV) #wrong
        print(f'THE COLOR ARRAY {hsv}')
        min_green = np.array([40,50,50]) 
        max_green = np.array([70,255,255]) 
        min_red = np.array([120,50,70]) 
        max_red = np.array([180,255,255]) 
        min_blue = np.array([70,50,50]) 
        max_blue = np.array([120,255,255]) 
        
        mask_g = cv2.inRange(hsv, min_green, max_green) 
        mask_r = cv2.inRange(hsv, min_red, max_red) 
        mask_b = cv2.inRange(hsv, min_blue, max_blue) 

        res_b = cv2.bitwise_and(out_image, out_image, mask= mask_b) 
        res_g = cv2.bitwise_and(out_image,out_image, mask= mask_g) 
        res_r = cv2.bitwise_and(out_image,out_image, mask= mask_r)
        
        if res_b.any() == True:
            return "blue"
        elif res_g.any() == True:
            return "green"
        elif res_r.any() == True:
            return "red"
        else:
            return "beige"
    
    def rVizFunc(self, new_list):
        self.marker_array = MarkerArray()
        
        print(f'GOING IN??? {new_list}')
        
        marklist = []
        i = 0 

        for cat, pos_msg in new_list: 
            print(cat)
            #to handle if we have more of the same object
            if cat in marklist:
                cat = 8 + cat 
            if ~np.isnan(pos_msg).any():
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "object"
                marker.id = cat
                marker.pose.position.x = pos_msg[0]
                marker.pose.position.y = pos_msg[1]
                marker.pose.position.z = pos_msg[2]
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1

                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                
                if cat%8 == 6:
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                elif cat%8 == 7:
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD

                else:
                    marker.type = Marker.CYLINDER
                    marker.action = Marker.ADD

                self.marker_array.markers.append(marker)

                marklist.append([cat,i])

        self.marker_pub.publish(self.marker_array)

    def run(self):
        while self.keep_alive(): 
            rospy.spin()
    
    def keep_alive(self):
        return not rospy.is_shutdown()
                

if __name__ == '__main__':

    ##  Start node  ##

    object_detect().run()
