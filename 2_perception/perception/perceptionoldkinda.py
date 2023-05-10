
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

import torch
import cv2

import utils
from detector import Detector

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


class BoundingBox(TypedDict):
    """Bounding box dictionary.

    Attributes:
        x: Top-left corner column
        y: Top-left corner row
        width: Width of bounding box in pixel
        height: Height of bounding box in pixel
        score: Confidence score of bounding box.
        category: Category (not implemented yet!)
    """

    x: int
    y: int
    width: int
    height: int
    score: float
    category: int


class Detectoooooor(nn.Module):
    """Baseline module for object detection."""

    def __init__(self) -> None:
        """Create the module.

        Define all trainable layers.
        """
        super(Detector, self).__init__()

        self.features = models.mobilenet_v2(pretrained=True).features
        # output of mobilenet_v2 will be 1280x15x20 for 480x640 input images

        self.head = nn.Conv2d(in_channels=1280, out_channels=13, kernel_size=1)
        # 1x1 Convolution to reduce channels to out_channels without changing H and W

        # 1280x15x20 -> 5x15x20, where each element 5 channel tuple corresponds to
        #   (rel_x_offset, rel_y_offset, rel_x_width, rel_y_height, confidence
        # Where rel_x_offset, rel_y_offset is relative offset from cell_center
        # Where rel_x_width, rel_y_width is relative to image size
        # Where confidence is predicted IOU * probability of object center in this cell
        self.out_cells_x = 20
        self.out_cells_y = 15
        self.img_height = 480
        self.img_width = 640

    def forward(self, inp: torch.Tensor) -> torch.Tensor:
        """Forward pass.

        Compute output of neural network from input.

        Args:
            inp: The input images. Shape (N, 3, H, W).

        Returns:
            The output tensor encoding the predicted bounding boxes.
            Shape (N, 5, self.out_cells_y, self.out_cells_y).
        """
        features = self.features(inp)
        out = self.head(features)  # Linear (i.e., no) activation

        return out



    def decode_output(
        self, out: torch.Tensor, threshold: Optional[float] = None, topk: int = 100
    ) -> List[List[BoundingBox]]:
        """Convert output to list of bounding boxes.

        Args:
            out (torch.tensor):
                The output tensor encoding the predicted bounding boxes.
                Shape (N, 5, self.out_cells_x, self.out_cells_y).
                The 5 channels encode in order:
                    - the x offset,
                    - the y offset,
                    - the width,
                    - the height,
                    - the confidence.
            threshold:
                The confidence threshold above which a bounding box will be accepted.
                If None, the topk bounding boxes will be returned.
            topk (int):
                Number of returned bounding boxes if threshold is None.

        Returns:
            List containing N lists of detected bounding boxes in the respective images.
        """
        bbs = []
        #out = torchvision.transforms.Resize(img_arr,h,w)
        out = out.cpu()
        #print("HERE")
        #print(out.shape)

        # decode bounding boxes for each image

        #for o in 
        #out[2] = out[2]/2
        #out[3] = out[3]/1.5

        for o in out:
            img_bbs = []

            # find cells with bounding box center
            if threshold is not None:
                bb_indices = torch.nonzero(o[4, :, :] >= threshold)
            else:
                _, flattened_indices = torch.topk(o[4, :, :].flatten(), topk)
                bb_indices = np.array(
                    np.unravel_index(flattened_indices.numpy(), o[4, :, :].shape)
                ).T

            # loop over all cells with bounding box center
            for bb_index in bb_indices:
                bb_coeffs = o[0:4, bb_index[0], bb_index[1]]
                #print(f"bb_index {bb_index}")
                #print(f"bb_coeffs {bb_coeffs}")
                category = torch.argmax(o[5:13, bb_index[0], bb_index[1]]).item()
                #print(f'category {category}') #HM prints out 0 on these 
                #print(bb_coeffs)

                # decode bounding box size and position
                width = self.img_width * abs(bb_coeffs[2].item())
                height = self.img_height * abs(bb_coeffs[3].item())
                y = (
                    self.img_height / self.out_cells_y * (bb_index[0] + bb_coeffs[1])
                    - height / 2.0
                ).item()
                x = (
                    self.img_width / self.out_cells_x * (bb_index[1] + bb_coeffs[0])
                    - width / 2.0
                ).item()

                img_bbs.append(
                    {
                        "width": width,
                        "height": height,
                        "x": x,
                        "y": y,
                        "score": o[4, bb_index[0], bb_index[1]].item(),
                        "category_id": category, #self implemented 
                    }
                )
            bbs.append(img_bbs)

        return bbs

    def input_transform(self, image: Image, anns: List) -> Tuple[torch.Tensor]:
        """Prepare image and targets on loading.

        This function is called before an image is added to a batch.
        Must be passed as transforms function to dataset.

        Args:
            image:
                The image loaded from the dataset.
            anns:
                List of annotations in COCO format.

        Returns:
            Tuple:
                image: The image. Shape (3, H, W).
                target:
                    The network target encoding the bounding box.
                    Shape (5, self.out_cells_y, self.out_cells_x).
        """
        # Convert PIL.Image to torch.Tensor


        scale_h = (720/self.img_height)
        scale_w = (1280/self.img_width)
        

        image_rere = torchvision.transforms.Resize((480,640))
        image = image_rere(image)
        image = transforms.ToTensor()(image)
        image = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )(image)

        # Convert bounding boxes to target format

        # First two channels contain relativ x and y offset of bounding box center
        # Channel 3 & 4 contain relative width and height, respectively
        # Last channel is 1 for cell with bounding box center and 0 without

        # If there is no bb, the first 4 channels will not influence the loss
        # -> can be any number (will be kept at 0)
        target = torch.zeros(13, self.out_cells_y, self.out_cells_x)
        for ann in anns:
            #print(f"ann {ann}")
            x = ann["bbox"][0]/scale_w
            y = ann["bbox"][1]/scale_h
            width = ann["bbox"][2]/scale_w
            height = ann["bbox"][3]/scale_h
            category = ann["category_id"]

            x_center = x + width / (2.0)
            y_center = y + height / (2.0)
            x_center_rel = x_center / (self.img_width) * self.out_cells_x
            y_center_rel = y_center / (self.img_height) * self.out_cells_y
            x_ind = int(x_center_rel)
            y_ind = int(y_center_rel)
            x_cell_pos = x_center_rel - x_ind
            y_cell_pos = y_center_rel - y_ind
            rel_width = width / (self.img_width)
            rel_height = height / (self.img_height) 
            assert x_ind < self.out_cells_x
            assert y_ind < self.out_cells_y
            #print(rel_height)
            #print("hej")
            #print(f"x_ind {x_ind}")
            #print(f"y_ind {y_ind}")
            #print(f"target shape {target.shape}")
            #print(f'image {image.shape}')
            
            # channels, rows (y cells), cols (x cells)
            target[4, y_ind, x_ind] = 1 #HÄR GÅR DET HELT ÅT SKOGEN
            #print("JAGSAHEJ")
            # bb size
            target[0, y_ind, x_ind] = x_cell_pos
            target[1, y_ind, x_ind] = y_cell_pos
            target[2, y_ind, x_ind] = rel_width
            target[3, y_ind, x_ind] = rel_height
            target[5+category, y_ind, x_ind] = 1

            #print("DRYGT")
        return image, target
    
    
class perception: 

    def __init__(self):
        rospy.init_node('perception')

        self.puben = rospy.Publisher('boundingboxes', Img_msg, queue_size=10)
        self.pub_pos = rospy.Publisher('pospub', PoseWithCovarianceStamped, queue_size=10)

        self.bridge = CvBridge()

        self.detector = Detector()
        state_dic = torch.load("/home/robot/BASHFUL_WS/src/2_perception/perception/det_2023-03-23_13-49-32-840392.pt")
        self.detector.load_state_dict(state_dic)

        self.detector = self.detector.to(device="cuda")
        #print(type(self.detector))

        self.Pin = PinholeCameraModel()

        #self.img = rospy.Subscriber('/camera/color/image_raw', Img_msg, self.image_callback)
        #depth KOLLA OM IMAGE_RAW OR IMAGE_RECT_RAW
        #self.dep = rospy.Subscriber('camera/depth/image_raw', Img_msg, self.depth_callback)
        #camera info KOLLA VERKLIGEN UPP DET HÄR
        #self.cam_inf = rospy.Subscriber('camera/depth/camera_info', Cam_info, self.camera_info_callback)
        
        #rospy.wait_for_message('/camera/color/image_raw', Img_msg)

        #CHANSNING
        ts = ApproximateTimeSynchronizer([Subscriber('/usb_cam/image_raw', Img_msg),
                              Subscriber('camera/depth/image_rect_raw', Img_msg),
                              Subscriber('usb_cam/camera_info', Cam_info)],10,1)
        """
        ts = ApproximateTimeSynchronizer([Subscriber('/camera/color/image_raw', Img_msg),
                              Subscriber('camera/depth/image_rect_raw', Img_msg),
                              Subscriber('camera/depth/camera_info', Cam_info)],10,1)
        
        ts.registerCallback(self.image_callback)
        """

        print("NEW IMAGE RECEIVED") 


    def image_callback(self, msg: Img_msg, msgd: Img_msg, msgi: Cam_info):
        #print("HERE")
        print(msg)
        self.Pin.fromCameraInfo(msgi)
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
        #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        cv_image = cv2.resize(cv_image, (640,480))
        #print("HERE2")
        imgage = torch.from_numpy(cv_image)
        imgage = imgage.to(device="cuda")
        imgage = imgage.float()

        #colors = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
        """lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        mimgage = cv2.inRange(colors, lower_blue, upper_blue)"""
        #BE CAREFULL TO CHANGE BACK

        imgage = torch.reshape(imgage, (1,3,480,640))
        
        out = self.detector(imgage)
        
        deco = self.detector.decode_output(out, threshold=0.6)[0]
        
        
        skor = 0
        
        cv_imaged = self.bridge.imgmsg_to_cv2(msgd, desired_encoding='passthrough')
        #imgage = torch.reshape(imgage, (1,3,480,640))
        cv_imaged = cv2.resize(cv_imaged, (640,480))
        depth = np.array(cv_imaged, dtype = np.float32)
        depth = np.reshape(depth, (480,640))
        
        #print(depth.mean())#DEPTH DISTANCE IS IN MILLIMETER
        depth = depth/1000 #DEPTH DISTANCE IS IN METER
        #print(depth.mean())
        """try:
            self.pub_pos.publish(self.bridge.cv2_to_imgmsg(depth, Pose))
        except CvBridgeError as e:
            print(f'NEMEN {e}')"""

        #depp = np.array(cv_imaged)
        cv_rec = cv_image
        #getting the function for the depth out
        #depp = self.img_dep()

        for bbs in deco:
            skor += bbs["score"] 
            #print("baranågot")
            #print(int(bbs["x"]-bbs["width"]//2))
            #print(int(bbs["y"]-bbs["height"]//2))
            
            u1 = int(bbs["x"])
            v1 = int(bbs["y"])
            u2 = int(bbs["x"]+bbs["width"])
            v2 = int(bbs["y"]+bbs["height"])
            
            cv_rec = cv2.rectangle(cv_rec, (u1, v1), (u2,v2) , (255, 255, 255), 4)
            #print(imgage.shape)
            ##imgage = imgage.numpy()

            #colors = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
            """lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
            mimgage = cv2.inRange(colors, lower_blue, upper_blue)"""

            #imgage = imgage.to(device="cpu")
            #a = np.logical_and(imgage[0,2,v1:v2,u1:u2].mean()<60, imgage[0,1,v1:v2,u1:u2].mean()<60)
            #b = np.logical_and(a, imgage[0,0,v1:v2,u1:u2].mean()>130)
            #print(colors[:, 2])
            #if b.any() == True:
            #print(f"SEEEE {b}")
            #print(colors.shape)
            #cv_rec = cv2.rectangle(cv_rec, (u1, v1), (u2,v2) , (255, 255, 255), 4)
            dlal = depth[v1:v2, u1:u2]
            dlal = dlal[~np.isnan(dlal)]
            d = dlal.mean() if dlal.size else 0
            #d = depth[(int(bbs["y"]+bbs["height"]//2), int(bbs["x"]+bbs["width"]//2))]
            u = bbs["x"]+bbs["width"]//2
            v = bbs["y"]+bbs["height"]//2
            cv_rec = cv2.putText(cv_rec, f'CATEGORY {bbs["category_id"]} X: {int(u)} Y: {int(v)} Z: {d}', (u1, v1-2), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)#u2//2, lineType=cv2.LINE_AA)
            

            if d == 0: 
                print("jajaj")
                continue

            #print(cv_rec)

            pos_msg = self.Pin.projectPixelTo3dRay((v,u)) #TODO, TEXT TO BOUNDING BOX ABOUT POS
            #print(pos_msg)
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.point.x = pos_msg[0]*d
            pm.point.y = pos_msg[1]*d
            pm.point.z = pos_msg[2]*d
            self.pub_pos.publish(pm)

            """pm = PoseWithCovarianceStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.pose.covariance = msg.pose.covariance 
            pm.pose.pose.position.x = pos_msg[0]*d
            pm.pose.pose.position.y = pos_msg[1]*d
            pm.pose.pose.position.z = pos_msg[2]*d
            pm.pose.pose.orientation.w = bbs["category_id"]
            self.pub_pos.publish(pm)"""
                
        skor = skor/len(deco) if deco else 0
        #if skor:
            #print(f'I SEE CUBE! confident score: {skor}')

        #if skor.any() == True:
        #    print("I SEE SOMETHING")

        #antagligen onödig
        self.puben.publish(self.bridge.cv2_to_imgmsg(cv_rec, "rgb8"))    
        """try:
            self.puben.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e) """

    #def run(self):
    #    while self.keep_alive(): 
    #        rospy.spin()
    
    #def keep_alive(self):
    #    return not rospy.is_shutdown()
                
"""
    def depth_callback(self, msg: Img_dep):

        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        imgage = torch.from_numpy(cv_image)
        imgage = imgage.to(device="cuda")
        imgage = imgage.float()
        
        imgage = torch.reshape(imgage, (1,3,480,640))
        
        depth = np.array(cv_image, dtype = np.float32)

        try:
            self.pub_pos.publish(self.bridge.cv2_to_imgmsg(depth, "rgb8"))
        except CvBridgeError as e:
            print(e)

        print("NEW IMG DEPTH RECEIVED")
    """

    #pub = None

    #class PLANB: 

    #def __init__(self):
    #    rospy.init_node('PLANB')
    #
    #    rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
    #    
    #    rospy.wait_for_message('/camera/depth/color/points', PointCloud2)
 
    #    print("NEW POINTCLOUD RECEIVED")



class object_detect:

    ROOT_PATH = Path(__file__).parent
    DEVICE = 'cuda'
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    THRESHOLD = 0.8

    def __init__(self):
        rospy.init_node('object_detect')

        self.puben = rospy.Publisher('boundingboxes', Img_msg, queue_size=10)
        self.pusben = rospy.Publisher('boundingboxes_usb', Img_msgu, queue_size=10)
        self.pub_pos = rospy.Publisher('pospub', PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/marker2', MarkerArray, queue_size=10)

        self.bridge = CvBridge()

        #self.detector = Detector()
        #state_dic = torch.load("/home/robot/BASHFUL_WS/src/2_perception/perception/det_2023-03-23_13-49-32-840392.pt")
        #self.detector.load_state_dict(state_dic)

        #self.detector = self.detector.to(device="cuda")
         #det_2023-03-25_11-18-29-981589.pt
        self.detector = utils.load_model(Detector(),
                                         '/home/robot/BASHFUL_WS/src/2_perception/perception/det_2023-04-01_12-49-43-223143.pt',
                                         device=self.DEVICE,
                                         compiled=True)
        self.detector.eval()


        self.Pin = PinholeCameraModel()

        tss = ApproximateTimeSynchronizer([Subscriber('/usb_cam/image_raw', Img_msgu),
                              Subscriber('usb_cam/camera_info', Cam_infou)],10,1)
        
        ts = ApproximateTimeSynchronizer([Subscriber('/camera/color/image_raw', Img_msg),
                              Subscriber('camera/depth/image_rect_raw', Img_msg),
                              Subscriber('camera/depth/camera_info', Cam_info)],10,1)
        
        ts.registerCallback(self.callback)
        #tss.registerCallback(self.callback_usb)


        print("NEW IMAGE RECEIVED") 

    """def runnnn(self):
        key = 0
        is_captured, frame = self.cap.read()
        while is_captured and key != ord('q'):
            self.callback(frame)
            is_captured, frame = self.cap.read()
            key = cv2.waitKey(10)"""
    

    def callback_usb(self, msg: Img_msgu, msgi:Cam_infou):
        self.Pin.fromCameraInfo(msgi)
        """im = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        cmprsmsg = self.bridge.cv2_to_compressed_imgmsg(im)  # Convert the image to a compress message
        cv_image = self.bridge.compressed_imgmsg_to_cv2(cmprsmsg)
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        cv_image = cv2.resize(cv_image, (640,480))
        
        imgage = torch.from_numpy(cv_image) / 255
        image = imgage.to(self.DEVICE)
        #imgage = imgage.float()

        #imgage = torch.reshape(imgage, (1,3,480,640))
        
        #out = self.detector(imgage)
        
        #deco = self.detector.decode_output(out, threshold=0.6)[0]
        
        image = image.permute(2, 0, 1)
        image = image[None, :, :, :] # add new dimension -> (1, C, H, W)

        out = self.detector(image)#.cpu()
        bbs = Detector.decode_output(out, self.THRESHOLD-0.5)[0]
        
        
        skor = 0

        """
        cv_imaged = self.bridge.imgmsg_to_cv2(msgd, desired_encoding='passthrough')
        #imgage = torch.reshape(image, (1,3,480,640))
        cv_imaged = cv2.resize(cv_imaged, (640,480))
        depth = np.array(cv_imaged, dtype = np.float32)
        depth = np.reshape(depth, (480,640))
        
        #print(depth.mean())#DEPTH DISTANCE IS IN MILLIMETER
        depth = depth/1000 #DEPTH DISTANCE IS IN METER
        """
        #cv_rec = cv_image
        out_image = cv_image
        
        for bb in bbs:
            skor += bb["score"] 
            ## Get box
            u1 = int(bb["x"])
            v1 = int(bb["y"])
            u2 = int(bb["x"]+bb["width"])
            v2 = int(bb["y"]+bb["height"])
            cat = bb["category_id"]

            ## Draw rectangle
            """
            dlal = depth[(v1+5):(v2-5), (u1+5):(u2-5)]
            dlal = dlal[~np.isnan(dlal)]
            d = dlal.mean() if dlal.size else 0"""
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
            """u = (bb["x"]+bb["width"]//2)*1.5
            v = bb["y"]+bb["height"]//2
            
            print(f'X:{u} Y:{v}')
            if d == 0: 
                print("No depth")
                continue

            #print(cv_rec)

            pos_msg = self.Pin.projectPixelTo3dRay((u,v)) #TODO, TEXT TO BOUNDING BOX ABOUT POS
            #print(pos_msg)
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.point.x = pos_msg[0]*d
            pm.point.y = pos_msg[1]*d
            pm.point.z = pos_msg[2]*(d*1.5)
            self.pub_pos.publish(pm)"""

        #out_image = cv2.cvtColor(out_image, cv2.COLOR_RGB2BGR)
        #cv2.imshow('Detection', out_image)
        self.pusben.publish(self.bridge.cv2_to_imgmsg(out_image, "rgb8"))  

        
        


    def callback(self, msg: Img_msg, msgd: Img_msg, msgi: Cam_info):

        #print(msg)
        self.Pin.fromCameraInfo(msgi)
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
        #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        cv_image = cv2.resize(cv_image, (640,480))
        
        imgage = torch.from_numpy(cv_image) / 255
        image = imgage.to(self.DEVICE)
        #imgage = imgage.float()

        #imgage = torch.reshape(imgage, (1,3,480,640))
        
        #out = self.detector(imgage)
        
        #deco = self.detector.decode_output(out, threshold=0.6)[0]
        
        image = image.permute(2, 0, 1)
        image = image[None, :, :, :] # add new dimension -> (1, C, H, W)

        out = self.detector(image)#.cpu()
        bbs = Detector.decode_output(out, self.THRESHOLD)[0]
        
        
        skor = 0

        
        cv_imaged = self.bridge.imgmsg_to_cv2(msgd, desired_encoding='passthrough')
        #imgage = torch.reshape(image, (1,3,480,640))
        cv_imaged = cv2.resize(cv_imaged, (640,480))
        depth_v = np.array(cv_imaged, dtype = np.float32)
        depth = np.reshape(depth_v, (480,640))
        
        #print(depth.mean())#DEPTH DISTANCE IS IN MILLIMETER
        depth = depth/1000 #DEPTH DISTANCE IS IN METER
        
        #cv_rec = cv_image
        out_image = cv_image
        
        for bb in bbs:
            skor += bb["score"] 
            ## Get box
            u1 = int(bb["x"])
            v1 = int(bb["y"])
            u2 = int(bb["x"]+bb["width"])
            v2 = int(bb["y"]+bb["height"])
            cat = bb["category_id"]

            ## Draw rectangle
            
            dlal = depth[(v1):(v2), (u1):(u2)]
            dlal = dlal[~np.isnan(dlal)]
            d = dlal.mean() if dlal.size else 0 #np.zeros_like(dlal)
            #mask = dlal < d
            #d = dlal[mask].mean()
            #print(f'DEPTH {d}')
            if d == 0 or np.isnan(d): 
                print("No depth")
                continue
            #if d > 1.5:
            #    continue
                        ##imgage = imgage.numpy()

            """colors = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV) 
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
            mimgage = cv2.inRange(colors, lower_blue, upper_blue)
            #cv2_bitvice_xor()
            """
            """res = cv.bitwise_and(frame,frame, mask= mimgage)
            cv.imshow('frame',frame)
            cv.imshow('mask',mask)
            cv.imshow('res',res)
            """
            """#print(cv_image)
            #imgage = imgage.to(device="cpu")
            a = np.logical_and(cv_image[v1:v2,u1:u2,0].mean()<250, cv_image[v1:v2,u1:u2,2].mean()<250)
            b = np.logical_and(a, cv_image[v1:v2,u1:u2,1].mean()>150)
            #print(colors[:, 2])
            if b.any() == False:
                #print(f"SEEEE {b}")
                #print(colors.shape)
                continue"""
            
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
            u = (bb["x"]+bb["width"]//2)*1.5
            v = bb["y"]+bb["height"]//2
            
            #print(f'X:{u} Y:{v}')
            
            #print(cv_rec)

            pos_msg = self.Pin.projectPixelTo3dRay((u,v)) #TODO, TEXT TO BOUNDING BOX ABOUT POS
            #print(pos_msg)
            new_pose = np.array([pos_msg[0]*d,pos_msg[1]*d, pos_msg[2]*(d*1.3)], dtype=float)
            self.rVizFunc(cat, new_pose)
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.point.x = pos_msg[0]*d
            pm.point.y = pos_msg[1]*d
            pm.point.z = pos_msg[2]*(d*1.5)
            self.pub_pos.publish(pm) 

        #out_image = cv2.cvtColor(out_image, cv2.COLOR_RGB2BGR)
        #cv2.imshow('Detection', out_image)
        self.puben.publish(self.bridge.cv2_to_imgmsg(out_image, "rgb8"))  

        
        
        
        #cv_imaged = self.bridge.imgmsg_to_cv2(msgd, desired_encoding='passthrough')
        ##imgage = torch.reshape(imgage, (1,3,480,640))
        #cv_imaged = cv2.resize(cv_imaged, (640,480))
        #depth = np.array(cv_imaged, dtype = np.float32)
        #depth = np.reshape(depth, (480,640))
        
        ##print(depth.mean())#DEPTH DISTANCE IS IN MILLIMETER
        #depth = depth/1000 #DEPTH DISTANCE IS IN METER
        
        #cv_rec = cv_image
        
        """for bbs in deco:
            skor += bbs["score"] 
            
            u1 = int(bbs["x"])
            v1 = int(bbs["y"])
            u2 = int(bbs["x"]+bbs["width"])
            v2 = int(bbs["y"]+bbs["height"])
            
            cv_rec = cv2.rectangle(cv_rec, (u1, v1), (u2,v2) , (255, 255, 255), 4)
            
            dlal = depth[v1:v2, u1:u2]
            dlal = dlal[~np.isnan(dlal)]
            d = dlal.mean() if dlal.size else 0
            #d = depth[(int(bbs["y"]+bbs["height"]//2), int(bbs["x"]+bbs["width"]//2))]
            u = bbs["x"]+bbs["width"]//2
            v = bbs["y"]+bbs["height"]//2
            cv_rec = cv2.putText(cv_rec, f'CATEGORY {bbs["category_id"]} X: {int(u)} Y: {int(v)} Z: {d}', (u1, v1-2), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)#u2//2, lineType=cv2.LINE_AA)
            

            if d == 0: 
                print("jajaj")
                continue

            #print(cv_rec)

            pos_msg = self.Pin.projectPixelTo3dRay((u,v)) #TODO, TEXT TO BOUNDING BOX ABOUT POS
            #print(pos_msg)
            pm = PointStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.point.x = pos_msg[0]*d
            pm.point.y = pos_msg[1]*d
            pm.point.z = pos_msg[2]*d
            self.pub_pos.publish(pm)"""

        """
            pm = PoseWithCovarianceStamped()
            pm.header.stamp = msg.header.stamp
            pm.header.frame_id = msg.header.frame_id #"camera_link" 
            pm.pose.covariance = msg.pose.covariance 
            pm.pose.pose.position.x = pos_msg[0]*d
            pm.pose.pose.position.y = pos_msg[1]*d
            pm.pose.pose.position.z = pos_msg[2]*d
            pm.pose.pose.orientation.w = bbs["category_id"]
            self.pub_pos.publish(pm)"""
                
        #skor = skor/len(deco) if deco else 0
        #if skor:
            #print(f'I SEE CUBE! confident score: {skor}')

        #if skor.any() == True:
        #    print("I SEE SOMETHING")

        #antagligen onödig
        #self.puben.publish(self.bridge.cv2_to_imgmsg(cv_rec, "rgb8"))    
        """try:
            self.puben.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e) """




    ##########

    """    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (640, 480))
        out_image = image

        image = torch.from_numpy(image) / 255
        image = image.to(self.DEVICE)
        image = image.permute(2, 0, 1)
        image = image[None, :, :, :] # add new dimension -> (1, C, H, W)

        out = self.detector(image).cpu()
        bbs = Detector.decode_output(out, self.THRESHOLD)[0]

        for bb in bbs:

            ## Get box
            u1 = int(bb["x"])
            v1 = int(bb["y"])
            u2 = int(bb["x"]+bb["width"])
            v2 = int(bb["y"]+bb["height"])
            cat = bb["category_id"]

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

        out_image = cv2.cvtColor(out_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Detection', out_image)"""
    
    def rVizFunc(self,cat, pos_msg):
        self.marker_array = MarkerArray()
        self.marker = Marker()
        self.marker.header.frame_id = "camera_color_optical_frame"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "object"
        self.marker.id = cat
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        if ~np.isnan(pos_msg).any():
            self.marker.pose.position.x = pos_msg[0]
            self.marker.pose.position.y = pos_msg[1]
            self.marker.pose.position.z = pos_msg[2]
            self.marker.pose.orientation.x = 0
            self.marker.pose.orientation.y = 0
            self.marker.pose.orientation.z = 0
            self.marker.pose.orientation.w = 1

            self.marker.scale.x = 0.05
            self.marker.scale.y = 0.05
            self.marker.scale.z = 0.05
            self.marker.color.a = 1.0
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 1.0
            self.marker_array.markers.append(self.marker)
            self.marker_pub.publish(self.marker_array)

    def run(self):
        while self.keep_alive(): 
            rospy.spin()
    
    """def run(self):
        key = 0
        is_captured, frame = self.cap.read()
        while is_captured and key != ord('q'):
            self.callback(frame)
            is_captured, frame = self.cap.read()
            key = cv2.waitKey(10)
    """
    def keep_alive(self):
        return not rospy.is_shutdown()
                

if __name__ == '__main__':

    ##  Start node  ##

    object_detect().run()

