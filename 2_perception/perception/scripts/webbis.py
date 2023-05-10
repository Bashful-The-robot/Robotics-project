#!/usr/bin/env python3

from pathlib import Path

import torch
import cv2

import utils
from detector import Detector

class object_detect:

    ROOT_PATH = Path(__file__).parent
    DEVICE = 'cuda'
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    THRESHOLD = 0.5

    def __init__(self):

        ## Neural Network
        self.detector = utils.load_model(Detector(),
                                         '/home/robot/BASHFUL_WS/src/2_perception/perception/det_2023-03-25_11-18-29-981589.pt',
                                         device=self.DEVICE,
                                         compiled=True)
        self.detector.eval()

        #self.cap = cv2.VideoCapture('/home/robot/BASHFUL_WS/src/2_perception/perception/2023-03-30-123931.webm')
        self.cap = cv2.VideoCapture(0)
        """
        ts = ApproximateTimeSynchronizer([Subscriber('/usb_cam/image_raw', Img_msg),
                              Subscriber('camera/depth/image_rect_raw', Img_msg),
                              Subscriber('usb_cam/camera_info', Cam_info)],10,1)
    """
    def run(self):
        key = 0
        is_captured, frame = self.cap.read()
        print(frame)
        while is_captured and key != ord('q'):
            self.callback(frame)
            is_captured, frame = self.cap.read()
            key = cv2.waitKey(10)

    def callback(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
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
        cv2.imshow('Detection', out_image)

if __name__ == '__main__':

    ##  Start node  ##

    object_detect().run()

