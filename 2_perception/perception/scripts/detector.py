"""Baseline detector model.

Inspired by
You only look once: Unified, real-time object detection, Redmon, 2016.
"""
from typing import List, Optional, Tuple, TypedDict

import numpy as np
import torch
import albumentations as A
import torch.nn as nn

from torch import Tensor
from PIL import Image

from torchvision import ops, models, transforms


class BoundingBox(TypedDict):
    """Bounding box dictionary.

    Attributes:
        x: Top-left corner column
        y: Top-left corner row
        width: Width of bounding box in pixel
        height: Height of bounding box in pixel
        score: Confidence score of bounding box.
        category_id: Category (not implemented yet!)
    """

    x: int
    y: int
    width: int
    height: int
    score: float
    category_id: int


class Detector(nn.Module):
    """Baseline module for object detection."""

    # 1280x15x20 -> 5x15x20, where each element 5 channel tuple corresponds to
    #   (rel_x_offset, rel_y_offset, rel_x_width, rel_y_height, confidence
    # Where rel_x_offset, rel_y_offset is relative offset from cell_center
    # Where rel_x_width, rel_y_width is relative to image size
    # Where confidence is predicted IOU * probability of object center in this cell
    OUT_CELLS_X = 20
    OUT_CELLS_Y = 15
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480

    def __init__(self) -> None:
        """Create the module.

        Define all trainable layers.
        """
        super(Detector, self).__init__()

        self.features = models.mobilenet_v2(weights='MobileNet_V2_Weights.DEFAULT').features
        # output of mobilenet_v2 will be 1280x15x20 for 480x640 input images

        self.head = nn.Conv2d(in_channels=1280, out_channels=13, kernel_size=1)
        # 1x1 Convolution to reduce channels to out_channels without changing H and W

    def forward(self, inp: Tensor) -> Tensor:
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

    @classmethod
    def decode_output(
        cls,
        out: Tensor,
        threshold: Optional[float] = None,
        topk: int = 100,
    ) -> List[List[BoundingBox]]:
        """Convert output to list of bounding boxes.

        Args:
            out (Tensor):
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
        out = out.cpu()

        # decode bounding boxes for each image

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

            scores = []
            bboxes = []

            # loop over all cells with bounding box center
            for bb_index in bb_indices:
                xn, yn, wn, hn = o[0:4, bb_index[0], bb_index[1]]
                category = torch.argmax(o[5:13, bb_index[0], bb_index[1]]).item()
                score = o[4, bb_index[0], bb_index[1]].item()

                # decode bounding box size and position
                width = cls.IMAGE_WIDTH * abs(wn.item())
                height = cls.IMAGE_HEIGHT * abs(hn.item())
                y = (
                    cls.IMAGE_HEIGHT / cls.OUT_CELLS_Y * (bb_index[0] + yn)
                    - height / 2.0
                ).item()
                x = (
                    cls.IMAGE_WIDTH / cls.OUT_CELLS_X * (bb_index[1] + xn)
                    - width / 2.0
                ).item()

                img_bbs.append(
                    {
                        "width": width,
                        "height": height,
                        "x": x,
                        "y": y,
                        "score": score,
                        "category_id": category, #self implemented 
                    }
                )

                bboxes.append([x, y, x+width, y+height])
                scores.append(score)

            if img_bbs:
                
                indices = ops.nms(Tensor(bboxes),
                                  Tensor(scores),
                                  0.15)
                
                img_bbs = [img_bbs[i] for i in indices]

            bbs.append(img_bbs)

        return bbs

    @classmethod
    def input_transform(cls, image: Image, anns: List, augment: bool = False) -> Tuple[Tensor, Tensor]:
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

        tfs = [
            A.Resize(height=cls.IMAGE_HEIGHT, width=cls.IMAGE_WIDTH),
        ]

        if augment:
            tfs += [
                A.HorizontalFlip(p=0.5),
                A.ISONoise(p=0.01), 
                A.ColorJitter(p=0.01),
                A.GaussNoise(var_limit=(10.0, 50.0), mean=0),
                A.RandomToneCurve(scale=0.5,p=1),
                A.PixelDropout(p=0.01), 
                A.augmentations.geometric.transforms.Affine(shear=[-45, 45],p=0.01),
            ]

        transform = A.Compose(tfs, bbox_params=A.BboxParams(format='coco'))

        image = np.asarray(image)
        bboxes = [
            [ann["bbox"][0], 
             ann["bbox"][1], 
             ann["bbox"][2], 
             ann["bbox"][3],
             ann["category_id"]]
            for ann in anns
        ]

        augmented = transform(image=image, bboxes=bboxes)

        image = augmented["image"]
        bboxes = augmented["bboxes"]

        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406], 
                std=[0.229, 0.224, 0.225]
            ),
        ])

        image = transform(image)

        target = torch.zeros(13, cls.OUT_CELLS_Y, cls.OUT_CELLS_X)
        for (x, y, w, h, cat) in bboxes:
            
            x_center = x + w / (2.0)
            y_center = y + h / (2.0)
            x_center_rel = x_center / (cls.IMAGE_WIDTH) * cls.OUT_CELLS_X
            y_center_rel = y_center / (cls.IMAGE_HEIGHT) * cls.OUT_CELLS_Y
            x_ind = int(x_center_rel)
            y_ind = int(y_center_rel)
            x_cell_pos = x_center_rel - x_ind
            y_cell_pos = y_center_rel - y_ind
            rel_width = w / (cls.IMAGE_WIDTH)
            rel_height = h / (cls.IMAGE_HEIGHT) 
            assert x_ind < cls.OUT_CELLS_X
            assert y_ind < cls.OUT_CELLS_Y
            
            target[0, y_ind, x_ind] = x_cell_pos
            target[1, y_ind, x_ind] = y_cell_pos
            target[2, y_ind, x_ind] = rel_width
            target[3, y_ind, x_ind] = rel_height
            target[4, y_ind, x_ind] = 1 
            target[5+cat, y_ind, x_ind] = 1
        
        return image, target
    