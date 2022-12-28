#!/usr/bin/env python
import numpy as np
from pathlib import Path
import skimage
from skimage.io import imread
from mask_rcnn.mask_rcnn import MaskRCNNDetector

import time
import sys

print('usage: python mrcnn_det.py xxx.jpg [xxx.h5]')
fnprefix= "_out"
fnprefix2= "00000262"
arglen=len(sys.argv)
print(sys.argv)

h5path=None
if arglen>2:
    h5path = sys.argv[2]
if arglen>1:
    image_path = sys.argv[1]
# Define file paths
#calib_path = Path("data/") / "kitti_demo" / "calib" / "000571.txt"
#image_path = Path("data/") / "kitti_demo" / "image_2" / "000571.png"
#lidar_path = Path("data/") / "kitti_demo" / "velodyne" / "000571.bin"

# Load image
image = imread(image_path)
image=image[:,:,:3]
skimage.io.imshow(image)
print(type(image), image.shape)

import cupy as cp
detector = MaskRCNNDetector(h5path=h5path)
detections = detector.detect(image)

print('\n\n\n******** detection done****************\n\n\n')
detections.visualize(image)







