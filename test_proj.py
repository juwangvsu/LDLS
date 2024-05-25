#!/usr/bin/env python
import numpy as np
from pathlib import Path
import skimage

from lidar_segmentation.detections import MaskRCNNDetections
from lidar_segmentation.segmentation import LidarSegmentation
from lidar_segmentation.kitti_utils import load_kitti_lidar_data, load_kitti_object_calib
from lidar_segmentation.utils import load_image
from mask_rcnn.mask_rcnn import MaskRCNNDetector

import time
import sys
#import mrcnn
usepcl = False #change to false if pcl error and use the plotly display

print('usage: python test_proj.py apgdata 001145')
fnprefix= "_out"
fnprefix2= "00000262"
arglen=len(sys.argv)
print(sys.argv)
if arglen>1:
    fnprefix = sys.argv[1]
if arglen>2:
    fnprefix2 = sys.argv[2]

calib_path = Path("data/") / fnprefix / "calib" / (fnprefix2+".txt")
image_path = Path("data/") / fnprefix / "image_2" / (fnprefix2+".png")
lidar_path = Path("data/") / fnprefix / "velodyne" / (fnprefix2+".bin")

# Load calibration data
projection = load_kitti_object_calib(calib_path)

# Load image
image = load_image(image_path)
image=image[:,:,:3]
skimage.io.imshow(image)
print(type(image), image.shape)
# Load lidar
lidar = load_kitti_lidar_data(lidar_path, load_reflectance=False)
print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])

#mrcnn.visualize.display_images(image)
#input('pause')

# Next, perform 3D segmentation using a LidarSegmentation object. The LidarSegmentation.run() method takes as inputs a LiDAR point cloud, Mask-RCNN detections, and a maximum number of iterations parameter.

#lidarseg = LidarSegmentation(projection)
# Be sure to set save_all=False when running segmentation
# If set to true, returns label diffusion results at each iteration in the results
# This is useful for analysis or visualizing the diffusion, but slow.

#results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
from lidar_segmentation.plotting import plot_segmentation_result
from lidar_segmentation.plotting import plot_points
if usepcl:
    import visualization
#    visualization.visualization_test()
    visualization.displaylidar(lidar, 'ldls lidar')
else:
    plot_points(lidar)

# Show points colored by instance label
#plot_segmentation_result(results, label_type='instance')

print('\n\n\n******** plot_segmentation_result done****************\n\n\n')

from lidar_segmentation.plotting import plot_diffusion







