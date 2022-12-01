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

# # Load input data
# 
# Load the following files:
# - Calibration data (relates the LiDAR and camera sensor coordinate frames)
# - Image
# - LiDAR point cloud

# Define file paths
#calib_path = Path("data/") / "kitti_demo" / "calib" / "000571.txt"
#image_path = Path("data/") / "kitti_demo" / "image_2" / "000571.png"
#lidar_path = Path("data/") / "kitti_demo" / "velodyne" / "000571.bin"

calib_path = Path("data/") / "_out" / "calib" / "00000262.txt"
image_path = Path("data/") / "_out" / "image_2" / "00000262.png"
lidar_path = Path("data/") / "_out" / "velodyne" / "00000262.bin"
# Load calibration data
projection = load_kitti_object_calib(calib_path)

# Load image
image = load_image(image_path)
image=image[:,:,:3]
skimage.io.imshow(image)
print(type(image), image.shape)
input('pause')
# Load lidar
lidar = load_kitti_lidar_data(lidar_path, load_reflectance=False)
print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])


# # Run Mask-RCNN detector on image
# 
# The first step in the LDLS pipeline is to run Mask-RCNN on the input image to generate 2D segmentation masks. The following code block runs Mask-RCNN and visualizes results on the input image.

# In[1]:

import cupy as cp
detector = MaskRCNNDetector()
detections = detector.detect(image)
t0=time.time()
for n in range(50):
  detections = detector.detect(image)
print(" mask rcnn dt = %0.3f ms"%((time.time()-t0)*1000.0/n) )

print('\n\n\n******** detection done****************\n\n\n')
detections.visualize(image)

print('\n\n\n******** maskrcnn gpu released ****************\n\n\n')
from numba import cuda
device = cuda.get_current_device()
device.reset()

print('\n\n\n******** visualize done****************\n\n\n')
# # Perform LiDAR segmentation
# 
# Next, perform 3D segmentation using a LidarSegmentation object. The LidarSegmentation.run() method takes as inputs a LiDAR point cloud, Mask-RCNN detections, and a maximum number of iterations parameter.

lidarseg = LidarSegmentation(projection)
# Be sure to set save_all=False when running segmentation
# If set to true, returns label diffusion results at each iteration in the results
# This is useful for analysis or visualizing the diffusion, but slow.

results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
t0=time.time()
for n in range(50):
  results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
print(" lidarseg dt =%0.3f ms"%((time.time()-t0)*1000.0/n) )


print('\n\n\n******** lidarseg.run done****************\n\n\n')


#get_ipython().run_line_magic('timeit', 'lidarseg.run(lidar, detections, max_iters=50, save_all=False)')


# Plot the resulting labeled pointcloud using [Plotly](https://plot.ly/). You can visualize the results with points colored according to class labels (Person, Car, ...), or instance labels (Person 1, Person 2, Car 1, ...).

from lidar_segmentation.plotting import plot_segmentation_result

# Show points colored by class label
plot_segmentation_result(results, label_type='class')

# Show points colored by instance label
plot_segmentation_result(results, label_type='instance')

print('\n\n\n******** plot_segmentation_result done****************\n\n\n')

# You can also visualize the label diffusion over time. This requires running the lidar segmentation with the `save_all` parameter set to `true` (note that this is significantly slower due to saving the full diffusion results in an array).
# 
# Run the following code block to visualize this. You can use the slide bar on the bottom to see results at different iterations.

from lidar_segmentation.plotting import plot_diffusion

results_all = lidarseg.run(lidar, detections, max_iters=50, save_all=True)
plot_diffusion(results_all)

results.class_labels()

results.instance_labels()

detections.class_ids






