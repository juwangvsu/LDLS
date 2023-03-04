#!/usr/bin/env python
#print('usage: python demo.py apgdata 001145')
#print('usage: python demo.py kitti_object/training 000049')
import numpy as np
from pathlib import Path
import skimage

from lidar_segmentation.detections import MaskRCNNDetections
from lidar_segmentation.segmentation import LidarSegmentation
from lidar_segmentation.kitti_utils import load_kitti_lidar_data, load_kitti_object_calib
from lidar_segmentation.utils import load_image
from lidar_segmentation.evaluation import LidarSegmentationGroundTruth, plot_range_vs_accuracy
from mask_rcnn.mask_rcnn import MaskRCNNDetector

import time
import sys

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

#usepcl = False # True #change to false if pcl error and use the plotly display
usepcl = True # True #change to false if pcl error and use the plotly display
print('usage: python demo.py apgdata 001145')
fnprefix= "_out"
fnprefix2= "00000262"
arglen=len(sys.argv)
print(sys.argv)
if arglen>1:
    fnprefix = sys.argv[1]
if arglen>2:
    fnprefix2 = sys.argv[2]
# Define file paths
#calib_path = Path("data/") / "kitti_demo" / "calib" / "000571.txt"
#image_path = Path("data/") / "kitti_demo" / "image_2" / "000571.png"
#lidar_path = Path("data/") / "kitti_demo" / "velodyne" / "000571.bin"

calib_path = Path("data/") / fnprefix / "calib" / (fnprefix2+".txt")
image_path = Path("data/") / fnprefix / "image_2" / (fnprefix2+".png")
lidar_path = Path("data/") / fnprefix / "velodyne" / (fnprefix2+".bin")
gt_seg_path = Path("data/") / fnprefix / "gt_segmentation" / (fnprefix2+".txt")

# Load calibration data
projection = load_kitti_object_calib(calib_path)

# Load image
image = load_image(image_path)
image=image[:,:,:3]
skimage.io.imshow(image)
print(type(image), image.shape)
#input('pause')
# Load lidar
lidar = load_kitti_lidar_data(lidar_path, load_reflectance=False)
print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])

lidar_seg_gt = LidarSegmentationGroundTruth.load_file(gt_seg_path)
print("Loaded lidar seg gt ", len(lidar_seg_gt.class_labels))
# # Run Mask-RCNN detector on image
# 
# The first step in the LDLS pipeline is to run Mask-RCNN on the input image to generate 2D segmentation masks. The following code block runs Mask-RCNN and visualizes results on the input image.

# In[1]:

import cupy as cp
detector = MaskRCNNDetector()
detections = detector.detect(image)
t0=time.time()
for n in range(2):
  detections = detector.detect(image)
print(" mask rcnn dt = %0.3f ms"%((time.time()-t0)*1000.0/n) )
print("mrcnn detection results: ", detections.shape, detections.class_ids,detections.class_names, detections.scores)
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

print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])
results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
print("lid seg results: #points: ", len(results.points))
t0=time.time()
for n in range(2):
  results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
print(" lidarseg dt =%0.3f ms"%((time.time()-t0)*1000.0/n) )

lidar_seg_gt_fromresults = LidarSegmentationGroundTruth(results.instance_labels(),None, results.class_labels())
lidar_seg_gt_fromresults.to_file("/tmp/mygt.txt")

lidar_seg_gt.class_labels=lidar_seg_gt.class_labels[results.in_camera_view]
lidar_seg_gt.instance_labels=lidar_seg_gt.instance_labels[results.in_camera_view]
print('\n\n\n******** lidarseg.run done****************\n\n\n')
print('\n\n\n******** plot accuracy ****************\n\n\n')
plot_range_vs_accuracy([results], [lidar_seg_gt])

#get_ipython().run_line_magic('timeit', 'lidarseg.run(lidar, detections, max_iters=50, save_all=False)')


# Plot the resulting labeled pointcloud using [Plotly](https://plot.ly/). You can visualize the results with points colored according to class labels (Person, Car, ...), or instance labels (Person 1, Person 2, Car 1, ...).

from lidar_segmentation.plotting import plot_segmentation_result

# Show points colored by class label
if usepcl:
    import visualization
    print('results.points: ', results.points.shape, results.points[1])
#    print('labels: ', results.class_labels())
    visualization.save_labels(results.class_labels()*300 + 50)
    visualization.displaylidar(results.points, 'ldls seg result', results.class_labels())
   # visualization.displaylidar(lidar, 'ldls lidar')
#    visualization.visualization_test()
else:
    print('labels: ', results.class_labels())
    plot_segmentation_result(results, label_type='class')
# Show points colored by instance label
    plot_segmentation_result(results, label_type='instance')

print('\n\n\n******** plot_segmentation_result done****************\n\n\n')

# You can also visualize the label diffusion over time. This requires running the lidar segmentation with the `save_all` parameter set to `true` (note that this is significantly slower due to saving the full diffusion results in an array).
# 
# Run the following code block to visualize this. You can use the slide bar on the bottom to see results at different iterations.

from lidar_segmentation.plotting import plot_diffusion

#results_all = lidarseg.run(lidar, detections, max_iters=50, save_all=True)
#plot_diffusion(results_all)

results.class_labels()

results.instance_labels()

detections.class_ids






