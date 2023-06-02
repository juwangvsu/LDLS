#!/usr/bin/env python
# calib parameter sensitivity evaluation. 
# first evaluation sensitivity along x/y/z translation, or  xr/yr/zr rot
# usage angle dithering: python demo_param_sen.py --datapath apgsample --id 001145 --xr --d_range 1 --steps 10 --gen_gt
#usage: python demo_param_sen.py --datapath apgsample --id 001145 --x --range 1 --steps 10
# this will assume gt point labels @ gt_segmentation,
# --x dithering along x axis, similarly --y --z
# --range  +- from the gt calib param
# --steps number of dithering data point in each selected parameter
import numpy as np
import quaternion # pip install numpy-quaternion
from pathlib import Path
import skimage

from lidar_segmentation.detections import MaskRCNNDetections
from lidar_segmentation.segmentation import LidarSegmentation
from lidar_segmentation.kitti_utils import load_kitti_lidar_data, load_kitti_object_calib
from lidar_segmentation.utils import load_image, expand_nparray
from lidar_segmentation.evaluation import LidarSegmentationGroundTruth, plot_range_vs_accuracy
from mask_rcnn.mask_rcnn import MaskRCNNDetector
from lidar_segmentation.plotting import plot_segmentation_result

import time
import sys
import argparse
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

parser = argparse.ArgumentParser(
        description='demo lidar seg.')
parser.add_argument('--datapath', required=False,
                        default='kitti_demo',
                        metavar="/path/to/logs/")
parser.add_argument('--outpath', required=False,
                        default='dithered',
                        metavar="/path/to/logs/")
parser.add_argument('--id', required=False,
                        default='kitti_demo',
                        metavar="/path/to/logs/")
parser.add_argument('--d_range', required=False,
                        default=1.0,
                        metavar="/path/to/logs/")
parser.add_argument('--steps', required=False,
                        default=10,
                        metavar="/path/to/logs/")
parser.add_argument('--gen_gt', action='store_true') #default false
parser.add_argument('--true_gt', action='store_false') #default false
parser.add_argument('--x', action='store_true') #default false
parser.add_argument('--y', action='store_true') #default false
parser.add_argument('--z', action='store_true') #default false
parser.add_argument('--xr', action='store_true') #default false x rotation
parser.add_argument('--yr', action='store_true') #default false y rotation
parser.add_argument('--zr', action='store_true') #default false z rotation
parser.add_argument('--pra', action='store_true') #default false, plot range acc

args = parser.parse_args()
fnprefix = args.datapath
fnprefix2 = args.id
gen_gt = args.gen_gt
true_gt = args.true_gt
arglen=len(sys.argv)
print(sys.argv)
#if arglen>1:
#    fnprefix = sys.argv[1]
#if arglen>2:
#    fnprefix2 = sys.argv[2]
# Define file paths
#calib_path = Path("data/") / "kitti_demo" / "calib" / "000571.txt"
#image_path = Path("data/") / "kitti_demo" / "image_2" / "000571.png"
#lidar_path = Path("data/") / "kitti_demo" / "velodyne" / "000571.bin"

calib_path = Path("data/") / fnprefix / "calib" / (fnprefix2+".txt")
image_path = Path("data/") / fnprefix / "image_2" / (fnprefix2+".png")
lidar_path = Path("data/") / fnprefix / "velodyne" / (fnprefix2+".bin")
gt_seg_path = Path("data/") / fnprefix / "gt_segmentation" / (fnprefix2+".txt")
#result_seg_path = Path("data/") / fnprefix / "result_segmentation" / (fnprefix2+".txt")
dither_seg_path = Path("data/") / fnprefix / "dithered" / (fnprefix2+".txt")

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
steps=int (args.steps)
d_range=float(args.d_range)
tf_mat = np.array(projection.transformation_matrix)
for i in range(steps):
    result_seg_path = Path("data/") / fnprefix / "result_segmentation" / (fnprefix2+"_"+str(i)+".txt")
    delta = (d_range/steps) * i - d_range/2
    print('calib transformation matrix: ', tf_mat)
    if args.x:
        projection.transformation_matrix[0,3] = tf_mat[0,3] + delta
    if args.xr or args.yr or args.zr:
        m1 = tf_mat[:3,:3]
        q1=quaternion.from_rotation_matrix(m1)
        v1=quaternion.as_euler_angles(q1) # alpha, beta, gamma, in z-y-z
        if args.zr:
            rd_vec = np.array([0,0, delta]) 
        if args.yr:
            rd_vec = np.array([0,delta,0]) 
        if args.xr:
            rd_vec = np.array([delta,0,0]) 
        q2=quaternion.from_rotation_vector(rd_vec) # rot vector NOT same as euler angles
        v2_recal=quaternion.as_euler_angles(q2)
        m2=quaternion.as_rotation_matrix(q2)
        projection.transformation_matrix[:3,:3] = m2.dot(projection.transformation_matrix[:3,:3]) # apply m2 afterward
        print("delta , euler angles : v1, v2_recal: ", delta, v1, v2_recal)
        # m1=quaternion.as_rotation_vector (q1)
        # quaternion.as_rotation_matrix(q4)
    print('calib transformation  d, m: ', delta, projection.transformation_matrix)
    lidarseg = LidarSegmentation(projection)
# Be sure to set save_all=False when running segmentation
# If set to true, returns label diffusion results at each iteration in the results
# This is useful for analysis or visualizing the diffusion, but slow.

    print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])
    results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
    print("lid seg results: #points: ", len(results.points))

    instance_labels_full = expand_nparray(results.instance_labels(), results.in_camera_view)
    class_labels_full = expand_nparray(results.class_labels(), results.in_camera_view)

    lidar_seg_gt_fromresults = LidarSegmentationGroundTruth(instance_labels_full,None, class_labels_full) ## class_labels int value, not string.

    if gen_gt: #default save the result label to files.
        lidar_seg_gt_fromresults.to_file(result_seg_path)

#debugging hack: if have true gt or not
    if true_gt: #default True
        lidar_seg_gt = LidarSegmentationGroundTruth.load_file(gt_seg_path)
        print("Loaded lidar seg gt ", len(lidar_seg_gt.class_labels))
        if len(lidar_seg_gt.class_labels) == len(results.in_camera_view):
        # the gt have the full lidar points, only pick the set that is visiable
        # the points in results already filtered with i_camera_view
            lidar_seg_gt.class_labels=lidar_seg_gt.class_labels[results.in_camera_view]
            lidar_seg_gt.instance_labels=lidar_seg_gt.instance_labels[results.in_camera_view]
        print('\n******** lidarseg.run done****************\n')
        print('\n******** plot accuracy ****************\n')
        plot_range_vs_accuracy([results], [lidar_seg_gt], pra=args.pra)

#get_ipython().run_line_magic('timeit', 'lidarseg.run(lidar, detections, max_iters=50, save_all=False)')


# Plot the resulting labeled pointcloud using [Plotly](https://plot.ly/). You can visualize the results with points colored according to class labels (Person, Car, ...), or instance labels (Person 1, Person 2, Car 1, ...).


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

    print('\n******** plot_segmentation_result done****************\n')

# You can also visualize the label diffusion over time. This requires running the lidar segmentation with the `save_all` parameter set to `true` (note that this is significantly slower due to saving the full diffusion results in an array).
# 
# Run the following code block to visualize this. You can use the slide bar on the bottom to see results at different iterations.

from lidar_segmentation.plotting import plot_diffusion

#results_all = lidarseg.run(lidar, detections, max_iters=50, save_all=True)
#plot_diffusion(results_all)

results.class_labels()

results.instance_labels()

detections.class_ids






