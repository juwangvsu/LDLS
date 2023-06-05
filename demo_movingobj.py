#!/usr/bin/env python
#two frames, calculate reletive velocity to seperate points.
#print('usage: python demo_movingobj.py --datapath apgdata --id 001145  --true_gt')
import numpy as np
from pathlib import Path
import skimage
from matplotlib import pyplot as plt
from lidar_segmentation.detections import MaskRCNNDetections
from lidar_segmentation.segmentation import LidarSegmentation
from lidar_segmentation.kitti_utils import load_kitti_lidar_data, load_kitti_object_calib
from lidar_segmentation.utils import load_image, expand_nparray
from lidar_segmentation.evaluation import LidarSegmentationGroundTruth, plot_range_vs_accuracy
from mask_rcnn.mask_rcnn import MaskRCNNDetector

from pandas import read_csv

#read the relative tf matrix from csv file, produced by match_savedpcds
rel_icp= read_csv('rel_csvlog.csv',skiprows=[0], header=None)
tr_mat=rel_icp.values[0,20:32].reshape(3,4) # 3x4 matrix for tr
x=tr_mat[0,3]
y=tr_mat[1,3]
z=tr_mat[2,3]
tr_m44=np.zeros((4,4))
tr_m44[0:3,:]=tr_mat
tr_m44[3,3]=1

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
print('usage: python demo.py --datapath kitti_object/training --id 000049 --true_gt') 
fnprefix= "_out"
fnprefix2= "00000262"

parser = argparse.ArgumentParser(
        description='demo lidar seg.')
parser.add_argument('--datapath', required=False,
                        default='kitti_demo',
                        metavar="/path/to/logs/")
parser.add_argument('--id', required=False,
                        default='kitti_demo',
                        metavar="/path/to/logs/")
parser.add_argument('--frames', required=False,
                        default=2,
                        metavar="/path/to/logs/")
parser.add_argument('--incr', required=False,
                        default=1,
                        metavar="/path/to/logs/")
parser.add_argument('--gen_gt', action='store_true') #default false
parser.add_argument('--true_gt', action='store_false')

args = parser.parse_args()
fnprefix = args.datapath
fnprefix2 = args.id
gen_gt = args.gen_gt
true_gt = args.true_gt
framenum = args.frames
incr = args.incr
arglen=len(sys.argv)
print(sys.argv)

calib_path = Path("data/") / fnprefix / "calib" / (fnprefix2+".txt")
image_path = Path("data/") / fnprefix / "image_2" / (fnprefix2+".png")
lidar_path = Path("data/") / fnprefix / "velodyne" / (fnprefix2+".bin")
gt_seg_path = Path("data/") / fnprefix / "gt_segmentation" / (fnprefix2+".txt")

# Load calibration data
projection = load_kitti_object_calib(calib_path)
image_l=[]
lidar_l=[]
# Load image
for i in range(framenum):
    image_l.append(load_image(image_path))
    image_l[i] =image_l[i][:,:,:3]
    skimage.io.imshow(image_l[i])
    plt.show()
    print(type(image_l[i]), image_l[i].shape)
# Load lidar
    lidar = load_kitti_lidar_data(lidar_path, load_reflectance=False)
    lidar_l.append(lidar)
    print("Loaded LiDAR point cloud with %d points" % lidar.shape[0])

# # Run Mask-RCNN detector on image

import cupy as cp
detector = MaskRCNNDetector()
detections = detector.detect(image_l[1])
t0=time.time()
print("mrcnn detection results: ", detections.shape, detections.class_ids,detections.class_names, detections.scores)
print('\n\n\n******** detection done****************\n\n\n')
detections.visualize(image_l[1])

print('\n\n\n******** maskrcnn gpu released ****************\n\n\n')
from numba import cuda
device = cuda.get_current_device()
device.reset()

print('\n\n\n******** visualize done****************\n\n\n')
# # Perform LiDAR segmentation
# 
# run() : detections.masks uxv labels (from mrcnn), lidar [n,:] n lidar points 
# output: detection result 
lidarseg = LidarSegmentation(projection)

# run : detections.masks uxv labels, lidar [n,:] n lidar points 
# output: detection result , results.points points.shape (8211, 3)
print("Loaded LiDAR point cloud with %d points" % lidar_l[1].shape[1])
results = lidarseg.run(lidar_l[1], detections, max_iters=50, save_all=False)
print("lid seg results: #points: ", len(results.points), " points.shape", results.points.shape)
resultpts_4n=np.ones((4, results.points.shape[0]))
resultpts_4n[0:3,:]=results.points.transpose()

print("lid seg results transposed: 6 samples: ", resultpts_4n[:,0:6])
resultpts_frame0 = np.matmul(tr_m44, resultpts_4n)
print("resultpts_4n.shape, resultpts_frame0.shape, lidar_l[0].shape", resultpts_4n.shape, resultpts_frame0.shape, lidar_l[0].shape)
# (4, 8211) (4, 8211) (32768, 3)

resultpts_frame0_nx3 = np.float32(resultpts_frame0.transpose()[:,0:3]) 

# superimpose mix lidar_l[0] and the transformed seg result of lidar_l[1]
lidar_mixed = np.concatenate((lidar_l[0],resultpts_frame0_nx3),axis=0)

lidar0_dummylabel=np.zeros((lidar_l[0].shape[0]),dtype='float32')
label_mixed= np.concatenate((lidar0_dummylabel, results.class_labels()),axis=0)

lidar_mixed0 = np.concatenate((lidar_l[0],results.points),axis=0) #lidar[0] + lidar[1] detected portion

print("lid seg results in frame0: ", resultpts_frame0.shape)
#resultpts_frame0.shape (4,8211)

#sys.exit(1)

t0=time.time()
#for n in range(2):
#  results = lidarseg.run(lidar, detections, max_iters=50, save_all=False)
#print(" lidarseg dt =%0.3f ms"%((time.time()-t0)*1000.0/n) )

instance_labels_full = expand_nparray(results.instance_labels(), results.in_camera_view)
class_labels_full = expand_nparray(results.class_labels(), results.in_camera_view)

lidar_seg_gt_fromresults = LidarSegmentationGroundTruth(instance_labels_full,None, class_labels_full)

if gen_gt: #default False
    lidar_seg_gt_fromresults.to_file(gt_seg_path)

#debugging hack: if have true gt or not
if true_gt: #default True
    lidar_seg_gt = LidarSegmentationGroundTruth.load_file(gt_seg_path)
    print("Loaded lidar seg gt ", len(lidar_seg_gt.class_labels))
    if len(lidar_seg_gt.class_labels) == len(results.in_camera_view):
        # the gt have the full lidar points, only pick the set that is visiable
        # the points in results already filtered with i_camera_view
        lidar_seg_gt.class_labels=lidar_seg_gt.class_labels[results.in_camera_view]
        lidar_seg_gt.instance_labels=lidar_seg_gt.instance_labels[results.in_camera_view]
    print('\n\n\n******** lidarseg.run done****************\n\n\n')
    print('\n\n\n******** plot accuracy range_scatter.eps ****************\n\n\n')
    plot_range_vs_accuracy([results], [lidar_seg_gt]) 
    #result in "range_scatter.eps"



from lidar_segmentation.plotting import plot_segmentation_result

# Show points colored by class label
if usepcl:
    import visualization
    print('results.points: ', results.points.shape, results.points[1])
    visualization.save_labels(results.class_labels()*300 + 50)
    visualization.displaylidar(results.points, 'ldls seg result', results.class_labels())
    lidar0_dummylabel=np.zeros((lidar_l[0].shape[0]))
    visualization.displaylidar(lidar_l[0], 'lidar frame  0 dummpy label', lidar0_dummylabel)
    visualization.displaylidar(lidar_mixed, 'mixed lidar frame  0 and lidar frame 1 corrected by relative pose', label_mixed)
    visualization.displaylidar(lidar_mixed0, 'mixed lidar frame  0 and lidar frame 1 ', label_mixed)
else:
    print('labels: ', results.class_labels())
    plot_segmentation_result(results, label_type='class')
# Show points colored by instance label
    plot_segmentation_result(results, label_type='instance')

print('\n\n\n******** plot_segmentation_result done****************\n\n\n')

sys.exit(1)
# Run the following code block to visualize this. You can use the slide bar on the bottom to see results at different iterations.

from lidar_segmentation.plotting import plot_diffusion

#results_all = lidarseg.run(lidar, detections, max_iters=50, save_all=True)
#plot_diffusion(results_all)

results.class_labels()

results.instance_labels()

detections.class_ids






