# -*- coding: utf-8 -*-
from __future__ import print_function

import numpy as np
import numpy
import pcl
import time
import pcl.pcl_visualization
# from pcl.pcl_registration import icp, gicp, icp_nl
visual=None
# input PointCloud
# return np array with nan filtered
def filter_nan(cloud):
    cloud_np = np.array(cloud)
    cloud_np_valid = cloud_np[~np.isnan(cloud_np).any(axis=1), :]
    return cloud_np_valid

# input cloud could be two type: xyz, or xyzrgb, different show methods.
def displaycloud(cloud, title, mode='xyzrgb', blocking=False):
    global visual
    if visual == None:
        visual = pcl.pcl_visualization.CloudViewing()
    if mode=='xyzrgb':
        if not isinstance(cloud, pcl._pcl.PointCloud_PointXYZRGB):
            cloudtmp_np = filter_nan(cloud)
            cloudtmp_np = numpy.pad(cloudtmp_np, ((0,0),(0,1)), mode='constant', constant_values=1)
            cloud = pcl._pcl.PointCloud_PointXYZRGB()
            cloud.from_array(cloudtmp_np)
        visual.ShowColorCloud(cloud, b'cloud')
    else:
        visual.ShowMonochromeCloud(cloud, b'cloud')

    if blocking:
      print('blocking ..... hit q to end at the gui')
      v = True
      while v:
        v = not(visual.WasStopped())
        #print('waiting')
        time.sleep(0.1)
      visual=None

# input PointCloud, not np array
def show2cloud(cloud1, cloud2, blocking=True):
    # filter out nan and expand to xyzrgb, color 1 and 26600
    cloud1_np_valid = filter_nan(cloud1)
    cloud2_np_valid = filter_nan(cloud2)
    cloud_xyzrgb_np1 = numpy.pad(cloud1_np_valid, ((0,0),(0,1)), mode='constant', constant_values=1)
    cloud_xyzrgb_np2 = numpy.pad(cloud2_np_valid, ((0,0),(0,1)), mode='constant', constant_values=26600)
    pc2 = pcl._pcl.PointCloud_PointXYZRGB()
    twocloud_np = np.concatenate((cloud_xyzrgb_np1, cloud_xyzrgb_np2), axis=0)
    pc2.from_array(twocloud_np)
    displaycloud(pc2, '2 cloud', 'xyzrgb', blocking=blocking)


def show2cloud_demo(cloudfn="/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/mapdata_37.pcd"):
    cloud = pcl.load(cloudfn)
    cloud_np = np.array(cloud)
    cloud_np_valid = cloud_np[~np.isnan(cloud_np).any(axis=1), :]
    cloud_np_valid_cen = cloud_np_valid-np.mean(cloud_np_valid, 0)
    cloud_xyzrgb_np1 = numpy.pad(cloud_np_valid, ((0,0),(0,1)), mode='constant', constant_values=1)
    cloud_xyzrgb_np2 = numpy.pad(cloud_np_valid_cen, ((0,0),(0,1)), mode='constant', constant_values=26600)

    #expand column to make point xyzrgb
    #cloud_xyzrgb_np = numpy.pad(cloud_np_valid, ((0,0),(0,1)), mode='constant', constant_values=1)
    pc2 = pcl._pcl.PointCloud_PointXYZRGB()
    #visual = pcl.pcl_visualization.CloudViewing()
    twocloud_np = np.concatenate((cloud_xyzrgb_np1, cloud_xyzrgb_np2), axis=0)

    pc2.from_array(twocloud_np)
    displaycloud(pc2,'nop', mode='xyzrgb', blocking=True)
#    visual.ShowMonochromeCloud(cloud, b'cloud2')
#    visual.ShowColorCloud(pc2, b'cloud')


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    show2cloud_demo()
    cloudfn1="/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/mapdata_37.pcd"
    cloud1 = pcl.load(cloudfn1)
    cloudfn2="/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/mapdata_7.pcd"
    cloud2 = pcl.load(cloudfn2)
    show2cloud(cloud1, cloud2)
    displaycloud(cloud1, 'nop', mode='xyzrgb', blocking=True)
