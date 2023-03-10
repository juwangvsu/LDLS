# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import numpy as np
import numpy
import pcl
import time
import pcl.pcl_visualization
import pandas as pd
import open3d as o3d
from lidar_segmentation.utils import CLASS_NAMES
# from pcl.pcl_registration import icp, gicp, icp_nl
visual=None

###############################################################################################
def save_labels(labels):
    print('label  type: ', type(labels), labels[1])
    df = pd.DataFrame(labels)
    data_fn = os.path.join('./', 'labels.csv')
    df.to_csv(data_fn, index=False)

###############################################################################################
# cloudi: pcl xyz, or xyzrgb, different show methods.
#save pcl pointcloud,x,y,z,rgb 4 column,rgb 24 bits
def save_cloud_csv(cloud):
    #cloud.from_array(lidar)
    #cloud.to_file("pcdoutput.pcd")
    pcl.save(cloud, "pcdfile.pcd")
    print('cloud type: ', type(cloud), cloud[1])
    cloud_np = cloud.to_array()
    print('cloud type: ', type(cloud_np), cloud_np[1])
    df = pd.DataFrame(cloud_np)
    data_fn = os.path.join('./', 'pcl_cloud.csv')
    df.to_csv(data_fn, index=False)

###############################################################################################
# input PointCloud
# return np array with nan filtered
def filter_nan(cloud):
    cloud_np = np.array(cloud) # better to use cloud.to_array() func
    cloud_np_valid = cloud_np[~np.isnan(cloud_np).any(axis=1), :]
    return cloud_np_valid

###############################################################################################
# display lidar data from LDLS pkg
# <class 'numpy.ndarray'> lidar.shape (96857, 3)
def displaylidar(lidar, title, class_labels=None, mode='xyzrgb', blocking=False):
    print('display lidar from LDLS format, class labels', type(lidar), lidar.shape, class_labels)
    cloud = pcl._pcl.PointCloud_PointXYZRGB()
    lidar = numpy.pad(lidar, ((0,0),(0,1)), mode='constant', constant_values=1)
    class_labels_str = np.array([CLASS_NAMES[i] for i in range(6)])
    print('classes 0,1...5 names: ', class_labels_str)
    print('classes 0, 1, 2, 3, 4, 5 points: ', sum(class_labels==0), sum(class_labels==1), sum(class_labels==2), sum(class_labels==3), sum(class_labels==4),sum(class_labels==5))
    class_labels[class_labels==0]=100
    class_labels[class_labels==1]=(200<<8) +150
    class_labels[class_labels==2]=200<<16
    class_labels[class_labels==3]=(200<<16)+(200<<8)
    lidar[:,-1]=class_labels
    cloud.from_array(lidar)

# this display with python-pcl
#    displaycloud(cloud, title, mode='xyzrgb', blocking=True, class_labels=class_labels)

# below display with open3d
    xyzrgb=cloud.to_array()
    pcolor_np=xyzrgb[:,3].astype(int)
    numpoints =len(pcolor_np)
    pcolor_np3=np.ones(numpoints*3)
    pcolor_np3 =pcolor_np3.reshape(numpoints,3)

    pcolor_np3[:,0] = np.right_shift(pcolor_np,16)/255
    pcolor_np3[:,1] = np.right_shift(np.bitwise_and(pcolor_np,0xffff),8)/255
    pcolor_np3[:,2] = np.bitwise_and(pcolor_np,0xff)/255

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyzrgb[:,0:3])
    pcd.colors = o3d.utility.Vector3dVector(pcolor_np3)
    o3d.io.write_point_cloud("pcdfile2.pcd", pcd)
    o3d.visualization.draw_geometries([pcd], zoom=0.0412,
                                  front=[-0.8757, -0.12125, 0.4795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[0.4694, -0.219768, 0.84624])
    #Ctrl C in o3d window copy view paramter to clipboard, paste here


###############################################################################################
# input 
# cloudi: pcl xyz, or xyzrgb, different show methods.
def displaycloud(cloud, title, mode='xyzrgb', blocking=False, class_labels=None):
    global visual
    print('cloud type: ', type(cloud), cloud[1])
    save_cloud_csv(cloud) # save to csv file for examine, pcl_cloud.csv
    if visual == None:
        visual = pcl.pcl_visualization.CloudViewing()
    if mode=='xyzrgb':
        if not isinstance(cloud, pcl._pcl.PointCloud_PointXYZRGB):
            cloudtmp_np = filter_nan(cloud)
            print('cloudtmp_np shape: ',cloudtmp_np.shape)
            cloudtmp_np = numpy.pad(cloudtmp_np, ((0,0),(0,1)), mode='constant', constant_values=1)
            if not class_labels is None:
                print('class lable: ', class_labels.shape, class_labels)
                cloudtmp_np[:,-1]=class_labels
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

###############################################################################################
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


###############################################################################################
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

###############################################################################################
def visualization_test():
    cloudfn1="data/apgdata/pcd/001145.pcd"
    cloud1 = pcl.load(cloudfn1)
    displaycloud(cloud1, 'nop', mode='xyzrgb', blocking=True)

###############################################################################################
if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    cloudfn1="data/apgdata/pcd/001145.pcd"
    cloud1 = pcl.load(cloudfn1)
    cloudfn2="/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/mapdata_7.pcd"
    displaycloud(cloud1, 'nop', mode='xyzrgb', blocking=True)
