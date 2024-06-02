#!/usr/bin/env python
# listen to /pt2 and save the point cloud to tttt.pcd, only save the first one
# ros2 version
# usage: python baglistener_pt2pcd_ros2.py
# miniconda

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
import numpy
import numpy as np
import random
import csv
import copy
from threading import Thread, Lock
import open3d as o3d
#print("Load a ply point cloud, print it, and render it")
#pcd = o3d.io.read_point_cloud("001145.pcd")

mutex = Lock()
# turtlebot point msg ex:
# header:
#  frame_id: "camera_depth_optical_frame"
# height: 240 
# width: 320 
# fields" x,y,z, rgb
# point_step: 32
# row_step: 10240 #  == 32*320
# is_bigendian: False
# data: [235 ...] bytes, in py2.7 it is a string.
#   ex: data[0:32] first data point. data[0:4] x, data[4:8] y, data[8:12] z
#       data[16:20] rgb, the rest are padded 0s

#test bytes to int, < indicates the endianess is little-endian
import struct

fdigis=6

# keep points if y<0.1. kind of filter out the floor points
def filter_y(pc3array):
    return pc3array[pc3array[:,1]<0.1,:]

def filter_nan(cloud):
    cloud_np = np.array(cloud)
    cloud_np_valid = cloud_np[~np.isnan(cloud_np).any(axis=1), :]
    return cloud_np_valid

#convert ros pt2 data 
#input: ros PointCloud2, output PCD file, and return numpy array of points
#return: filtered numpy ndarray (:,3), and string in global pt2buf
# pt2buf is string, pt2buf_bytearray is bytearray , bzero32 is list of 32 0
# string can not be modified, so convert to bytearray and convert back to string
# the published data still have the same number of points, with some points 
# modified (such as floor become 0s). but the pcd have much
# smaller data points.
def pt2_2_pcd(pt2msg, pcdfn='pcdfile2.pcd'):
    global pt2buf, pt2buf_bytearray,bzero32
    pt2buf_bytearray = bytearray(pt2msg.data)
    pointsarray = numpy.ndarray((pt2msg.height*pt2msg.width,3), dtype='float32')
    for i in range(pt2msg.height):
        for j in range(pt2msg.width):
            pointoff = i*pt2msg.row_step + j * pt2msg.point_step
            xoff= pointoff
            yoff= pointoff + 4
            zoff= pointoff + 8
            rgboff= pointoff + 16
            xstr = pt2msg.data[xoff: xoff+4]
            ystr = pt2msg.data[yoff: yoff+4]
            zstr = pt2msg.data[zoff: zoff+4]
            x = struct.unpack('<f', xstr)[0]
            y = struct.unpack('<f', ystr)[0]
            z = struct.unpack('<f', zstr)[0]

            pointsarray[i*pt2msg.width+j,0]=x
            pointsarray[i*pt2msg.width+j,1]=y
            pointsarray[i*pt2msg.width+j,2]=z
            if  y>0.1:
                pt2buf_bytearray[pointoff:pointoff+32]=bzero32
    pt2buf = str(pt2buf_bytearray)
    print(pointsarray) 
    ptarray_valid=filter_nan(pointsarray)
    #ptarray_y=filter_y(ptarray_valid)
    ptarray_y=pointsarray

    numpoints =len(ptarray_y)
    pcolor_np3=np.ones(numpoints*3)
    pcolor_np3 =pcolor_np3.reshape(numpoints,3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(ptarray_y[:,0:3])
    pcd.colors = o3d.utility.Vector3dVector(pcolor_np3)
    print(pcd.points, pcd.colors)
    o3d.io.write_point_cloud(pcdfn, pcd)

    #pcl.save(pc2,'tttt.pcd') pcl sucks
    return ptarray_y

def callback_pt2(pt2msg):
   global pt2pub, msgt1_prev, pt2frames, letters, pt2random
   msgt1 =pt2msg.header.stamp.sec + 1.0*pt2msg.header.stamp.nanosec/1000000000
   print('t1, t1_prev len(data): ', msgt1, msgt1_prev, len(pt2msg.data))
   if msgt1 < msgt1_prev:
       #print('time glitch')
       return
   if len(pt2msg.data) > 32: #save one here
       print("pub this pt2 frame time",msgt1," len(pt2msg.data)/32:", len(pt2msg.data)/32)
       msgt1_prev=msgt1
       #print(pt2msg)
       #print(struct.unpack("<B", pt2msg.data[0]))
       #print(struct.unpack("<B", pt2msg.data[1]))
       b4=pt2msg.data[0]+pt2msg.data[1]+pt2msg.data[2]+pt2msg.data[3]
       #print(struct.unpack('<f', b4))
       pcdfn ='pcdfile_'+str(pt2frames)+'.pcd' 
       pt_filtered = pt2_2_pcd(pt2msg, pcdfn=pcdfn)
       #pt2msg.data=pt2msg.data[0:10240] #carto node crash
       #pt2msg.data=pt2msg.data[0:2457600/2] #no display, but carto node not crash
       #letters = ''.join(struct.pack("<B",i) for i in range(255))
       #pt2random = ''.join(random.choice(letters) for i in range(2457600)) 
       #pt2msg.data=pt2random
       #pt2msg.data=pt2buf

   #    print("pub this pt2 frame len(pt2msg.data)/32:", len(pt2msg.data)/32)
       #pt2pub.publish(pt2msg)
   pt2frames = pt2frames +1

def callback(imumsg):
   global imupub, msgt2_prev 
   msgt2 =imumsg.header.stamp.secs + 1.0*imumsg.header.stamp.nsecs/1000000000
   if msgt2 < msgt2_prev:
       print('time glitch')
       return
   msgt2_prev=msgt2
   imumsg.header.frame_id='imu_link'
   print('imu tm:', imumsg.header.stamp.secs,imumsg.header.stamp.nsecs)
   imupub.publish(imumsg)

# save the current odom to csv file
def saveodom():
    global odomsub, outfile, prevmsg_t, writer, prevodom
    writer.writerow({'t': prevmsg_t, 'type':'odom',
        'x': round(prevodom.pose.pose.position.x,fdigis),
        'y': round(prevodom.pose.pose.position.y,fdigis),
        'z': round(prevodom.pose.pose.position.z,fdigis),
        'qx':round(prevodom.pose.pose.orientation.x,fdigis),
        'qy':round(prevodom.pose.pose.orientation.y,fdigis),
        'qz':round(prevodom.pose.pose.orientation.z,fdigis),
        'qw':round(prevodom.pose.pose.orientation.w, fdigis)})
    outfile.flush()
    print('odom msg time: ', prevmsg_t, 'odom.x', prevodom.pose.pose.position.x, prevodom.pose.pose.position.y)

def callback3(odommsg):
    global odomsub, outfile, prevmsg_t, writer, prevodom, initodom 
    msg_t = odommsg.header.stamp.secs + 1.0*odommsg.header.stamp.nsecs/1000000000
    if (msg_t-prevmsg_t) < 0.02:
        return
    prevmsg_t = msg_t
    if initodom is None:
        initodom = copy.deepcopy(odommsg)
    #adjust the initial odom position, assuming orientation is ok.
    mutex.acquire()
    prevodom = copy.deepcopy(odommsg)
    prevodom.pose.pose.position.x = odommsg.pose.pose.position.x - initodom.pose.pose.position.x
    prevodom.pose.pose.position.y = odommsg.pose.pose.position.y - initodom.pose.pose.position.y
    prevodom.pose.pose.position.z = odommsg.pose.pose.position.z - initodom.pose.pose.position.z
    print('prevodom initodoms : ', prevodom.pose.pose.position.x, initodom.pose.pose.position.x)
    mutex.release()

class ListenerNode(Node):
    def __init__(self):
        super().__init__("baglistener")

def listener():

    global imupub , pt2pub, msgt1_prev, msgt2_prev, pt2frames,letters
    global pt2random
    global pt2buf, pt2buf_bytearray, bzero32
    global odomsub, outfile, prevmsg_t, writer, prevodom, initodom
    pt2frames=0
    msgt1_prev=-1.0
    msgt2_prev=-1.0
    #letters = ''.join(struct.pack("<B",i) for i in range(255))
    #pt2random = ''.join(random.choice(letters) for i in range(2457600)) 
    pt2buf = ''.join('\x00' for i in range(2457600)) 
    bzero32=[0] * 32

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rclpy.init()
    node = ListenerNode()
    #pt2pub = node.create_publisher(PointCloud2, '/camera/depth/points2', queue_size=10)
    pt2sub = node.create_subscription(PointCloud2, "/point_cloud", callback_pt2,10)
    #rospy.Subscriber("/odom_gt", Odometry, callback3)

    prevmsg_t=0
    filename = 'scan_hrt.csv'
    outfile= open(filename, 'w')

    fieldnames = ['t', 'type', 'x', 'y', 'z','qx', 'qy', 'qz', 'qw']
    writer = csv.DictWriter(outfile, fieldnames=fieldnames)
    writer.writeheader()
    prevodom=Odometry()
    initodom=None # this is to be replaced by the first valid odom
    # spin() simply keeps python from exiting until this node is stopped
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    listener()
