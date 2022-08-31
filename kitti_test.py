import numpy as np
import struct
import open3d as o3d
#from open3d import *

def convert_kitti_bin_to_pcd(binFilePath):
    size_float = 4
    list_pcd = []
    with open(binFilePath, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd

pcd = convert_kitti_bin_to_pcd("data/kitti_demo/velodyne/000571.bin")
o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)
