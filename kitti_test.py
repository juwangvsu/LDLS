# 5/24/24 jw: convert kitti point cloud .bin to .pcd format
import numpy as np
import struct
import open3d as o3d
import argparse
import glob
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

#assuming *bin files under bindir
def convert_dir(bindir, pcddir, numfiles=-1):
    pcdfilelist3 = glob.glob(bindir+"/*.bin")  # scan point cloud in cam frame
    for i in range(numfiles if not numfiles== -1 else len(pcdfilelist3)):
        print(i, pcdfilelist3[i])
        fname = pcdfilelist3[i].split('/')[-1]
        pcd = convert_kitti_bin_to_pcd(pcdfilelist3[i])
        o3d.io.write_point_cloud(pcddir+"/"+fname+".pcd", pcd)
        


def convert_test():
    pcd = convert_kitti_bin_to_pcd("data/kitti_demo/velodyne/000571.bin")
    o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='convert kitti bin to pcd')
    parser.add_argument('--bindir', required=False,
                        default='/media/student/kitti/kitti_object/training/velodyne',
                        metavar="/path/to/logs/")
    parser.add_argument('--pcddir', required=False,
                        default='/media/student/kitti/kitti_object/training/pcd',
                        metavar="/path/to/logs/")
    parser.add_argument('--mode', required=False,
                        default='single', #single or dir
                        metavar="/path/to/logs/")
    args = parser.parse_args()
    print('usage python $@ --mode single|dir')
    if args.mode =='single':
        convert_test()
    elif args.mode=='dir':
        convert_dir(args.bindir, args.pcddir, numfiles=10)

