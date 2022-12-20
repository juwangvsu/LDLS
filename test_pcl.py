import pcl
import open3d as o3d
import numpy as np
p = pcl.load_XYZRGB ("pcdfile.pcd")
print(p , type(p))
#fil = p.make_statistical_outlier_filter()
#fil.set_mean_k (50)
#fil.set_std_dev_mul_thresh (1.0)
#fil.filter().to_file("inliers.pcd")
pcl.save(p, "pcdfile3.pcd")

xyzrgb=p.to_array()
pcolor_np=xyzrgb[:,3].astype(int)
numpoints =len(pcolor_np)
pcolor_np3=np.ones(numpoints*3)
pcolor_np3 =pcolor_np3.reshape(numpoints,3)

pcolor_np3[:,0] = np.right_shift(pcolor_np,16)/255
pcolor_np3[:,1] = np.right_shift(np.bitwise_and(pcolor_np,0xffff),8)/255
pcolor_np3[:,2] = np.bitwise_and(pcolor_np,0xff)/255


#pcolor_np3 = np.pad(pcolor_np.reshape(numpoints,1),((0,0),(0,2)), mode='constant', constant_values=60)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyzrgb[:,0:3])
pcd.colors = o3d.utility.Vector3dVector(pcolor_np3)
print(pcd.points, pcd.colors)
o3d.io.write_point_cloud("pcdfile2.pcd", pcd)
