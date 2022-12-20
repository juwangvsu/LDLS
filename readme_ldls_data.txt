this file:
/media/student/data_4tb2/ldls_docker_bag

-----------12/16/22 add open3d as viewer  -------------------------------------
open3d viewing work
it is a bit ugly with python-pcl and open3d code mixed.  python-pcl is legacy code, but function limited
open3d is newer and good

see visualization.py

-----------12/15/22 add python-pcl local build pkg -------------------------------------
this allow to use python-pcl to show point cloud
	see readme_carto_homepc.txt 
	
-----------12/15/22 [R2 | T2] in calib file------------------------------------
ldls on husky data works now with new calib file
sensitivity:
	T : x 0.52 error is the threshold, horizontal portion reduce 
	    y 0.4 threshold, vertical portion reduce as error +
	    z low effect, due to projection of lidar to cam on z axis
	R: obviously very sensitive
	P2: probably not very sensitive

see 11/29 note detail

-----------12/11/22 test apg data ------------------------------------------
@lenova1 
	test 3DGC, Point-GNN
	point cloud object detection
-----------12/11/22 test apg data ------------------------------------------
@lenova1
	LDLS/data/apgdata
	calib is copyed from kitti dataset, obviously incorrect, result very poor.
	
demo.py:
	cmd line update:
	python demo.py [apgdata 001145]
-----------11/30/22 forward------------------------------------------
see same file in LDLS repo

add kitti 3d annotator
https://github.com/brian-h-wang/kitti-3d-annotator.git

-----------11/29, 12/15/22 [R2 | T2] in calib file------------------------------------------
point cloud2:
point cloud2:
	/husky6/full_cloud don't use. not sure what this is
	/husky6/lidar_points lidar data, has intensity channel
	/husky6/point_cloud_pipeline/cloud seems same as abovea
		frame_id: husky6/base
	/husky6/forward/depth_registered/points camera depth cloud

images:
	/husky6/forward/color/image/compressed camera compressed image 480x640
		frame_id: husky6/forward_color_optical_frame


KITTI calib file prep: LDLS/data/kitti_demo/calib/000571.txt
P0:
array([[718.3351,   0.    , 600.3891,   0.    ],
       [  0.    , 718.3351, 181.5122,   0.    ],
       [  0.    ,   0.    ,   1.    ,   0.    ]])
P2: projection matrix cam2
array([[ 7.183351e+02,  0.000000e+00,  6.003891e+02,  4.450382e+01],
       [ 0.000000e+00,  7.183351e+02,  1.815122e+02, -5.951107e-01],
       [ 0.000000e+00,  0.000000e+00,  1.000000e+00,  2.616315e-03]])

Tr_velo_to_cam: tf from lidar to camera
Tr_velo_to_cam:  [R|T]
array([[ 0.00775545, -0.9999694 , -0.0010143 , -0.00727554],
       [ 0.00229406,  0.00103212, -0.9999968 , -0.06324057],
       [ 0.9999673 ,  0.0077531 ,  0.00230199, -0.2670414 ]])
the T = [0, -.063, -.267] meter, in cam frame, this is in agree with the sensor setup. i.e., lidar is 0.267 meter behind the camera, but camera z-axis car's x-axis (length)

R0_rect rectify cam frame, not used in LDLS code, impact limited
	Y = P_rect_xx * R_rect_00 * (R|T)_velo_to_cam * X
	X = [x y z 1]' from the velodyne
coordinate system Y = [uz vz z]' on image plane
image coordinate is Y/z
sensor setup: https://www.cvlibs.net/datasets/kitti/setup.php

tf from base to cam: 
rosrun tf tf_echo husky6/base husky6/forward_color_optical_frame
this convert base frame to cam frame, or it convert a cam frame point to base frame point,
so to transform a base frame point p_b to cam frame p_c, we must use
	p_c= inv(R) * p_b = R2 * p_b
	notice R2 is consistent with the  Tr_velo_to_cam:  [R] part
At time 0.000
- Translation: T1 [0.446, 0.022, 0.524]
- Rotation: in Quaternion [0.512, -0.496, 0.499, -0.492]
            in RPY (radian) [-1.589, -0.024, -1.562] (this is actually zyx euler angle)
            in RPY (degree) [-91.023, -1.367, -89.511]
  R1 matrix [  0.0107930, -0.0179426,  0.9997808;
	  -0.9996538, -0.0241888,  0.0103575;
	   0.0239977, -0.9995464, -0.0181974 ]]

tf tf_echo husky6/forward_color_optical_frame husky6/base 
(this go to the TR section of kitti calib file)
- Translation: T2 [0.005, 0.532, -0.437]
- Rotation: in Quaternion [0.512, -0.496, 0.499, 0.492]
            in RPY (radian) [2.715, -1.551, -1.121]
            in RPY (degree) [155.585, -88.877, -64.213]
  R2 matrix [0, -1 , 0,
	    0, 0, -1,
	    1, 0, 0]

12/15/22: so [R2 | T2] should be used in the calib file for husky dataset 

2022-06-08-12-57-32.bag tf msg not complete
2022-06-08-12-53-16.bag tf good
rosrun tf tf_echo /husky6/base husky6/forward_color_optical_frame

camera projection matrix (kitti calib file):
   P = calib_dict['P2'].reshape((3, 4))
	this need to be derived from the rostopic cam_info?

rostopic echo /husky6/forward/color/camera_info
  frame_id: "husky6/forward_color_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [620.5262451171875, 0.0, 322.3762512207031, 0.0, 620.6295166015625, 245.07872009277344, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [620.5262451171875, 0.0, 322.3762512207031, 0.0, 
    0.0, 620.6295166015625, 245.07872009277344, 0.0, 
    0.0, 0.0, 1.0, 0.0]

-----------------build bag2pcd ----------
arldell, lenova1

cd ~/catkin_ws/src;
ln -sn /media/student/data6/venk/LDLS/Tools_RosBag2KITTI/catkin_ws/src/obstale_detection
catkin_make

-----------------run bag2pcd apgdata----------
cd ~/Documents/dataset/ldls_docker_bag
cd ~/Documents/dataset/arl_aws

mkdir output/pcd
mkdir output/png
 rosbag play bag5.bag 
 rosbag play 2021-07-09-10-33-16.bag
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image_repub/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud
	12/14/22: this 2021 dataset, example data: apgdata/, 
	topic: /husky6/point_cloud_pipeline/cloud, frameid: husky6/base, examing the data show this frame is align with kitti base frame, which is also
		align with kitti lidar frame.
	=/husky6/forward/color/image_repub/compressed, this camera frame also seems normal, which is like the kitti camera frame orientation.
	so the lidar to cam tf should be similar to kitti original one.

	husky6 sensor setup (based on tf): w.r.t husky6/base
		husky6/base: (0,0,0): x veh length, y veh width, z veh height, base is body center of the robot
		camera:      (.446, .002, .524), husky6/forward_color_optical_frame
		husky6/ouster_lidar: (.3, 0.0, .586)
		so lidar is about 15 cm behind cam, and 6 cm higher than cam in body frame
		lidar data publlished is in base frame, not ouster frame. 

---------------- convert pcd to bin --------------
LDLS/Tools_RosBag2KITTI/pcd2bin/build$
	cmake ../; make; sudo cp pcd2bin /usr/local/bin
to run:
	cd output; mkdir bin
		assume there is pcd folder and empty bin folder
	pcd2bin

/media/student/data_4tb2/Tools_RosBag2KITTI	
husky6/forward_color_optical_frame

-----------trouble shooting FAQ ----------------------
expect dtype get ... numpy error
	cause: numpy version too high, use 1.19.4
