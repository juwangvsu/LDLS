this file:
/media/student/data_4tb2/ldls_docker_bag

-----------------4/29/2025 run bag2pcd jackal data----------
hpzbook, (openmmlab)
data/jackal_test
	cp data/kitti_demo/calib/000571.txt data/jackal_test/calib/000071.txt
	ln -sn bin velodyne
	ln -sn png image_2	
test:
        python test_proj.py jackal_test 000071 [rgb]
                this show lidar points in browser

see: 5/24/24 readme_ldls_docker.txt
------9/16/23 Caliberation paper draft --------------------------

https://www.overleaf.com/project/62f29edf00b10b04c3a423fc
figures/images:
	C:\Users\Ju Wang\Documents\army 2021\venkat\paperdraft

arl-caliberation page: 
	use chess board, background subtraction, 
	https://gitlab.sitcore.net/aimm/phoenix-r1/-/wikis/Tutorials/Multi-Sensor-Calibration
	https://gitlab.sitcore.net/aimm/phoenix-r1/-/blob/master/src/sensors/calibration/msg-cal/msg_cal/tutorial/README.md

------9/16/23 icpnp caliberation code works --------------------------
ldls:
	/media/student/data5/cvbridge_build_ws/src/point_cloud_icp
	python icpnp.py --root_folder /media/student/data10/arl_aws/output/ --data_folder velodyne_no_ground --fn 000075_radius_inview_notree.pcd --calib_path calib_test.txt

-------9/10/23 quaternion euler angles rotmat rotvec opencv scipy --------

opencv scipy ros etc all right-hand system
rotVec = np.array([[1.57, 0.  , 0.  ]], dtype=float32)
rotmat,_=cv2.Rodrigues(rotVec)
rotvec_2, = cv2.Rodrigues(rotmat)
from scipy.spatial.transform import Rotation as R
quat_sci = R.from_rotvec(rotVec)
quat_sci.as_matrix()
array([[[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00],
        [ 0.00000000e+00,  7.96274259e-04, -9.99999683e-01],
        [ 0.00000000e+00,  9.99999683e-01,  7.96274259e-04]]])

A rotation vector is a 3 dimensional vector which is co-directional to the axis of 
rotation and whose norm gives the angle of rotation [1].

------9/3/23 pcd2bin -----------------
argparse --indir --outdir

-------3/20/23 demo_param -----------------
sweeping x/y/z 
 usage angle dithering: python demo_param_sen.py --datapath apgsample --id 001145 --xr --d_range 1 --steps 10 --gen_gt
#usage: python demo_param_sen.py --datapath apgsample --id 001145 --x --range 1 --steps 10

-------3/9/23 quaternion euler angles TR matrix -----------------

raw data: kitti_object/calib/
test code: test_quaternion.py

Given two frames: cam frame F_cam, lidar frame F_lidar, 
	TR_lidar_to_cam is the 3x4 matrix that do "lidar frame to cam frame"
	pt_lidar in F_lidar, pt_cam the same point in F_cam,
		pt_cam = TR_lidar_to_cam * pt_lidar

	q_kitti is the rotation part:
		m_kitti = TR_lidar_to_cam[:3,:3]
		q_kitti = quaternion.from_rotation_matrix
			quaternion(-0.50573, -0.49799, 0.4945671, -0.50163)
		ea_kitti = quaternion.as_euler_angles(q_kitti)
			array([-1.571413  ,  1.55598824, -3.14911734])
		ea_kitti_rostf
			(1.57588, -1.56170, -0.001822)
			in ros tf, the equiv euler angle as ea_kitti, they 
			look different, but are the same.

	rotation can be interpreted in two ways:
	
  		(1) convert F_cam to F_lidar, so the F_cam's xyz axis will become F_lidar's axis.
		the euler angle is ea_kitti (alpha, beta, gamma) z-y'-z'', 
	 	each time rotation will be applied to the new intermediate frame	
  		so original F_cam is xyz, the it become x'y'z', then x''y''z''
		final result is x'''y'''z'''
		rotating sequence: first apply alpha on z, 
		then apply beta in y', then apply
		gamma in z''.
		(1).a the equiv ea_kitti_rostf only need two rotations:
			z- alpha 90, then y' beta -90. 

		(2) convert a point coord from one frame to another frame:
		in F_lidar frame, apply the euler angles to a point pt_lidar 
		this result in a new point in F_lidar, the new point's coor in
		F_lidar is the same as the original point pt_lidar's coor in
		F_cam, or pt_cam. 
		ea_kitti [-90, 90, -180] rotation applied to a point in the 
		fixed F_lidar frame. in the order of
		rot sequence: z- gamma -180,then y- beta 90,then z- alpha -90
			ex: quaternion.rotate_vectors(q_kitti,[0,0,1])
				# pt_lidar [0,0,1]-> pt_cam [0,-1,0]
				  pt_lidar [0,1,0]-> pt_cam [-1,0,0]
				  pt_lidar [1,0,0]-> pt_cam [0,0,1]
			a quick way to visually verify is to example the two
			frame in sensor_setup_topview.png: [0,0,1] in lidar
			frame will be [0,-1,0] in cam frame etc.
more note:
	quat format in numpy-quaternion is (w,x,y,z)
	euler angle in numpy-quaternion convention is 'rzyz' and fixed 

	quat in tf2.transformations.quaternion is (x,y,z,w)
	euler angle in ros tf can specify 24 different rotation sequence
	
	ea_kitti_rostf = tf.transformations.euler_from_quaternion(q_kitti_rostf,'rzyz')
	ros tf rpy: roll about an X-axis) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis),

	quat in  scipy.spatial.transform is (x,y,z,w)

ros2 tf:
	from tf2_ros.buffer import Buffer
	from tf2_ros import TransformException
	from tf2_ros.transform_listener import TransformListener
	from tf_transformations import euler_from_quaternion

--------2/28/23 ldls hptitan retest -------------------

retest ok, with python-pcl (local build)

---------1/4/22 sensor caliberation notes from phoenix --------------------
https://gitlab.sitcore.net/aimm/phoenix-r1/-/blob/master/src/sensors/calibration/msg-cal/msg_cal/tutorial/README.md
https://gitlab.sitcore.net/aimm/phoenix-r1/-/wikis/Tutorials/Multi-Sensor-Calibration

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

-----------11/29, 12/15/22 [R2 | T2] in calib file-----------------------------
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
	P2 is loaded to matrix P
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

T is the position of the lidar in cam frame,
The R part is the rotation matrix, the equivlent euler angles if applied to cam axis will become lidar axis?

   point_cam = [R|T] * point_lidar 
	this matrix multiply the point in lidaar frame is the point in camera frame 

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

----------------- Documents/datasets----------
  arl_aws
    bag5.bag                  onedrive:aws`
      john rogers, husky+jackal
    2021-07-09-10-33-16.bag   onedrive:arldata
      venkat
  loam/
    2011_09_30_0018.bag       onedrive:/kitti
  msbuild
    test2_2022-08-01-xxx.bag  onedrive:/baal
  kitti
    kitti sequence (80g)      baalnouv1

-----------------4/28/2025 run bag2pcd jackal data----------
@ws3, 
mkdir -p ~/Documents/ldls_docker_bag/output/pcd
mkdir -p ~/Documents/ldls_docker_bag/output/png
cp .bag to ~/Documents/ldls_docker_bag/
docker run -t -d --restart always --network host -v /home/sysinit/Documents/ldls_docker_bag:/ldls_docker_bag --name ldls --entrypoint /bin/bash jwang3vsu/kittitool
docker exec -it ldls bash
	do 11/2022 test
 
-----------------11/2022 run bag2pcd apgdata----------
8/2023:
    obstale_detection built for pcl 1.8 and ros melodic, for ubuntu 20, use
    docker image jwang3vsu/kittitool
    see readme_docker_kittitool.txt
cd ~/Documents/dataset/ldls_docker_bag
cd ~/Documents/dataset/arl_aws

mkdir output/pcd
mkdir output/png
test:
 rosbag play bag5.bag 
 rosbag play 2021-07-09-10-33-16.bag
 rosbag play jackal_test3_2025-04-28-22-02-42.bag
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/d400/color/image_raw/compressed /pandar_points:=/baal/point_cloud_pipeline/cloud
	error: no intensity field.
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image_repub/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/d400/color/image_raw/compressed /pandar_points:=/baal/point_cloud_pipeline/cloud
	
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
quaternion pkg issue:
	ros tf2 have quaternion funcs
	numpy-quaternion works on python 3
	ros tf2 can't be used in py3 env (miniconda)
	see phoenix note readme

expect dtype get ... numpy error
	cause: numpy version too high, use 1.19.4
