----------------------john R label tool ---------------
phoenix dve_dataset branch
dve_dataset/src/utils/localized_labeler

--------------2/14/23 odom observation etc-----------------------------------
test code:
	tf_save.py tf_sobek_odom_base.txt tf_hathor_odom_base.txt
	traj_save.py traj_hathor.txt
	posegraph_save.py posegraph_hathor.txt
odom topic
   /sobek/jackal_velo...controller/odom: wheel-odom by jackal driver node
   /sobek/odom:   puber: arl_robot_odometry node, fuse with imu orientation
   /sobek/pose_graph:      each node x/y/z is relative move from last node, puber: omnigraph
   /sobek/omnigraph/trajectory: puber: omnigraph
                           accumulative xyz in odom frame, corrected. comparable
                           to /sobek/odom, which is xyz based on imu + wheel-odom
                           the wheel-odom is /sobek/jackal_velo...controller/odom

roslaunch mylaunch_gravel.launch 
tf missing hathor/base to other links, possible urdf file error?
this cause the robot_model in rviz to only have a box.
The robot model move smoothly in rviz, this data is from tf.

bag5 (sobek) tf tree is good.

gravel bag (hathor) tf_static not good, missing from base to imu_link tf chain,
	this cause arl_robot_odometry to not being able to get iq0 and end up
	publish a bad tf odom->base, 
	see 2/14/23 readme_omnigraph

-----------------2/7/23 jackal bag full stack ---------------
arl_aws/jackal_2021/gravel-path-1_2021-04-07-14-46-00.bag

	rosrun phoenix_bag_launch generate_bag_args /media/student/data11/arl_aws/jackal_2021/gravel-path-1_2021-04-07-14-46-00.bag -o mylaunch_gravel.launch

	cp mylaunch_gravel.rviz /media/student/data55/phoenix-mri-master/phoenix-r1/docker-build/install/share/phoenix_bag_launch/rviz/hathor.rviz

	roslaunch mylaunch_gravel.launch

-----------------2/1/23 rerun filtered bag5 full stack ---------------
(1) filter bag file:
	rosbag_filter.sh
                the bag filtered to only contain sobek data
(2) @docker:
	rosrun phoenix_bag_launch generate_bag_args /media/student/data10/arl_aws/bag5_filter.bag -o mylaunch_bag5.launch
(3) create mylaunch_playbag_bag5.launch
	copy the rosbag play part from mylaunch.launch
(4) edit mylaunch_bag5.launch (remove the rosbag play part)
	modify rviz part 
(5) test run:	
        roslaunch mylaunch.launch
        roslaunch mylaunch_playbag.launch

potential missing files:
	./share/hardware_launch/calibration/sobek.yaml
	./share/hardware_launch/config/sobek.yaml
-------------------1/30/23 bag5 playback visualize --------------
rosbag play bag5.bag
@docker rviz, load bag5.rviz
@docker rosrun zip map_zip decompress /sobek/point_cloud_cache/renderers/full_map

--------------------1/30/23 rerun bag5.bag to full stack---------
https://gitlab.sitcore.net/aimm/phoenix-r1/-/wikis/TechnicalNotes/Running-Bags
see 9/22/22 readme_info
arl_aws bag5.bag only contain sensor data and tf,
recorded bag file contain lidar, camera etc
to run full stack on the bag data,
        (1) generate a launch file based on info in the bag file
        (2) use a customized rviz file (modified from samoyed.rviz from phoenix_bag_launch
        (3) run launch file

rosrun phoenix_bag_launch generate_bag_args MIT-CLEAR/06_24_friday/acl_jackal2/clipper_fig8_clockwise_2022-06-24-14-44-56.bag -o mylaunch.launch
roslaunch mylaunch.launch
        rviz global frame use acl_jackal2/base. if use acl_jackal2/map the lidar data display not working about a tf extrapolation error

rosrun phoenix_bag_launch generate_bag_args sim_data_2022-09-25-15-36-25.bag -o mylaunch_wartysim.launch
roslaunch mylaunch_waratysim.launch

---------12/26/22 mask rcnn train jackal balloon2.py --------------
converted labelme label to via format so can use balloon2.py
train result is good. 

test:
	python3 balloon2.py splash --weights=../../logs/balloon2_20221226T1741/mask_rcnn_balloon2__0007.h5 --image=/media/student/data10/arl_aws/ir_labels_s

train:
	python balloon2.py train --dataset=/media/student/data11/coco/balloon/balloon2 --weights=coco

label convert:
	python json_laelme2via.py

test result:
	/media/student/data10/arl_aws/ir_labels_s/splash_output/
	see notes inside the code

---------12/23/22 mask rcnn train jackal dataset note --------------

cd /media/student/data10/arl_aws
splitfolders --output ir_labels_s/json_split --ratio .8 .2 -- ir_labels_s
	this split the json files @ir_labels_s/class1 to
	 ir_labels_s/json_split/train val/

python jackal.py train --dataset=/media/student/data10/arl_aws/ir_labels/json_split  --weights=coco

python jackal.py splash --weights=../../logs/jackal20221225T1806/mask_rcnn_jackal_0001.h5 --image=/media/student/data10/arl_aws/ir_labels/image00084.jpg

status:
	pause as balloon2.py works now

----- 12/23/22 relabed a small data set----------------
ir_labels_s_relabel/albert/*.json, relabeld
			   class1/*.json---- post processed json_fix_albert.py
			   json_split/train
				      val ----- after splitfolders cmd
(1) mask_rcnn/samples$ python json_fix_albert.py
(2) splitfolders --output ir_labels_s_relabel/albert/json_split --ratio .8 .2 -- ir_labels_s_relabel/albert
	split the relabeled small data set
(3) python jackal.py train --dataset=/media/student/data10/arl_aws/ir_labels_s_relabel/albert/json_split  --weights=coco

---------12/23/22 mask rcnn train balloon dataset note --------------
LDLS/ use mrcnn, pip installed in ldls env
	mrcnn use tensorflow
	mrcnn retrain mem/gpu heavy, 12gb gpu, 32 gb mem
train:
	conda activate ldls
	cd LDLS/mask_rcnn/samples
	python balloon.py train --dataset=/media/student/data_4tb1/coco/balloon/balloon --weights=coco
		lenova1
	python balloon.py train --dataset=/media/student/data11/coco/balloon/balloon --weights=coco
		arldell
test:
	python3 balloon.py splash --weights=../../logs/balloon20221226T0112/mask_rcnn_balloon_0011.h5 --image=/media/student/data11/coco/balloon/balloon/val/3825919971_93fb1ec581_b.jpg
	
train results:
	LDLS/logs/

arldell: gpu too small, change image_per_gpu to 1, runs but slow
hptitan: mem small, to upgrade to 32gb
lenova1

--------12/20/22 run j.roger localized_labeler ----------
dve_dataset
lenova1:
	/media/student/data_4tb2/phoenix-r1/
arldell:
	/media/student/data6/phoenix-cdea_arl_objectmapper/phoenix-r1
	dockerimg: /dev/nvme0n1p13
test:
	cd src/util/localized_labeler/launch
	roslaunch localized_labeler interrobot_labeler.launch bag:=/media/student/data10/arl_aws/bag5.bag
	 roslaunch localized_labeler interrobot_labeler.launch bag:=/media/student/data_4tb1/arl_aws/bag5.bag
	this will play the bag file, and create label for image topic observed
	by chinook. the label is 6-point polygon 
	for sobek when it show in the image. labelme
	tool is used. label generated is json format. this can be used to train
	the mask rcnn? the label so far is not tight.

data output:
	~/Documents/dataset/arl_aws/ir_labels
	screen shot: jroger-labeler.png

issue (fixed): 
	it runs, 
	install image_view in docker
		apt update
		apt install ros-noetic-image-view
	rosrun image_view image_view image:=/sobek/forward/color/image_rect_color _image_transport:=compressed
	(2) fix the output_path, hardcoded in interrobot_labeller.cpp, node crash if fail to creae the dir.
		line 136 to 
		private_nh_.param("output_path", output_path_, std::string("/media/student/data_4tb1/arl_aws$/ir_labels"));
	
pending:
	retrain mask rcnn detec sobek
	https://app.roboflow.com/project/robot-seg/1
		a website to test train my dataset
	
note:
	phoenix src contain a ros wrapper of maskrcnn-benchmark
	can't run in docker, missing maskrcnn-benchmark pkg
	the maskrcnn-benchmark is facebook proj, now become detectron 2?
	LDLS use a tensorflow based maskrcnn

-------------------12/19/22 bag5 extraction --------------
rosbag decompress bag5.bag
mkdir output/pcd
mkdir output/png
cd ~/Documents/dataset/arl_aws
steps extract data:
	rosbag play bag5.bag
	rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/chinook/forward/color/image_rect_color/compressed /pandar_points:=/chinook/lidar_points
	cd output; mkdir bin; pcd2bin
tf: for ldls test
/chinook/lidar_points
	frame: chinook/ouster_link
/chinook/forward/color/image_rect_color/compressed
	frame: chinook/forward_color_optical_frame

rosrun tf tf_echo chinook/forward_color_optical_frame chinook/ouster_link
this go to the calib file (LDLS)
- Translation: [0.006, -0.083, -0.172]
- Rotation: in Quaternion [0.494, -0.483, 0.512, 0.510]
            in RPY (radian) [0.202, -1.525, 1.381]
            in RPY (degree) [11.559, -87.378, 79.112]
	R2 = [.008 -.9998 .013
		.045 -.013 -.998
		.998 .009 .05]
quaternion is different than agp's husky, but the rotation matrix agree

rostopic echo /chinook/forward/col/camera_info
D: [-0.05630842596292496, 0.07267305254936218, -0.0004568370641209185, 0.0009758229716680944, -0.02363106794655323]
K: [380.4864196777344, 0.0, 320.06768798828125, 0.0, 380.0823974609375, 240.1641082763672, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [380.4864196777344, 0.0, 320.06768798828125, 0.0, 
    0.0, 380.0823974609375, 240.1641082763672, 0.0, 
    0.0, 0.0, 1.0, 0.0]
   focal length 380, note apg's husky focal length 620. possible lense difference

these parameter are tested in ldls and seems good. test image 000339 modified
to have a human fig in the robot position.

-----------------------12/08/22 localized label john.r -----------------------
cd /media/student/data6/phoenix-cdea_arl_objectmapper/phoenix-r1/src/utils/localized_labeler/launch
roslaunch interrobot_labeler.launch bag:=/media/student/data10/arl_aws/bag5.bag

.catkin_docker:
	add data10 to mount for docker
		export EXTRA_DOCKER_ARGS="${EXTRA_DOCKER_ARGS:--it --net=host --ipc host --ulimit core=-1 --security-opt seccomp=unconfined --cap-add=SYS_PTRACE -v /var/run/dbus:/var/run/dbus -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket -v /media/student/data_4tb1/arl_aws:/media/student/data_4tb1/arl_aws}"

-----------------------bag5.bag -----------------------
lenova1, arldell
chinook husky
sobek	jackal

rosrun image_view extract_images _sec_per_frame:=0.001 image:=/chinook/forward/color/image_rect_color _image_transport:=compressed

LIdar:
/chinook/lidar_points 3566 msgs
/sobek/lidar_points	3565 msgs  

Image:
/chinook/forward/color/image_rect_color/compressed	10693 msgs    : sensor_msgs/CompressedImage 
/chinook/stereo_left/image_rect_color/compressed                     6623 msgs    : sensor_msgs/CompressedImage        
/chinook/stereo_right/image_rect_color/compressed                    9747 msgs    : sensor_msgs/CompressedImage  

/sobek/forward/color/image_rect_color/compressed                    10691 msgs    : sensor_msgs/CompressedImage        

other:
/tf                                                                117254 msgs    : tf2_msgs/TFMessage               
/tf_static                                                              8 msgs    : tf2_msgs/TFMessage
/sobek/pose_graph                                                     198 msgs    : omnimapper_msgs/PoseGraph          
/sobek/pose                                                         44544 msgs    : geometry_msgs/PoseStamped              
/sobek/odom                                                         44541 msgs    : nav_msgs/Odometry                
 /sobek/jackal_velocity_controller/odom                              17822 msgs    : nav_msgs/Odometry                  

/chinook/pose                                                       44562 msgs    : geometry_msgs/PoseStamped 
/chinook/pose_graph                                                     1 msg     : omnimapper_msgs/PoseGraph          
/chinook/odom                                                       44560 msgs    : nav_msgs/Odometry                  
/chinook/husky_velocity_controller/odom                              3566 msgs    : nav_msgs/Odometry        


map: zip compressed format, rviz can't display
	/sobek/local_point_cloud_cache/renderers/recent_map_compressed       3563 msgs    : zip/CompressedMessage      
	/sobek/point_cloud_cache/renderers/full_map_compressed                261 msgs    : zip/CompressedMessage      

	/.../global_occ_grid_converter pub full_map
		pkg/node: arl_grid_map_ros/grid_map_to_occupancy_grid_node
			convert a grid_map message into an OccupancyGrid and publish
			sub: /warty/map/global
			pub:
			    /warty/point_cloud_cache/renderers/full_map
				nav_msgs/OccupancyGrid 
				2d occup map

	/.../map_c  pub full_map_compressed
		pkg/node: zip/map_zip, src/mapping/zip

	/warty/point_cloud_pipeline/point_cloud_pipeline
		pub grid_map_msgs/GridMap /warty/map/global
		3d occup map
		

/spot/odometry:

posegraph: 
	no rviz display. just an array of pose.

----------------FAQ trouble shooting ------------
dve_dataset branch build issue in docker:
        see readme.txt in phoenix_note/

