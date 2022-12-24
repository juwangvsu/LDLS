----------------------john R label tool ---------------
phoenix dve_dataset branch
dve_dataset/src/utils/localized_labeler

---------12/23/22 mask rcnn train balloon dataset note --------------
LDLS/ use mrcnn, pip installed in ldls env
	mrcnn use tensorflow
	mrcnn retrain mem/gpu heavy, 12gb gpu, 32 gb mem
train:
	conda activate ldls
	cd LDLS/mask_rcnn/samples
	python balloon.py train --dataset=/media/student/data_4tb1/coco/balloon/balloon --weights=coco

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


/sobek/local_point_cloud_cache/renderers/recent_map_compressed       3563 msgs    : zip/CompressedMessage      
/sobek/point_cloud_cache/renderers/full_map_compressed                261 msgs    : zip/CompressedMessage      

----------------FAQ trouble shooting ------------
dve_dataset branch build issue in docker:
        see readme.txt in phoenix_note/

