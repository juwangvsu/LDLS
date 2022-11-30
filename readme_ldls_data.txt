this file:
/media/student/data_4tb2/ldls_docker_bag

-----------------------------------------------------
point cloud2:
	/husky6/full_cloud don't use. not sure what this is
	/husky6/lidar_points lidar data, has intensity channel
	/husky6/point_cloud_pipeline/cloud seems same as abovea
		frame_id: husky6/base
	/husky6/forward/depth_registered/points camera depth cloud

images:
	/husky6/forward/color/image/compressed camera compressed image 480x640
		frame_id: husky6/forward_color_optical_frame


KITTI calib file prep:

tf from cam to base: (this go to the TR section of kitti calib file)
At time 0.000
- Translation: [0.446, 0.022, 0.524]
- Rotation: in Quaternion [0.512, -0.496, 0.499, -0.492]
            in RPY (radian) [-1.589, -0.024, -1.562]
            in RPY (degree) [-91.023, -1.367, -89.511]
2022-06-08-12-57-32.bag tf msg not complete
2022-06-08-12-53-16.bag tf good
rosrun tf tf_echo /husky6/base husky6/forward_color_optical_frame

camera projection matrix (kitti calib file):
   P = calib_dict['P2'].reshape((3, 4))
	this need to be derived from the rostopic cam_info?

---------------------------
mkdir output/pcd
mkdir output/png
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud
 rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image_repub/compressed /pandar_points:=/husky6/point_cloud_pipeline/cloud


---------------- convert pcd to bin --------------
LDLS/Tools_RosBag2KITTI/pcd2bin/build$
	cmake ../; make; sudo cp pcd2bin /usr/local/bin
to run:
	cd output; mkdir bin
		assume there is pcd folder and empty bin folder
	pcd2bin

/media/student/data_4tb2/Tools_RosBag2KITTI	
husky6/forward_color_optical_frame

