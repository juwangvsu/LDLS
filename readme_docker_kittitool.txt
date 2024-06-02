--------6/2/24 ros2 point cloud to pcd file ------------
python baglistener_pt2pcd_ros2.py
	pcdfile_#.pcd

isaac sim generate 120 deg of data in each pt2 message.

-------8/26/23 docker image ros kitti tool w/ melodic----------------
@linux:

this docker image contain tool to extract image and lidar data from rosbag file into individual files
  pcd2bin
  rosrun obstacle_detection map_generate ...
docker build --tag jwang3vsu/kittitool -f Dockerfile.kitti .

to ruun:
  docker run --rm -it --network host -v /media/robot/data/ldls_docker_bag:/ldls_docker_bag jwang3vsu/kittitool
  @docker:
    directory: output/pcd, output/png, output/bin
    cd output/  
    (1) play bag file,
    (2) rosrun obstacle_detection map_generate ...
    (3) pcd2bin
    see  readme_ldls_data.txt 11/2022 note
docker push jwang3vsu/kittitool:latest


