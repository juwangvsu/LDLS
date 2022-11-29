mv src/obstacle_detection to ~/catkin_ws, and catkin_make there
edit the mapgenerate cpp file to remove the abs path

mkdir output/png
mkdir output/pcd
rosrun obstacle_detection map_generate /axis/image_rect_color/compressed:=/husky6/forward/color/image/compressed /panr_points:=/husky6/lidar_points

