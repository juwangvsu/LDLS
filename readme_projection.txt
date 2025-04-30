
-------------5/25/24 projection.py ----------------------a
point_cloud_icp/src/:
	projection.py use utils.Projection class, working...

	we project lidar points to a camera image based on calib txt file

this is a seperate repo

the projected lack useable feature for kitti data due to long distance of object
	and small number of feature points for object

-------------5/24/24 test_projection.py ----------------------
~/Documents/datasets/kitti_object/training/imgproj
	output of projected images

~/Documents/datasets/kitti_object/training/imgproj_similarity.csv
	fmt: fn1, fn2, quat

not working...

