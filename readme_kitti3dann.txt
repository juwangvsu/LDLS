tool to annotate 3D object, kitti_object lidar dataset

-------------setup.py --------------------------
pip install -e .
to run cmd line: 
	view_kitti1

---------------------------------------------------
conda activate ldls
	python main.py

dataset:
	kitti/

work in windows, 
buggy in linux:
	viewer() hang
	fix: rm libz.so.1 come with pptk, in ldls env, rm /media/student/data5/.pyenv/versions/miniconda3-latest/envs/ldls/lib/python3.7/site-packages/pptk/libs/libz.so.1
	this will make pptk to use system libz.so.1

--------------3/6/23 pcd bin tools  ----------------
(1) to view bin file:
	LDLS/
	python view_kittibin.py
		enter : data/apgsample/velodyne/001145.bin
(2) to convert pcd to bin
	LDLS/
	pcd2bin
		convert all pcd under pcd/ to bin/
		assume pcd/ and bin/ folders
(3) pcl_viewer ...

(4) gt annotation from kitti bin file and the label file 
	kitti-3d-annotator/
	python main.py
