----- modify code to not use hdf5 file format for dataset ---
	arldell
	newamdpc
-----------sticky---------------------
cv2 error:
	export PYTHONPATH=

--------------6/4/23 retest, demo_movingobj.py ---
	run two frames, to identify the moving object
	2nd frame lidar seg result search the same points in first frame,
	to do such, use match_savedpcd to estimate the relative pose btw the 
	two frame.

status:
	gicp pose estimate done:
	cd data/apgdata
	rosrun point_cloud_icp match_savedpcds _test_guess:=false _foldmode:=true _subsubmode:=twopcds  _foldpath:=pcd _caseid:=421 _caseid2:=422 _savemode:=true

tasks:
	python code:
		run match_savedpcds from python code
		show the gicp resulting pcd file in python code
			/var/tmp/rel_0001_reference_to_reference_0-0_fit_*.pcd
			/var/tmp/rel_csvlog.csv
		read the T|R from the csv file and transform the 2nd frame to 
			the 1st frame
		

--------------5/24/24 retest, test_proj cuda12---
hpzbook: (openmmlab)

code/pkg update:
	mrcnn hack due to tensorflow 2.x, original code use tf1.x
	keras.engine: /home/robot/.pyenv/versions/miniconda3-latest/envs/openmmlab/lib/python3.8/site-packages/mrcnn/model.py
		comment off keras.engine as KE
		replace KE by KL
	/home/student/Documents/venk/LDLS/mask_rcnn/mask_rcnn.py
		tensorflow 2.x update
	apt install cuda-toolkit-12-4
	pip install imgaug cupy-cuda12x  
test:
	python test_proj.py kitti_demo 000571
		this show lidar points in browser

TBD:
	check projection of lidar points...

	
--------------6/1/23 retest, demo.py regression test fix---
follow usage
--------------- 2/28/23 venkat/brain new ldls docker image ----------------------
hptitan ldls_docker_bag/ldls_noconda_docker.img
	/dev/sda4 dockerimg 400gb
onedrive, no conda, 59gb
--------------- 11/21/22 venkat/brain ldls docker image ----------------------
2/6/23 retest ok
arldell, /dev/nvme0n1p9 cuda 11.4 need upgrade, 
	/dev/nvme0n1p16= dockerimg, ldls:latest 
	image id: d71c5a8a0dc6, docker image some local change/commit
		/dev/nvme0n1p16 was ub with 11.7, but was wiped and burn as dockerimg
hptitan, /dev/sdb3 , 
	/dev/sdb1=dockerimg, ldls:latest image id: xxxxd742, orig image? cuda need
	upgrade to 11.7
	due to: Error 804: forward compatibility was attempted on non supported HW
	2/21/23: sdb3 crashed, 
		sdc1 ok with cuda 11.7, backup 2tb ssd external
cuda upgrade afterward:
    affect docker:
        sudo apt-get install -y nvidia-docker2

~/Documents/venk/ldls_docker_bag/
	ldls.docker.image
	run_docker.sh
	...bag
Test steps:
	cd ldls_docker_bag; 
	if image not loaded/imported yet:
		docker load --input ldls.docker.image
		require > 50gb to load the image, after loading it use about 30gb
		imported to dockerimg @ arldell /dev/nvme0n1p16
	./run_docker.sh
		docker container use cuda 11.6, host cuda must >=11.6
		must use nvidia run time
	@docker:
	 	conda activate LDLS
		python demo.py
test new code in host machine works:
	venk/ldls_docker_bag/ldls_brain/

	maskrcnn hybrid(native): 214.987 vs 619.678 ms 
	maskrcnn hybrid(docker): 254.987 vs 619.678 ms 
		the hybrid model use pytorch and torch2trt
		the stock maskrcnn use tf
	lidar process (native): 209.121 vs 312.450 ms
	lidar process(docker) 232.771 vs 312.450 ms
	kd_tree stuff took dt = 121.533 ms

the ldls code in the docker image use hybrid mask_rcnn, improving the speed

Status:
	docker image runs ok, but torch not finding cuda() device. see testcuda.py output
	test new code in host machine works:
		venk/ldls_docker_bag/ldls_brain/
	fixed: the host cuda version must be > the docker cuda version. so we upgrade to cuda 11.7 solve the problem
		
	

--------------- run ldls in docker ----------------------
clone ldls, install pyenv, conda, ldls, and various package
firefox to run jupyter-lab, since plotly need jupyter,
jupyter looking for a brower, so install firefox
then firefox need to be run as non-root,
so adduser robot in docker, @host xhost +si:localuser:robot
also add a robot user at host for xhost to work
now I can in docker: su -l robot; export DISPLAY=:0; firefox

now I need robot user in docker to access pyenv, ldls code.

--------------------9/7/2022 gpu memory usage and release ----
maskrcnn use 3gb of gpu memory.
lidar segmentation use cupy, which use 300mb gpu memory
cupy gpu management can not free maskrcnn gpu, which is 
tensorflow.
To release tf gpu memory, use numba: (see demo.py)
	from numba import cuda
	device = cuda.get_current_device()
	device.reset()

----------------------8/18/22 python-pcl ----------------
using python-pcl to visualize the result:
	python-pcl install using the whl build from source
	pip install ./python_pcl-0.3.0rc1-cp37-cp37m-linux_x86_64.whl 
pcl_slideshow.py

----------------------8/9/22 kitti2bag ----------------
pip2 install kitti2bag
pyenv shell system
if still not work, modified to use python2 in the script

-----------------------7/27/22 LDLS test -------------------
forked from origin: https://github.com/juwangvsu/LDLS.git
demo.py:
	arldell out of gpu mem
	newpcamd works.
	need about 7gb gpu memory
	chart_studio.plotly also use 1.5 gb gpu memory. that is after lidarsegment run so won't impose additional gpu mem.

-----------------------7/27/22 map load 0.9.13 and 0.8.4 -------------------
use -windowed -ResX=600 -ResY=400 improve frame rate

0.9.13:
  ./CarlaUE4.sh -windowed -ResX=600 -ResY=400
  change startup map:
	vi CarlaUE4/Config/DefaultEngine.ini
	GameDefaultMap=/Game/Carla/Maps/Town02.Town02

  map, 
     python config.py -m Town05
        0.9.13 default load Town10HD
  weather
     Carla_9-13/PythonAPI/util$ python config.py --weather ClearNoon

  performance manuel drive:
	python manual_control.py
		R recording images.(_out/)
		P auto driving
	arldell/0.9.13/Town10HD, 10HD_Opt  3 fps
			Town04, 05 --- 5 fps
			Town01,02,03  --- 8-15 fps	
		map detail and # objs matter
	

0.8.4:
  has two maps.
  ./CarlaUE4.sh -windowed -ResX=600 -ResY=400
	run in drive mode. wasd keyboard ctl car
  ./CarlaUE4.sh /Game/Maps/Town01 --carla-settings=Example.CarlaSettings.ini -windowed -ResX=600 -ResY=400
	server mode, wait for client connect, no wasd ctl
  ./CarlaUE4.sh /Game/Maps/Town02 --carla-settings=Example.CarlaSettings.ini

	15 fps.

  no config.py to change map at run time.

----7/26/22 carla-training-data testing -------------
two computer setup work for 0.8.4

datagen..py:
	pyenv shell system
	python3 datageneration.py -a
		start auto pilot
	see constants.py
	data saving logic: STEPS_BETWEEN_RECORDINGS=10 (sec)
		DISTANCE_SINCE_LAST_RECORDING=10
to add pedestrian in the saved data, edit
    constants.py: 
	CLASSES_TO_LABEL = ["Vehicle" , "Pedestrian"]

the measurements contain all cars and pedestrian. the function
	create_kitti_datapoint() check if each object is viewable in current
	image, using player and objs location and camera matrix

    for agent in measurements.non_player_agents:
        print(agent.id) # unique id of the agent
        #print(type(agent)) # unique id of the agent
        #print(agent) # unique id of the agent
        if agent.HasField('vehicle'):
            print('got vehicle:') # unique id of the agent
            print(agent.vehicle.forward_speed)
            print(agent.vehicle.transform)
            print(agent.vehicle.bounding_box)
        if agent.HasField('pedestrian'):
            print('got pedestrian:') # unique id of the agent
            print(agent.pedestrian.transform)
            print(agent.pedestrian.bounding_box)


----7/15/22 carla-training-data testing -------------
/media/student/data6/venk/carla-training-data/visualization
	python3 vis_utils.py -d ../_out --show_image_with_boxes --vis --ind 1

pip3 install PyQt5 mayavi vtk
sudo apt install python3-pyqt5.qtsvg
pip3 install pyqt5-tools 

----7/15/22 carla testing -------------
carla version messy:
	0.8.4 no window zip, don't know how to install python api
	carla crash in linux if wrong client version. huge file in /var/lib/apport/coredump

0.8.4 finaly working:
Install process:
	download CARLA_0.8.4.tar.gz
	cd PythonClient
	pyenv shell system
		use system python3
	python3 setup.py build
	sudo python3 setup.py install
	pip3 list|grep carla
		this should show "carla-client   0.8.4"
Test run:
./CarlaUE4.sh --carla-settings=Example.CarlaSettings.ini
python3 manual_control.py
	system python3

carla changes:
	0.8.4 carla.client python code
	0.9.13 Client c++ code


----7/15/22 yolov5_ros testing -------------

/media/student/data5/cvbridge_build_ws/
	build with catkin build and python3
	test with usb_cam pkg
	currently force it to use cpu
		cv_bridge and python3 issue was trick and for now seems resolved. see readme file 
		in yolov5_ros

----7/8/22 testing 2d-3d labeling -------------
	LDLS/
		2d-3d code, ipynb run time error at arldell, cudnn problem, 
		runs good at newamdpc, gpu memory?
		pyenv shell mini...
		conda env create -f environment.yml
		conda activate ldls
		pip install Cython
		conda deactivate
		conda env update --file environment.yml
		conda activate ldls
		pip install open3d scikit-image
	        pip install chart-studio
		conda install -c conda-forge jupyterlab
	        conda install -c https://conda.anaconda.org/plotly plotly

		ipython kernel install --user --name=LDLS
		jupyter-lab # this will load jupyter webpage to run ipynb
			plotly not working to show points
			fix: install nodejs 14 and jupyter labextension. 
				curl -fsSL https://deb.nodesource.com/setup_14.x | sudo -E bash -
				sudo apt install -y nodejs
				jupyter labextension install "@jupyter-widgets/jupyterlab-manager"

				jupyter labextension install "jupyterlab-plotly"	
		jupyter-notebook plotpy seems to word.
		
	ssd_keras/
		testing tensorflow code similar to LDLS , OOM run out of 
		memory error 
	
-----7/6/22 add h5 code to tutorial.py to save carla sim data ---
-----6/30/22 arl venk Deep_Continuous_Fusion @ arldell  ---
	arldell:
        	venk/Deep...
	msilap:
		k:\
        forked.
        https://github.com/juwangvsu/Deep_Continuous_Fusion_for_Multi-Sensor_3D_Object_Detection.git


        carla simulator
	msilap:
		k:\
	arldell:
        	venk/...
                both linux and window works. python api code work
                remote carla simulator work. specify host ip at client
		run simulator:
			cd /media/student/data6/venk/Carla_9-13
			./CarlaUE4.sh
		run python code:
			pyenv shell mini...
			TTI dataset formacd /media/student/data6/venk/Carla_9-13/PythonAPI/examples
			student@arldell:/media/student/data6/venk/PythonAPI/examples$ python vehicle_gallery.py  --host 192.168.86.229
			student@arldell:/media/student/data6/venk/PythonAPI/examples$ python manual_control.py

	data repot:
		https://drive.google.com/drive/folders/1rGEApv2lBG_HFMQrA_4snG3E4_rQCpF3

------------------FAQ trouble shooting-----------------

arldell gpu memory not enough if chrome or other gpu program running.

numpy if upgrade to 1.21+ might break numba. use 16.3 or 17.3 if open3d requires
  error: expected dtype object, got 'numpy.dtype
  this error is possibly due to an incompatibility between your numpy and numba versions. 

carla 0.9.13 seems won't run without a gpu

catastrophic error: cannot open source file "cuda_fp16.h"
	check nvcc
	check /usr/local/cuda/bin is good
	add nvcc in PATH, such as:
		export PATH=/usr/local/cuda/bin:$PATH
plotly error:
	pip install chart-studio
	conda install -c https://conda.anaconda.org/plotly plotly

$PYTHONPATH: cv2 error
	should not contain /usr/local/lib/python2.7.../ 
	this might cause cv2 error. the opencv-python contain opencv binary
	4.1.x but the /usr/local/lib/python2.7 might contain the opencv 3.2.0
	somehow the cv2 might try to load the wrong one if there is an 
	3.2.0 cv2.so at /usr/local/lib...
	fix:
		export PYTHONPATH=
	
cupy:(hpzbook)
	pip install cupy-cuda12x
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
	sudo dpkg -i cuda-keyring_1.1-1_all.deb
	sudo apt-get update
	sudo apt-get -y install cuda-toolkit-12-4

--------additional pkgs installed --------------------
open3d 0.15.2
 
