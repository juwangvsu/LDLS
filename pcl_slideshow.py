#!/usr/bin/env python
#  usage: python pcl_slideshow.py demo1|demo2|demo3 [fnprefix]|demo5 "phoenix-data/pcdfiles/*.pcd"|
# pyenv shell mini... (removed rospy stuff so works with python3)
# demo1: show all scan*pcd, demo2 show all submap*.pcd
# demo3: show all pcd file with exact name as prefix_###.pcd
# if blank screen, mouse middle wheel to zoom out 
# run under ldls

# orig: cartographer/pcl_slideshow.py,
# run under ~/Docments/cartographer, pyenv shell 2.7.17
# copyright Ju Wang
import yaml
import numpy
#import rospy
#from std_msgs.msg import String
#from std_msgs.msg import Int32, String
#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu, PointCloud2
import json
import yaml
import csv
import sys
#import tf
#from tf.transformations import *
import glob
import pcl
import visualization
import time
import re
pcdfilelist = glob.glob("test/submap_test_h*.pcd")  # submap point cloud in local frame
pcdfilelist2 = glob.glob("test/scan_hrt*.pcd")  # scan point cloud in cam frame
pcdfilelist_bodyrel = glob.glob("rel-body-map*.pcd") # point cloud in body frame
pcdfilelist_body = glob.glob("bodymap*.pcd") # point cloud in body frame of the previous seqno 
yamlfilelist = glob.glob("map*.yaml")
# q0= [-0.5, 0.5, -0.5, 0.5]
# Rm=[0 0 1; -1 0 0; 0 -1 0]
test_xyz = [1,1,1, 1]

# interactive check two original cloud
def democheckpcds():
    while True:
        print("enter two seq number to select two cloud fle: ")
        input_a = raw_input()
        if input_a=='q':
            return
        seq1=int(input_a.split()[0])
        seq2=int(input_a.split()[1])
        cloud1 = pcl.load(pcdfilelist[seq1])
        cloud2 = pcl.load(pcdfilelist[seq2])
        visualization.show2cloud(cloud1, cloud2, blocking=False)


# interactive check two cloud, cloud1 is in its body frame, cloud2 is transformed to cloud1's body frame, both are pre-calculated from gen mode
def democheckpcds_bodyrel():
    print("show two consective clouds, cloud 2 in cloud1's body frame: ")
    while True:
        print("enter the seq number to select cloud fle: ")
        input_a = raw_input()
        if input_a=='q':
            return
        seq1=int(input_a.split()[0])
        print(pcdfilelist_body[seq1],pcdfilelist_bodyrel[seq1])
        cloud1 = pcl.load(pcdfilelist_body[seq1])
        cloud2 = pcl.load(pcdfilelist_bodyrel[seq1])
        visualization.show2cloud(cloud1, cloud2, blocking=False)

def sortfilelist(flist):
#assume file name format: xxxx_##.yyya, the substr after the last _ is number
    seqnlist=[]
    if len(flist)==0:
        return None
    fn=flist[0]
    fnprelist = fn.split('.')[0].split('_')[0:-1]
    zz=''
    for qq in fnprelist:
        if zz is '':
            zz=qq
        else:
            zz=zz+'_'+qq
    fnprefix = zz
    #fnprefix = fn.split('.')[0].split('_')[0]
    fnpostfix=fn.split('.')[1]
    for fn in flist:
        seqn = int(fn.split('.')[0].split('_')[-1])
        seqnlist.append(seqn)
    seqnlist.sort()
    sortedflist=[]
    for seqn in seqnlist:
        fn=fnprefix+'_'+str(seqn)+'.'+fnpostfix
        sortedflist.append(fn)
    return sortedflist

# transform from cam frame to body frame
# input numpy array homo points
# euler angle of cam frame: [z-x-y order, -90, -90, 0], 
#   equvalent [x-y-z order: -90, 90, 0]
# quat [ -0.5, 0.5, -0.5, 0.5 ]
# rotation matrix:[  0.0000000,  0.0000000,  1.0000000;
#                   -1.0000000,  0.0000000,  0.0000000;
#                   0.0000000, -1.0000000,  0.0000000 ]
def  cam_to_body(parray_cam_h, pcsize):
    q_cam = [ -0.5, 0.5, -0.5, 0.5 ]
    Rm_cam=quaternion_matrix(q_cam)
    parray_body_h = numpy.array(parray_cam_h, copy=True)  
#    print('parray_cam_h.size ', parray_cam_h.size)
    for i in range(1,pcsize):
            parray_body_h[i] = numpy.matmul(Rm_cam,parray_body_h[i]) 
    return parray_body_h

# transform from  frame1 to frame0
# Rm, rot matrix
# t0 translation
# return pc in world frame, homo representation
def  frame1_to_frame0(parray_body_h, Rm, t0, pcsize):
        # transfrom from body frame to world frame
        parray_wf_h = numpy.array(parray_body_h, copy=True)
        #parray_wf_h = numpy.pad(parray_wf, ((0,0),(0,1)), mode='constant', constant_values=1)
        for i in range(1,pcsize):
            parray_wf_h[i] = numpy.matmul(Rm,parray_body_h[i]) + t0
        return parray_wf_h

def loadpcd(pcdfilename="mapdata_6.pcd"):
    p = pcl.load(pcdfilename)
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (50)
    fil.set_std_dev_mul_thresh (1.0)
    fil.filter().to_file("test.pcd".encode('utf-8'))
    return p

def pcd_worldframe(yamlfn="mappose_6.yaml",pcdfn="mapdata_6.pcd"):
  with open(yamlfn, "r") as stream:
    try:
        pose0=yaml.safe_load(stream)
        print(pose0['header'])
        t0=[pose0['pose']['position']['x'], pose0['pose']['position']['y'], pose0['pose']['position']['z'], 0]
        #t0=[-pose0['pose']['position']['z'], -pose0['pose']['position']['y'], pose0['pose']['position']['x'], 0]
        q0=[pose0['pose']['orientation']['x'], pose0['pose']['orientation']['y'], pose0['pose']['orientation']['z'],pose0['pose']['orientation']['w']]
        Rm=quaternion_matrix(q0)
        print(yamlfn, ' q0: ', q0 , 't0: ', t0)

        test_xyz_world_h = numpy.matmul(Rm,test_xyz)
        test_xyz_world = test_xyz_world_h[0:4] + t0
        #print('test_xyz_world_h: ', test_xyz_world_h, 'test_xyz_world: ', test_xyz_world)
        p = loadpcd(pcdfn)
        parray_cam = numpy.asarray(p)

        #expand column
        parray_cam_h = numpy.pad(parray_cam, ((0,0),(0,1)), mode='constant', constant_values=1)

        # transform to body frame
        parray_body_h = cam_to_body(parray_cam_h, p.size)
        pp_body =pcl.PointCloud(parray_body_h[:,[0,1,2]])
        pp_body.to_file("body"+pcdfn.encode('utf-8'))

        # transfrom from body frame to world frame (frame0)
        parray_wf_h = frame1_to_frame0(parray_body_h, Rm, t0, p.size)
        pp=pcl.PointCloud(parray_wf_h[:,[0,1,2]])
        pp.to_file("wf"+pcdfn.encode('utf-8'))

    except yaml.YAMLError as exc:
        print(exc)

#calculate the 2nd frame's pcd and pose w.r.t the body frame of previous frame 
# p_w = Rm0 * P_f0 + t0
# p_w = Rm1 * P_f1 + t1
# p_f0 = Rm0^-1 (Rm1 * p_f1 +t1 -t0)
# p_f0 = Rm0^-1 * Rm1 * p_f1 + Rm0^-1 *(t1-t0)
# so frame1 pose in frame0 is described by 
# Rm2 = Rm0^-1 * Rm1, t2 = Rm0^-1 *(t1-t0)
def pcd_relframe(yamlfn1="mappose_6.yaml",pcdfn1="mapdata_6.pcd", yamlfn2="mappose_7.yaml",pcdfn2="mapdata_7.pcd"):
     stream1=open(yamlfn1, "r")
     stream2=open(yamlfn2, "r")
     pose1=yaml.safe_load(stream1)
     t1=[pose1['pose']['position']['x'], pose1['pose']['position']['y'], pose1['pose']['position']['z'], 0]
     q1=[pose1['pose']['orientation']['x'], pose1['pose']['orientation']['y'], pose1['pose']['orientation']['z'],pose1['pose']['orientation']['w']]
     pose2=yaml.safe_load(stream2)
     t2=[pose2['pose']['position']['x'], pose2['pose']['position']['y'], pose2['pose']['position']['z'], 0]
     q2=[pose2['pose']['orientation']['x'], pose2['pose']['orientation']['y'], pose2['pose']['orientation']['z'],pose2['pose']['orientation']['w']]
     Rm1=quaternion_matrix(q1)
     Rinv1 = numpy.linalg.inv(Rm1)
     Rm2=quaternion_matrix(q2)
     print(yamlfn1, ' q1: ', q1 , 't1: ', t1)
     Rm3 = numpy.matmul(Rinv1, Rm2) 
     t3 = numpy.matmul (Rinv1 , (numpy.array(t2)-numpy.array(t1)))
     q3=quaternion_from_matrix(Rm3)
     yamlfn3="rel-"+yamlfn2
     print(yamlfn3, ' q2: ', q2 , 't2: ', t2)
     stream3 = open(yamlfn3, "w")
     stream4 = open(yamlfn3+".txt", "w") # pure text format: x,y,z,qx,qy,qz,qw
     stream4.write(str(t3[0:3]).strip('[]')+"\n")
     stream4.write(str(q3).strip('[]'))
     stream4.close()

     pose3=dict.copy(pose2)
     pose3['pose']['position']['x']=float(t3[0])
     pose3['pose']['position']['y']=float(t3[1])
     pose3['pose']['position']['z']=float(t3[2])
     pose3['pose']['orientation']['x'] = float(q3[0])
     pose3['pose']['orientation']['y'] = float(q3[1])
     pose3['pose']['orientation']['z'] = float(q3[2])
     pose3['pose']['orientation']['w'] = float(q3[3])
     yaml.safe_dump(pose3,stream3)
     stream3.close()

     p = loadpcd(pcdfn2)
     parray_cam = numpy.asarray(p)
     parray_cam_h = numpy.pad(parray_cam, ((0,0),(0,1)), mode='constant', constant_values=1)

     # transform to body frame
     parray_body_h = cam_to_body(parray_cam_h, p.size)
     pp_body =pcl.PointCloud(parray_body_h[:,[0,1,2]])

     # transform to frame 1 and save
     parray_wf_h = frame1_to_frame0(parray_body_h, Rm3, t3, p.size)
     pp=pcl.PointCloud(parray_wf_h[:,[0,1,2]])
     pp.to_file("rel-body-"+pcdfn2.encode('utf-8'))

#calculate pcd and pose w.r.t the body frame of previous point cloud
def pose_pcd_rel(yamlfilelist,pcdfilelist):
  for i in range(1,len(pcdfilelist)-1):
      pcd_relframe(yamlfilelist[i],pcdfilelist[i],yamlfilelist[i+1],pcdfilelist[i+1])

def pose_pcd_world(yamlfilelist,pcdfilelist):
  for i in range(1,len(pcdfilelist)):
      pcd_worldframe(yamlfilelist[i], pcdfilelist[i])

def filter_prefix(fnprefix, flist):
    newflist=[]
    for i in range(1, len(flist)):
        #m=re.match('^test/scan_hrt_voxeledge_[0-9]*.pcd', pcdfilelist3[0])
        #print(fnprefix)
        patt= fnprefix+'[0-9]*.pcd'
        #print('reg ex pattern: ', patt)
        m=re.match(patt, flist[i])
        if m:
            print(flist[i])
            newflist.append(flist[i])
    return newflist

def pcdfl_filtered(fnprefix):
      print('prefix: ', fnprefix)
      pcdfilelist3 = glob.glob(fnprefix+"*.pcd")
      pcdfl3_filtered = filter_prefix(fnprefix, pcdfilelist3)
      pcdfl3_filtered = sortfilelist(pcdfl3_filtered)
      return pcdfl3_filtered

def loopfunc(pcdfl3_filtered, pcdfl4_filtered=None):
      i=0;
      while True:
      #for i in range(len(pcdfl3_filtered)):
        key=raw_input('enter a key: ')
        print(key)
        if key=='q':
            break
        if key=='a':
            i = i-1
            i = max(i,0)
        if key=='f' or key=='':
            i=i+1
            i=min(i,len(pcdfl3_filtered)-1)
        cloud1 = pcl.load(pcdfl3_filtered[i])
        print(i, pcdfl3_filtered[i])
        if pcdfl4_filtered==None:
            visualization.displaycloud(cloud1,pcdfl3_filtered[i], 'xyz')
        else:
            cloud2 = pcl.load(pcdfl4_filtered[i])
            print(i, pcdfl4_filtered[i])
            visualization.show2cloud(cloud1, cloud2, False)

if __name__ == "__main__":

  #print(pcdfilelist)
  pcdfilelist = sortfilelist(pcdfilelist)
  pcdfilelist2 = sortfilelist(pcdfilelist2)
  #print(pcdfilelist)
  #print(yamlfilelist)
  arglen=len(sys.argv)
  cmd = "inspect"
  if arglen>1:
      cmd = sys.argv[1]
  print("limited func from pcl-python visualization ...")
  print(" usage: python pcl_slideshow.py demo1|demo2|demo3|demo5 [fnprefix] \"phoenix-data\"/pcdfiles/*.pcd ")
  print(" demo3: show all pcd file with exact name as prefix_###.pcd")
  print(" run under ldls")
  if cmd=="demo1":
      print("demo1, show submap clouds")
      for i in range(len(pcdfilelist)):
        cloud2 = pcl.load(pcdfilelist[i])
        print(pcdfilelist[i])
        visualization.displaycloud(cloud2,pcdfilelist[i],'xyz')
        time.sleep(1)
  if cmd=="demo2":
      print("demo2, show scan_hrt% clouds")
      for i in range(len(pcdfilelist2)):
        cloud2 = pcl.load(pcdfilelist2[i])
        print(i, pcdfilelist2[i])
        visualization.displaycloud(cloud2,pcdfilelist2[i], 'xyz')
        time.sleep(2)
  if cmd=="demo3":
      print("demo3, show clouds with exact file prefix")
      fnprefix = sys.argv[2]
      pcdfl3_filtered = pcdfl_filtered(fnprefix)
      loopfunc(pcdfl3_filtered, None)
      #visualization.show2cloud(cloud1, cloud2, False)
      #time.sleep(1)
  if cmd=="demo4":
      print("demo4, show two seq of clouds with two file prefix")
      fnprefix1 = sys.argv[2]
      fnprefix2 = sys.argv[3]
      pcdfl3_filtered = pcdfl_filtered(fnprefix1)
      pcdfl4_filtered = pcdfl_filtered(fnprefix2)
      loopfunc(pcdfl3_filtered, pcdfl4_filtered)
  if cmd=="demo5":
      print("demo5, show cloud filename reg exp")
      fnprefix1 = sys.argv[2]
      pcdfilelist3 = glob.glob(fnprefix1)  # scan point cloud in cam frame
      print('dbg,' , fnprefix1, pcdfilelist3)
      for i in range(len(pcdfilelist3)):
        cloud2 = pcl.load(pcdfilelist3[i])
        print(i, pcdfilelist3[i])
        visualization.displaycloud(cloud2,pcdfilelist3[i], 'xyz')
        time.sleep(2)
