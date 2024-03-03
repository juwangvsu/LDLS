import numpy as np
import quaternion
import sys
def test_quat():
    m1 = tf_mat[:3,:3]
    q1=quaternion.from_rotation_matrix(m1)
    v1=quaternion.as_rotation_vector(q1)
    rd_vec = np.array([delta,0,0])
    v2 = v1 + rd_vec
    q2=quaternion.from_rotation_vector(rd_vec)
    m2=quaternion.as_rotation_matrix(q2)
    projection.transformation_matrix[:3,:3] = m2

import cv2
from scipy.spatial.transform import Rotation as R

def test_opencv_scipy():
    print("test opencv scipy rotvec...")
    rotVec_1 = np.array([[1.57, -1.57  , 1.57  ]], dtype=float)
    rotmat_1,_ =cv2.Rodrigues(rotVec_1)
    rotvec_1,_ = cv2.Rodrigues(rotmat_1)
    quat_sci_1 = R.from_rotvec(rotVec_1)
    print('rotvec ', rotVec_1, '\nrotmat', quat_sci_1.as_matrix())

    rotVec_2 = np.array([[1.2, -1.2  , 1.2  ]], dtype=float)
    rotmat_2,_ =cv2.Rodrigues(rotVec_2)
    rotvec_2,_  = cv2.Rodrigues(rotmat_2)
    quat_sci_2 = R.from_rotvec(rotVec_2)
    print('rotvec ', rotVec_2, '\nrotmat', quat_sci_2.as_matrix())

test_opencv_scipy()
sys.exit()
v1=np.array([np.pi/4, 0., 0.]) #45 degree
q1=quaternion.from_rotation_vector(v1)
m1=quaternion.as_rotation_matrix(q1)
q1_m=quaternion.from_rotation_matrix(m1)
print('q1, q1_m ', q1, q1_m)
v2=np.array([np.pi/6, 0., 0.]) #30 degree
q2=quaternion.from_rotation_vector(v2)
m2=quaternion.as_rotation_matrix(q2)

v3=v1+v2
m3 = m1.dot(m2)
m3_wrong = m1*m2
print('m3 ', m1, m2, m3, m3_wrong)
q3_m=quaternion.from_rotation_matrix(m3)
q3_m_wrong=quaternion.from_rotation_matrix(m3_wrong)
print('q3_m ', q3_m, q3_m_wrong)
q3=quaternion.from_rotation_vector(v3)
q3_q = q1*q2
v3_m=quaternion.as_rotation_vector(q3_m)
v3_q=quaternion.as_rotation_vector(q3_q)
print('\nq3, q3_q, q3_m ', q3, q3_q, q3_m)
print('\n\nv3, v3_q, v3_m ', v3, v3_q, v3_m)

tr_husky_np=np.array([[ 0.00775545, -0.9999694 , -0.0010143 ,  0.005     ],
       [ 0.00229406,  0.00103212, -0.9999968 ,  0.53      ],
       [ 0.9999673 ,  0.0077531 ,  0.00230199,  0.24      ]])

tr_kitti_lidar_to_cam=np.array([[ 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03],
       [ 1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02],
       [ 9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01]])
m_kitti = tr_kitti_lidar_to_cam[:3,:3]
q_kitti = quaternion.from_rotation_matrix(m_kitti)
ea_kitti = quaternion.as_euler_angles(q_kitti)
ea_alpha=ea_kitti.copy()
ea_alpha[[0,1]]=0
ea_alpha[2]=ea_kitti[0] # alpha is actually also applied to z-axis, as z-y-z euler angle
ea_beta=ea_kitti.copy()
ea_beta[[0,2]]=0
ea_gamma=ea_kitti.copy()
ea_gamma[[0,1]]=0
q_alpha=quaternion.from_euler_angles(ea_alpha)
q_beta=quaternion.from_euler_angles(ea_beta)
q_gamma=quaternion.from_euler_angles(ea_gamma)
m_alpha=quaternion.as_rotation_matrix(q_alpha)
m_beta=quaternion.as_rotation_matrix(q_beta)
m_gamma=quaternion.as_rotation_matrix(q_gamma)


p1=np.array([1,0,0,1])
p2=np.array([0,1,0,1])
p3=np.array([0,0,1,1])
zvec_cam = quaternion.rotate_vectors(q_kitti,p3[:3])
print("rotation with quaternion directly\n p3 ->result: [0,-1,0]", zvec_cam)
yvec_cam = quaternion.rotate_vectors(q_kitti,p2[:3])
print(" p2->result: [-1,0, 0]", yvec_cam)
xvec_cam = quaternion.rotate_vectors(q_kitti,p1[:3])
print(" p1->result: [0, 0,1]", xvec_cam)

print("rotaing with m_kitti, p1-> ", m_kitti.dot(p1[:3]))
print("rotaing with m_kitti, p2-> ", m_kitti.dot(p2[:3]))
print("rotaing with m_kitti, p3-> ", m_kitti.dot(p3[:3]))

m_abg = m_alpha.dot(m_beta).dot(m_gamma) #combine 3 rots, first gamma, then beta, then gamma

print('m_kitti vs m_abg=m_alpha*m_beta*m_gamma :\n',m_kitti,'\n', m_abg)


#test_quat()
from scipy.spatial.transform import Rotation as R
rotmat = tr_husky_np[:3,:3]
rot = R.from_matrix(rotmat)
rot.as_rotvec()

