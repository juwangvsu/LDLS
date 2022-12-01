"""
kitti_utils.py

Various classes and functions for working with KITTI data.

"""

import numpy as np
from scipy.spatial import Delaunay

KITTI_CLASSES = ['BG', 'Car', 'Van', 'Truck',
                 'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                 'Misc', 'DontCare']

KITTI_GROUND_LEVEL = -1.73 # height of ground from Kitti car velodyne, in m

def check_points_in_box(points, box_corner_points):
    """
    Check if points are within a 3D bounding box.
    
    Parameters
    ----------
    points : ndarray(dtype=float, ndims=2)
        n_points by 3
    box_corner_points : ndarray
        8 by 3, box vertices

    Returns
    -------
    ndarray(dtype=bool)
        n_points-length array. Element i is 1 if point i is in the box and
        0 if not.
    """
    # Create Delaunay tessalation for the 3D bounding box
    # Then, can use Delaunay.find_simplex() to determine whether a point is
    # inside the box
    hull = Delaunay(box_corner_points)

    in_box = hull.find_simplex(points) >= 0
    return in_box

class KittiLabel(object):
    """
    Attribute descriptions, from KITTI object detection readme.
    Note that all 3D coordinates are given in the 3D camera frame.
    
    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
    alpha        Observation angle of object, ranging [-pi..pi]
    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
    dimensions   3D object dimensions: height, width, length (in meters)
    location     3D object location x,y,z in camera coordinates (in meters)
    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
    """

    def __init__(self, object_type, truncated, occluded, alpha, bbox,
                 dimensions, location, rotation_y):

        self.object_type = object_type  # type: str
        self.truncated = truncated  # type: bool
        self.occluded = occluded  # type: bool
        self.alpha = alpha  # type: float

        # bbox is [left, top, right, bottom] (in pixel coordinates)
        self.bbox = bbox  # type: np.ndarray

        # dimensions is [height, width, length] in 3D coordinates
        self.dimensions = dimensions  # type: np.ndarray

        # location is [x, y, z] in 3D camera coordinates
        self.location = location  # type: np.ndarray
        self.rotation_y = rotation_y  # type: float

    def box_corners(self):
        corner_points = np.empty((8, 3))
        height, width, length = self.dimensions

        k = 0
        # for dx in [-length / 2, length / 2]:
        #     for dy in [-width / 2, width / 2]:
        #         for dz in [0, height]:
        #             corner_points[k, :] = [dx, dy, dz]
        #             k += 1
        for dx in [-width / 2, width / 2]:
            for dz in [-length / 2, length / 2]:
                for dy in [0, -height]:
                    corner_points[k, :] = [dx, dy, dz]
                    k += 1

        # Rotate around z (vertical) axis
        # rotation = -obj.rotation_y + (np.pi / 2)
        rotation = -self.rotation_y + np.pi/2
        c = np.cos(rotation)
        s = np.sin(rotation)
        # R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        R = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        corner_points = (R.dot(corner_points.T)).T
        # add center coordinates to each point
        corner_points = corner_points + self.location
        return corner_points


def load_kitti_lidar_data(filename, verbose=False, load_reflectance=True):
    """
    Loads lidar data stored in KITTI format.
    
    Parameters
    ----------
    filename
    verbose

    Returns
    -------
    numpy.ndarray
        n_points by 4 array.
        Columns are x, y, z, reflectance

    """
    with open(filename, "rb") as lidar_file:
        # Velodyne data stored as binary float matrix
        lidar_data = np.fromfile(lidar_file, dtype=np.float32)
        # Velodyne data contains x,y,z, and reflectance
        lidar_data = lidar_data.reshape((-1,4))
    if verbose:
        print("Loaded lidar point cloud with %d points." % lidar_data.shape[0])
    if load_reflectance:
        return lidar_data
    else:
        return lidar_data[:,0:3]

def load_kitti_labels(filename):
    """
    
    Parameters
    ----------
    filename: str
    projection: KittiProjection
        Camera-Lidar projection. Needed to convert bounding box points from
        camera frame into lidar frames.

    Returns
    -------

    """
    with open(filename, "r") as label_file:
        lines = label_file.readlines()
    # Tr_c2v = projection.inverse_transform()  # camera to velodyne transform
    objects = []
    for line in lines:
        split_line = line.split(" ")
        if len(split_line) < 2:  # not a valid object line
            continue
        object_type = split_line[0]
        truncated = float(split_line[1])
        occluded = int(split_line[2])
        alpha = float(split_line[3])
        bbox = np.array([float(x) for x in split_line[4:8]])
        dimensions = np.array([float(x) for x in split_line[8:11]])
        location_cam = np.array([float(x) for x in split_line[11:14]])

        # transform camera frame center point to lidar frame coordinates
        # location_cam_h = np.append(location_cam, 1).reshape((4,1))
        # location_vel_h = Tr_c2v.dot(location_cam_h)
        # location = location_vel_h[0:3,0] # lidar frame 3D coordinates
        location = location_cam

        rotation_y = float(split_line[14])
        objects.append(KittiLabel(object_type, truncated, occluded,
                                   alpha, bbox, dimensions, location,
                                   rotation_y))
    return objects


class KittiProjection(object):

    def __init__(self, Tr, P):
        self.transformation_matrix = Tr
        self.projection_matrix = P

    def project(self, points, remove_behind=True):
        """
        Project points from the Velodyne coordinate frame to image frame
        pixel coordinates.
        
        Parameters
        ----------
        points: numpy.ndarray
            n by 3 numpy array.
            Each row represents a 3D lidar point, as [x, y, z]
        remove_behind: bool
            If True, projects all lidar points that are behind the camera
            (checked as x <= 0) to NaN

        Returns
        -------
        numpy.ndarray
            n by 2 array.
            Each row represents a point projected to 2D camera coordinates
            as [row, col]

        """
        n = points.shape[0]
        d = points.shape[1]
        Tr = self.transformation_matrix
        P = self.projection_matrix
        if d == 3:
            # Append 1 for homogenous coordinates
            points = np.concatenate([points, np.ones((n, 1))], axis=1)
        projected = (P.dot(Tr).dot(points.T)).T

        # normalize by dividing first and second dimensions by third dimension
        projected = np.column_stack(
            [projected[:, 0] / projected[:, 2],
             projected[:, 1] / projected[:, 2]])

        if remove_behind:
            behind = points[:,0] <= 0
            projected[behind,:] = np.nan

        return projected

    def transform(self, points):
        """
        Perform 3D transformation without projection.
        
        Parameters
        ----------
        points

        Returns
        -------

        """
        n = points.shape[0]
        Tr = self.transformation_matrix
        # Append 1 for homogenous coordinates
        points = np.concatenate([points, np.ones((n, 1))], axis=1)
        return ((Tr.dot(points.T)).T)[:,0:3]


    def inverse_transform(self, points):
        """
        Perform inverse 3D transformation
        
        Returns
        -------
        np.ndarray
            4 by 4 matrix

        """
        Tr = self.transformation_matrix
        Rinv = Tr[0:3, 0:3].T
        d = Tr[0:3, 3]
        Tr_inv = np.zeros((4, 4))
        Tr_inv[0:3, 0:3] = Rinv
        Tr_inv[0:3, 3] = -Rinv.dot(d)
        Tr_inv[3, 3] = 1
        points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
        return ((Tr_inv.dot(points.T)).T)[:,0:3]

    @classmethod
    def load_file(cls, calib_path):
        """
        Load a calibration file from KITTI object detection data.
        
        Parameters
        ----------
        calib_path: str
            Path to calibration file

        Returns
        -------

        """
        calib_dict = cls._file_to_calib_dict(calib_path)
        # Get transformation matrices from the calibration data
        velo_to_cam = calib_dict['Tr_velo_to_cam'].reshape((3, 4))
        Tr = np.concatenate(
            [velo_to_cam, np.array([0, 0, 0, 1]).reshape((1, 4))])
        P = calib_dict['P2'].reshape((3, 4))
        return cls(Tr, P)

    @staticmethod
    def _file_to_calib_dict(calib_path):
        # Read values from the KITTI-specification calibration file
        calib_dict = {}
        with open(calib_path, "r") as calib_file:
            for line in calib_file.readlines():
                s = line.split(":")
                if len(s) < 2: continue  # ignore blank lines
                name = s[0]
                try:
                    data = np.array(
                        [np.float32(num_str) for num_str in s[1].split()])
                    calib_dict[name] = data
                except ValueError:  # not a valid float value
                    continue
        return calib_dict
