----11/28/22 add bag2kitti submodule---

----8/31/22 lidar data format ---------------
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

