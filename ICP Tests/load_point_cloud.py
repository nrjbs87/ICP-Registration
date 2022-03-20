# examples/Python/Basic/working_with_numpy.py

import copy
import numpy as np
import open3d as o3d

if __name__ == "__main__":

    # Load saved point cloud and visualize it
    pcd_load = o3d.io.read_point_cloud("2.ply")
    o3d.visualization.draw_geometries([pcd_load])

    # convert Open3D.o3d.geometry.PointCloud to numpy array
    xyz_load = np.asarray(pcd_load.points)

