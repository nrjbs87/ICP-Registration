# examples/Python/Basic/working_with_numpy.py

import copy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

if __name__ == "__main__":

    # Load saved point cloud and visualize it
    pcd_load1 = o3d.io.read_point_cloud("toolbin_1_1.ply")
    pcd_load2 = o3d.io.read_point_cloud("toolbin_1_2.ply")

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='TopLeft', width=960, height=540, left=0, top=0)
    vis1.add_geometry(pcd_load1)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name='TopRight', width=960, height=540, left=960, top=0)
    vis2.add_geometry(pcd_load2)

    # vis2.poll_events()
    # vis2.update_renderer()
    # vis2.capture_screen_image("../results/ICP/pc2.jpg")

    # vis2.poll_events()
    # vis2.update_renderer()
    # vis2.capture_screen_image("../results/ICP/pc2.jpg")

    while True:
        vis1.update_geometry(pcd_load1)
        if not vis1.poll_events():
            break
        vis1.update_renderer()

        vis2.update_geometry(pcd_load2)
        if not vis2.poll_events():
            break
        vis2.update_renderer()

    vis1.destroy_window()
    vis2.destroy_window()