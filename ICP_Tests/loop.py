import open3d as o3d
import numpy as np
import copy
import pyrealsense2 as rs
import time


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
                 
def preprocess_point_cloud(pcd, voxel_size):
    #print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    #print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    #o3d.visualization.draw_geometries([pcd_down])

    radius_feature = voxel_size * 5
    #print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source, target):
    #print(":: Load two point clouds and disturb initial pose.")

    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source)
    target_pcd.points = o3d.utility.Vector3dVector(target)

    np_points = np.zeros((target.shape[0], 3))
    target_pcd.normals = o3d.utility.Vector3dVector(np_points)

    source_down, source_fpfh = preprocess_point_cloud(source_pcd, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_pcd, voxel_size)

    return source_pcd, target_pcd, source_down, target_down, source_fpfh, target_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    # print(":: Apply fast global registration with distance threshold %.3f" \
    #         % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)

    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def trim_depth(p, min_depth = 2, max_depth = 100):
    p_trimmed = np.delete(p, np.where((p[:, 2] >= min_depth) & (p[:, 2] <= max_depth))[0], axis=0)
    return p_trimmed

def unpack_buffer_data(buff):
    p = np.transpose(np.array(list(zip(*buff))))
    return p


if __name__ == "__main__":

    
    start_time = time.time()

    voxel_size = 0.05  # means 5cm for this dataset

    # Configure depth and color streams...
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('141722073646')
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # ...from Camera 2
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('140122071889')
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    print("--- %s seconds, enable pipelines ---" % (time.time() - start_time))

    try:
        while True:
            
            start_time = time.time()
            # https://github.com/erleben/pySoRo#adding-two-dimensional-data-protocols
            # perfectly describes the issue that I am running into ...

            # Camera 1
            # Wait for a coherent pair of frames: depth and color
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            depth_frame_1 = frames_1.get_depth_frame()
            if not color_frame_1 or not depth_frame_1:
                continue

            pc1 = rs.pointcloud()
            pc1.map_to(color_frame_1)
            points1 = pc1.calculate(depth_frame_1)
            points1_np = points1.get_vertices()
            points1_np = np.asanyarray(points1_np)

            p1 = unpack_buffer_data(points1_np)

            # Wait for a coherent pair of frames: depth and color
            frames_2 = pipeline_2.wait_for_frames()
            color_frame_2 = frames_2.get_color_frame()
            depth_frame_2 = frames_2.get_depth_frame()
            if not color_frame_2 or not depth_frame_2:
                continue

            pc2 = rs.pointcloud()
            pc2.map_to(color_frame_2)
            points2 = pc2.calculate(depth_frame_2)
            points2_np = points2.get_vertices()
            points2_np = np.asanyarray(points2_np)

            p2 = unpack_buffer_data(points2_np)
            
            p1_trimmed = trim_depth(p1)
            p2_trimmed = trim_depth(p2)

            print("--- %s seconds load buffer data from camera---" % (time.time() - start_time))

            start_time = time.time()
            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
            voxel_size, p1_trimmed, p2_trimmed)
            print("--- %s seconds prepare data---" % (time.time() - start_time))

            start_time = time.time()
            result_ransac = execute_fast_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        voxel_size)
            print("--- %s seconds execute ransac---" % (time.time() - start_time))

            start_time = time.time()
            result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                            voxel_size)
            print("--- %s seconds refine registration---" % (time.time() - start_time))

            draw_registration_result(source, target, result_icp.transformation)


    finally:

        # Stop streaming
        pipeline_1.stop()
        pipeline_2.stop()
            
          