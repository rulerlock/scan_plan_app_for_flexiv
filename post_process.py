import numpy as np 
import open3d as o3d
import re
import os
import copy
from scipy.spatial import cKDTree



def find_closest_points(pcd1_points, pcd2_points, threshold=0.1):
    tree = cKDTree(pcd2_points)
    distances, indices = tree.query(pcd1_points)
    closest_mask = distances < threshold
    return closest_mask

def clustering(points1, points2, threshold=0.1, minpoints=10):
    closest_mask = find_closest_points(points1, points2, threshold)
    remaining_points = points1[~closest_mask]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(remaining_points)
    labels = pcd.cluster_dbscan(eps=threshold, min_points=minpoints, print_progress=True)
    centroids = []
    for label in np.unique(labels):
        if label == -1:  # 跳过噪点
            continue
        indices = np.where(labels == label)[0]
        centroid = np.mean(np.asarray(pcd.points)[indices], axis=0)
        centroids.append((label, centroid))
    min_distance_idx = centroids.index(min(centroids, key=lambda x: np.linalg.norm(x[1])))
    target_pc = points1[closest_mask]
    gripper_pc = np.asarray(pcd.points)[np.where(labels == np.int32(min_distance_idx))[0]]
    defec_pc = []
    for lable in np.unique(labels):
        if lable == -1 or lable == min_distance_idx:
            continue
        defec_pc.append(np.asarray(pcd.points)[np.where(labels == lable)[0]])

    pcds = []
    pcd_target = o3d.geometry.PointCloud()
    pcd_target.points = o3d.utility.Vector3dVector(target_pc)
    #set green
    pcd_target.paint_uniform_color([0, 1, 0])
    pcds.append(pcd_target)
    pcd_gripper = o3d.geometry.PointCloud()
    pcd_gripper.points = o3d.utility.Vector3dVector(gripper_pc)
    #set blue
    pcd_gripper.paint_uniform_color([0, 0, 1])
    pcds.append(pcd_gripper)
    for points in defec_pc:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([1, 0, 0])
        pcds.append(pcd)
    o3d.visualization.draw_geometries(pcds, window_name="3D Point Cloud")
    return target_pc, gripper_pc, defec_pc