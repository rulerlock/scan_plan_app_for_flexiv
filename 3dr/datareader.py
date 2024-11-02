"""
Read and visualise local point cloud 

Usage:
------
Make sure point cloud files under "/scans" folder
Save camera to hand poses in file "/poses.txt"

"""


import numpy as np 
import open3d as o3d
import re
import os
import copy
from scipy.spatial import cKDTree


class Dataset:
    def __init__(self, data_path) -> None:
        self.use_clustered = False
        self.data_path = data_path
        self.poses = self.read_txt_file(data_path)
        self.scans = self.read_ply_file(data_path)
        self.preprocess()
        self.visualize_all()
        

    def __len__(self):
        return len(self.poses)
    

    def __getitem__(self, index):
        return self.scans[index], self.poses[index]

    def read_txt_file(self, file_path):
        with open(file_path + '/poses.txt', "r") as file:
            txt_data = file.read()
        matrix_strings = re.findall(r'\[\[.*?\]\]', txt_data, re.DOTALL)
        matrices = []
        for matrix_string in matrix_strings:
            lines = matrix_string.strip("[]").replace("[","").replace(']','').split('\n')
            matrix = np.array([list(map(float, line.split())) for line in lines])
            matrices.append(matrix)
        return np.array(matrices)

    def read_ply_file(self, file_path):
        scan_files = os.listdir(file_path + '/scans')
        ply_path = file_path + '/scans/'
        if os.path.exists(file_path + '/clustered'):
            scan_files = os.listdir(file_path + '/clustered')
            self.use_clustered = True
            ply_path = file_path + '/clustered/'
        scan_files.sort()
        scan_files = [i for i in scan_files if i.endswith('.ply')]
        scans = []
        for scan_file in scan_files:
            scan = o3d.io.read_point_cloud(ply_path+scan_file)
            scans.append(np.asarray(scan.points))
        return scans
    
    def store_clustered(self, clustered_points):
        clustered_path = self.data_path + '/clustered'
        if not os.path.exists(clustered_path):
            os.makedirs(clustered_path)
        for i, points in enumerate(clustered_points):
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(clustered_path + '/clustered_'+ str(i+1).zfill(3)+'.ply', pc)


    def clustering_process(point_cloud, threshold, minpoints): #(point_cloud, threshold=0.04, minpoints=20)

        pc = copy.deepcopy(point_cloud)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        labels = pcd.cluster_dbscan(eps=threshold, min_points=minpoints, print_progress=True)

        centroids = []
        for label in np.unique(labels):
            if label == -1: 
                continue
            indices = np.where(labels == label)[0]
            centroid = np.mean(np.asarray(pcd.points)[indices], axis=0)
            centroids.append((label, centroid))


        nearest_label = None
        min_distance = float('inf')
        for label, centroid in centroids:
            distance = np.linalg.norm(centroid)
            if distance < min_distance:
                nearest_label = label
                min_distance = distance


        clustered_points = []
        for i, label in enumerate(labels):
            if label == nearest_label:
                clustered_points.append(pc[i])

        return clustered_points
    
    def preprocess(self, threshold1=0.1, minpoints1=10):
        real_T_fake = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        points = []
        clustered_points = []
        filtered_points = []
        clustered_points = []
        for i in range(len(self.scans)):
            pc = o3d.geometry.PointCloud()
            cur_points = np.asarray(self.scans[i])
            # pc.points = o3d.utility.Vector3dVector(cur_points)
            # o3d.visualization.draw_geometries([pc], window_name=f"3D Point Cloud {i}")
            ones_column = np.ones((cur_points.shape[0], 1))
            
            cur_points = np.hstack((cur_points, ones_column))
            cur_points = cur_points.T
            cur_points = self.poses[i] @ real_T_fake @ cur_points
            # cur_points = np.linalg.inv(dataset.poses[i]) @ real_T_fake @ cur_points
            cur_points = cur_points.T
            cur_points = cur_points[:, :3]
            points.append(cur_points)
            filtered_cur_points = cur_points[(cur_points[:, 0] >= -0.3) & (cur_points[:, 0] <= 0.3) &
                                                (cur_points[:, 1] >= -0.3) & (cur_points[:, 1] <= 0.3) &
                                                (cur_points[:, 2] >= -0.16) & (cur_points[:, 2] <= 0.3)]
            filtered_points.append(filtered_cur_points)
            clustered_cur_points = np.array(Dataset.clustering_process(filtered_cur_points, threshold=threshold1, minpoints=minpoints1))
            clustered_points.append(clustered_cur_points)
            self.clustered_points = clustered_points

        self.store_clustered(clustered_points)

    def visualize_all(self):
        all_points = np.vstack(self.clustered_points)
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(all_points)
        self.all_points = all_points
        o3d.visualization.draw_geometries([pc], window_name='All Clustered Points')


    

