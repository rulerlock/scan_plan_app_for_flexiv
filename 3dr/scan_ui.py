"""
Main window for the 3D point cloud scan and CAD model matching.

Workflow:
1) Initialise the robot and the goal object.
2) Scan the object and save point cloud.
3) Reconstruction the point cloud files and confirm object shape is acceptable.
4) Upload the CAD model and match the key points.
5) Calculate the differences between the point cloud and the CAD model.

Usage:
------
When scanning:

Mouse: 
    Drag with left button to rotate around pivot (thick small axes)
    with right button to translate and the wheel to zoom.

Keyboard: 
    [p]     Pause
    [r]     Reset View
    [d]     Open gripper
    [g]     Close gripper
    [s]     Stabilize robot arm
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""


import sys
import re
import os
import copy
from scipy.spatial import cKDTree
import open3d as o3d
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QFileDialog, QLabel, QVBoxLayout, QWidget, QLineEdit
import gripper_init as gripinit
import pc_scan as scan
import datareader
from sklearn.svm import SVC

class ScanApp(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize the UI
        self.setWindowTitle("Grip and Scan with Flexiv RDK") 
        self.setGeometry(100, 100, 600, 400)

        # Setup the layout
        self.layout = QVBoxLayout()
        self.label = QLabel("Welcome! Please fill in the robot and local IP", self)
        self.layout.addWidget(self.label)


        # Robot_ip input
        self.robot_ip_label = QLabel("Robot IP:")
        self.robot_ip_input = QLineEdit()
        self.layout.addWidget(self.robot_ip_label)
        self.layout.addWidget(self.robot_ip_input)

        # Local_ip input
        self.local_ip_label = QLabel("Local IP:")
        self.local_ip_input = QLineEdit()
        self.layout.addWidget(self.local_ip_label)
        self.layout.addWidget(self.local_ip_input)

        # Show point cloud file loading status
        self.folder_path_label = QLabel("No folder selected")
        self.layout.addWidget(self.folder_path_label)

        # Init robot
        self.grip_object_button = QPushButton("Grip Object", self)
        self.grip_object_button.clicked.connect(self.grip_object)
        self.layout.addWidget(self.grip_object_button)

        # Capture point cloud
        self.start_scan_button = QPushButton("Start Scanning", self)
        self.start_scan_button.clicked.connect(self.start_scan)
        self.layout.addWidget(self.start_scan_button)        
        
        # Point cloud alignment
        self.view_scan_button = QPushButton("View Capture Result", self)
        self.view_scan_button.clicked.connect(self.view_scan)
        self.layout.addWidget(self.view_scan_button)

        # Load CAD model
        self.load_button = QPushButton("Load Point Cloud Files", self)
        self.load_button.clicked.connect(self.load_point_clouds)
        self.layout.addWidget(self.load_button)

        # CAD matching
        self.align_button = QPushButton("Supervised Alignment", self)
        self.align_button.clicked.connect(self.align_point_clouds)
        self.layout.addWidget(self.align_button)

        # Difference calculation
        self.diff_button = QPushButton("Calculate Difference", self)
        self.diff_button.clicked.connect(self.calculate_difference)
        self.diff_button.setEnabled(False)
        self.layout.addWidget(self.diff_button)

        # Set central widget
        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

        self.source = None
        self.target = None
        self.transformation = None


    def grip_object(self):
        # 获取用户输入的IP地址
        robot_ip = self.robot_ip_input.text()
        local_ip = self.local_ip_input.text()

        # 检查是否填写了IP
        if not robot_ip or not local_ip:
            print("Please enter both robot IP and local IP")
            return
        
        # 启动封装的循环类
        app_loop = gripinit.AppLoop(robot_ip, local_ip)
        app_loop.run()



    def start_scan(self):
        # 获取用户输入的IP地址
        robot_ip = self.robot_ip_input.text()
        local_ip = self.local_ip_input.text()

        # 检查是否填写了IP
        if not robot_ip or not local_ip:
            print("Please enter both robot IP and local IP")
            return
        
        # 启动封装的循环类
        app_loop = scan.AppLoop(robot_ip, local_ip)
        app_loop.run()

    def view_scan(self):
        folder_path = QFileDialog.getExistingDirectory(self, "Select Dataset Folder")
        if folder_path:
            self.data_path = folder_path
            self.folder_path_label.setText(f"Selected Folder: {folder_path}")
            self.dataset = datareader.Dataset(self.data_path)
            self.dataset.preprocess()
            self.dataset.visualize_all()



    def load_point_clouds(self):
        # Load two point cloud files
        options = QFileDialog.Options()
        file_name1, _ = QFileDialog.getOpenFileName(self, "Select Ideal Point Cloud", "", "Point Cloud Files (*.ply *.pcd);;All Files (*)", options=options)
        file_name2, _ = QFileDialog.getOpenFileName(self, "Select Scanned Point Cloud", "", "Point Cloud Files (*.ply *.pcd);;All Files (*)", options=options)
        
        if file_name1 and file_name2:
            # Read the point clouds
            # self.source = o3d.io.read_point_cloud(file_name1)
            all_points = np.vstack(self.dataset.scans)
            self.target = o3d.geometry.PointCloud()
            self.target.points = o3d.utility.Vector3dVector(all_points)
            self.source = o3d.io.read_point_cloud(file_name2)
            self.label.setText("Point cloud files loaded, please conduct alignment")
            o3d.visualization.draw_geometries([self.source, self.target], window_name="Please check point clouds size")
            

    def pick_points(self, point_cloud):
        """
        Opens the point cloud in a visualization window for user to pick points.
        Returns the indices of the picked points.
        """
        print("Use 'ctrl'+left-click to select corresponding points and press 'Q' when finished.")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(point_cloud)
        vis.run()  # User selects points in the window and presses 'Q' to finish
        vis.destroy_window()
        picked_points = vis.get_picked_points()
        print(f"Picked point indices: {picked_points}")  # Output picked points
        return picked_points

    def align_point_clouds(self):
    # Perform alignment if both point clouds are loaded
        if self.source is not None and self.target is not None:
            # Manually select corresponding points
            self.label.setText("Please manually select corresponding points for the source and target point clouds.")
            picked_ids_source = self.pick_points(self.source)
            picked_ids_target = self.pick_points(self.target)

        # Ensure equal number of picked points
        if len(picked_ids_source) == len(picked_ids_target) and len(picked_ids_source) >= 3:
            # Create correspondence between picked points
            corres = np.zeros((len(picked_ids_source), 2), dtype=np.int32)
            corres[:, 0] = picked_ids_source  # Indices from source point cloud
            corres[:, 1] = picked_ids_target  # Indices from target point cloud

            corres_vector = o3d.utility.Vector2iVector(corres)

            # Compute the transformation matrix
            trans_init = o3d.pipelines.registration.TransformationEstimationPointToPoint().compute_transformation(
                self.source, self.target, corres_vector)
            print("First transfromation matrix from source to target is", trans_init)
            # Apply transformation to the source
            self.source.transform(trans_init)
            self.label.setText("Alignment completed")
            self.diff_button.setEnabled(True)  # Enable difference calculation
            o3d.visualization.draw_geometries([self.source, self.target], window_name="Aligned Point Clouds")
            self.transformation = trans_init

        else:
            # self.label.setText("Number of selected points does not match or less than 3 points selected")
            print("Alignment failed. Number of selected points does not match or less than 3 points selected")
            self.label.setText("Point cloud files loaded, please conduct alignment")


    # def calculate_difference(self):
    #     # Calculate difference after alignment
    #     if self.source is not None and self.target is not None:
    #         if self.transformation is not None:
    #             # Further refine alignment using ICP
    #             threshold = 0.02
    #             reg_p2p = o3d.pipelines.registration.registration_icp(
    #                 self.source, self.target, threshold, self.transformation,
    #                 o3d.pipelines.registration.TransformationEstimationPointToPoint()
    #             )
    #             self.source.transform(reg_p2p.transformation)
    #             print("Fine transfromation matrix from source to target is", reg_p2p.transformation)

    #             # Calculate the difference between the point clouds
    #             # diff = self.source.compute_point_cloud_distance(self.target)
    #             diff = self.target.compute_point_cloud_distance(self.source)
    #             diff_cloud = self.source.select_by_index([i for i, d in enumerate(diff) if d > threshold])
    #             o3d.visualization.draw_geometries([diff_cloud])
    #             # o3d.visualization.draw_geometries([self.source, diff_cloud])
    #             # o3d.visualization.draw_geometries([self.source, self.target, diff_cloud])
    #             self.label.setText("Difference calculation completed")
    #             # print()/

    def rotate_vector(self, v, axis, theta):

        axis = axis / np.linalg.norm(axis)
        
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        term1 = v * cos_theta
        term2 = np.cross(axis, v) * sin_theta
        term3 = axis * np.dot(axis, v) * (1 - cos_theta)

        return term1 + term2 + term3

    def points_in_cylinder(self, points, translation, rotation, radius, height):
        transformed_points = copy.deepcopy(points)
        transformed_points = (transformed_points - translation) @ rotation.T

        distance_to_axis = np.sqrt(transformed_points[:, 0] ** 2 + transformed_points[:, 1] ** 2)
        within_radius = distance_to_axis < radius
        within_height = np.logical_and(transformed_points[:, 2] > - height / 2, transformed_points[:, 2] < height / 2)

        return np.logical_and(within_radius, within_height)


    def calculate_difference(self):
        threshold=0.002
        minpoints=10
        radius=0.05
        if self.source is not None and self.target is not None:
            if self.transformation is not None:
        # closest_mask = find_closest_points(points1, points2, threshold)
                target_pc, gripper_pc, defec_pc = self.clustering(np.asarray(self.target.points), np.asarray(self.source.points), threshold, minpoints)
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
                S1 = np.vstack([target_pc, gripper_pc])
                for points in defec_pc:
                    S2 = points
                    y = np.array([1] * len(S1) + [-1] * len(S2))
                    X = np.vstack([S1, S2])
                    sample_weight = [1] * len(S1) + [10] * len(S2)
                    svm = SVC(kernel='linear', C=1.)
                    svm.fit(X, y, sample_weight=sample_weight)
                    w = svm.coef_[0]
                    b = svm.intercept_[0]
                    cylinder_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.05, height=0.001)
                    normal_default = np.array([0, 0, 1])
                    rotation_axis = np.cross(normal_default, w)
                    rotation_angle = np.arccos(np.dot(normal_default, w) / (np.linalg.norm(w)))
                    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)
                    cylinder_mesh.rotate(rotation_matrix, center=[0, 0, 0])
                    center_s2 = np.mean(S2, axis=0)
                    d = (np.dot(w, center_s2) + b) / np.linalg.norm(w)
                    projection = center_s2 - d * w / np.linalg.norm(w)
                    for i in np.arange(0.0, radius - 0.03, 0.01):
                        for j in np.arange(0, np.pi * 2, np.pi/6):
                            cylinder_mesh.translate(-cylinder_mesh.get_center())
                            # cylinder 沿着projection点在平面上圆周运动，需要求取平面内任意单位向量，将其长度置为i，在平面内旋转角为j
                            dir = self.rotate_vector(rotation_axis, w, j)
                            dir = dir / np.linalg.norm(dir) * i
                            points_inside = self.points_in_cylinder(S1, projection, rotation_matrix, 0.05, 0.001)
                            if np.sum(points_inside) == 0:
                                cylinder_mesh.translate(projection + dir)
                                #vis
                                o3d.visualization.draw_geometries([cylinder_mesh] + pcds, window_name="3D Point Cloud")
                                flag = 1
                                break
                        if flag == 1:
                            break
    
    def find_closest_points(self, pcd1_points, pcd2_points, threshold=0.1):
        tree = cKDTree(pcd2_points)
        distances, indices = tree.query(pcd1_points)
        closest_mask = distances < threshold
        return closest_mask

    def clustering(self, points1, points2, threshold=0.002, minpoints=10):
        closest_mask = self.find_closest_points(points1, points2, threshold)
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
        return target_pc, gripper_pc, defec_pc

if __name__ == "__main__":
    scan_app = QApplication(sys.argv)
    mainWin = ScanApp()
    mainWin.show()
    sys.exit(scan_app.exec_())
