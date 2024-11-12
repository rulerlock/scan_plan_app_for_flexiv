# Dataset class for vdb fusion
import numpy as np 
import open3d as o3d
import re
import os
import copy


class Dataset:
    def __init__(self, data_path) -> None:
        self.poses = self.read_txt_file(data_path)
        self.scans = self.read_ply_file(data_path)

    def __len__(self):
        return len(self.poses)
    

    def __getitem__(self, index):
        return self.scans[index], self.poses[index]

    def read_txt_file(self, file_path):
        with open(file_path + '/h2c_poses.txt', "r") as file:
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
        scan_files.sort()
        scans = []
        for scan_file in scan_files:
            scan = o3d.io.read_point_cloud(file_path + '/scans/'+scan_file)
            scans.append(np.asarray(scan.points))
        return scans
    

if __name__ == "__main__":



    dataset = Dataset('c:/Users/lule9/Documents/Courses/ME5400A/semantic_scene_perception/cap_data/scan719')
    # print(len(dataset))
    # print(dataset[0])
    # for scan, origin in dataset:
    #     vdb_volume.integrate(scan, origin)

    # vert, tri = vdb_volume.extract_triangle_mesh()

    # # Visualize the results
    # mesh = o3d.geometry.TriangleMesh(
    #     o3d.utility.Vector3dVector(vert),
    #     o3d.utility.Vector3iVector(tri),
    # )

    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh])

    pc = o3d.geometry.PointCloud()
    tcp_T_grip = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.11],
                            [0, 0, 0, 1]])
    real_T_fake = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])
    points1 = np.asarray(dataset.scans[0])

    threshold = 0.02

    grip_T_tcp = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.12],
                        [0, 0, 0, 1]])


    # points2 = np.asarray(dataset.scans[1])
    # # pc.points = o3d.utility.Vector3dVector(np.vstack((points1, points2)))
    # pc.points = o3d.utility.Vector3dVector(points1)
    # origin_points = np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]])
    # origin_colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]])  # 红色
    # # 将原点作为另一个点加入到点云中
    # origin_point_clouds = []
    # for i in range(4):
    #     origin_point_cloud = o3d.geometry.PointCloud()
    #     origin_point_cloud.points = o3d.utility.Vector3dVector(origin_points[i].reshape(1, 3))
    #     origin_point_cloud.colors = o3d.utility.Vector3dVector(origin_colors[i].reshape(1, 3))
    #     origin_point_clouds.append(origin_point_cloud)
    # # point_cloud_with_origin.points = o3d.utility.Vector3dVector(np.vstack((np.vstack((points1, points2)), origin_point)))
    # # 可视化点云
    # o3d.visualization.draw_geometries([pc] + origin_point_clouds, window_name="3D Point Cloud with Origin")



    ones_column = np.ones((points1.shape[0], 1))
    points1 = np.hstack((points1, ones_column))
    points1 = points1.T
    points1 = grip_T_tcp @ dataset.poses[0] @ real_T_fake @ points1
    # points1 = real_T_fake @ points1
    points1 = points1.T
    points1 = points1[:, :3]
    filtered_points1 = points1[(points1[:, 0] >= -0.3) & (points1[:, 0] <= 0.3) & 
                        (points1[:, 1] >= -0.3) & (points1[:, 1] <= 0.3) & 
                        (points1[:, 2] >= -0.1) & (points1[:, 2] <= 0.5)]




    pc.points = o3d.utility.Vector3dVector(filtered_points1)
    origin_points = np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]])
    origin_colors = np.array([[0, 1, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])  # 红色
    # 将原点作为另一个点加入到点云中
    origin_point_clouds = []
    for i in range(4):
        origin_point_cloud = o3d.geometry.PointCloud()
        origin_point_cloud.points = o3d.utility.Vector3dVector(origin_points[i].reshape(1, 3))
        origin_point_cloud.colors = o3d.utility.Vector3dVector(origin_colors[i].reshape(1, 3))
        origin_point_clouds.append(origin_point_cloud)
    # point_cloud_with_origin.points = o3d.utility.Vector3dVector(np.vstack((np.vstack((points1, points2)), origin_point)))
    # 可视化点云
    # o3d.visualization.draw_geometries([pc] + origin_point_clouds, window_name="3D Point Cloud with Origin")

    # pc2 = o3d.geometry.PointCloud()
    # pc2.points = o3d.utility.Vector3dVector(copy.deepcopy(filtered_points1))
    # pc2.transform(np.array([[0.5, -0.8660254, 0, 0],
    #                         [0.8660254, 0.5, 0, 0],
    #                         [0, 0, 1, 3],
    #                         [0, 0, 0, 1]]))
    

    # p3 = copy.deepcopy(filtered_points1)
    # ones_column = np.ones((p3.shape[0], 1))
    # p3 = np.hstack((p3, ones_column))
    # p3 = p3.T
    # p3 = np.array([[0.5, -0.8660254, 0, 0],
    #                         [0.8660254, 0.5, 0, 0],
    #                         [0, 0, 1, 2],
    #                         [0, 0, 0, 1]]) @ p3
    # p3 = p3.T
    # p3 = p3[:, :3]
    # pc3 = o3d.geometry.PointCloud()
    # pc3.points = o3d.utility.Vector3dVector(p3)
    
    # o3d.visualization.draw_geometries([pc, pc2, pc3]+origin_point_clouds, window_name="3D Point Cloud with Origin")




    # points2 = np.asarray(dataset.scans[1])
    # ones_column = np.ones((points2.shape[0], 1))
    # points2 = np.hstack((points2, ones_column))
    # points2 = points2.T
    # points2 = dataset.poses[1] @ real_T_fake @ points2
    # points2 = points2.T
    # points2 = points2[:, :3]
    # filtered_points2 = points2[(points2[:, 0] >= -0.3) & (points2[:, 0] <= 0.3) &
    #                              (points2[:, 1] >= -0.3) & (points2[:, 1] <= 0.3) &
    #                                 (points2[:, 2] >= -0.1) & (points2[:, 2] <= 0.5)]



    # points3 = np.asarray(dataset.scans[2])
    # ones_column = np.ones((points3.shape[0], 1))
    # points3 = np.hstack((points3, ones_column))
    # points3 = points3.T
    # points3 = dataset.poses[2] @ real_T_fake @ points3
    # points3 = points3.T
    # points3 = points3[:, :3]
    # filtered_points3 = points3[(points3[:, 0] >= -0.3) & (points3[:, 0] <= 0.3) &
    #                             (points3[:, 1] >= -0.3) & (points3[:, 1] <= 0.3) &
    #                             (points3[:, 2] >= -0.1) & (points3[:, 2] <= 0.5)]



    # points4 = np.asarray(dataset.scans[3])
    # ones_column = np.ones((points4.shape[0], 1))
    # points4 = np.hstack((points4, ones_column))
    # points4 = points4.T
    # points4 = dataset.poses[3] @ real_T_fake @ points4
    # points4 = points4.T
    # points4 = points4[:, :3]
    # filtered_points4 = points4[(points4[:, 0] >= -0.3) & (points4[:, 0] <= 0.3) &
    #                             (points4[:, 1] >= -0.3) & (points4[:, 1] <= 0.3) &
    #                             (points4[:, 2] >= -0.1) & (points4[:, 2] <= 0.5)]



    # points5 = np.asarray(dataset.scans[4])
    # ones_column = np.ones((points5.shape[0], 1))
    # points5 = np.hstack((points5, ones_column))
    # points5 = points5.T
    # points5 = dataset.poses[4] @ real_T_fake @ points5
    # points5 = points5.T
    # points5 = points5[:, :3]
    # filtered_points5 = points5[(points5[:, 0] >= -0.3) & (points5[:, 0] <= 0.3) &
    #                            (points5[:, 1] >= -0.3) & (points5[:, 1] <= 0.3) &
    #                            (points5[:, 2] >= -0.1) & (points5[:, 2] <= 0.5)]



    # points6 = np.asarray(dataset.scans[5])
    # ones_column = np.ones((points6.shape[0], 1))
    # points6 = np.hstack((points6, ones_column))
    # points6 = points6.T
    # points6 = dataset.poses[5] @ real_T_fake @ points6
    # points6 = points6.T
    # points6 = points6[:, :3]
    # filtered_points6 = points6[(points6[:, 0] >= -0.3) & (points6[:, 0] <= 0.3) &
    #                              (points6[:, 1] >= -0.3) & (points6[:, 1] <= 0.3) &
    #                              (points6[:, 2] >= -0.1) & (points6[:, 2] <= 0.5)]



    # points7 = np.asarray(dataset.scans[6])
    # ones_column = np.ones((points7.shape[0], 1))
    # points7 = np.hstack((points7, ones_column))
    # points7 = points7.T
    # points7 = dataset.poses[6] @ real_T_fake @ points7
    # points7 = points7.T
    # points7 = points7[:, :3]
    # filtered_points7 = points7[(points7[:, 0] >= -0.3) & (points7[:, 0] <= 0.3) &
    #                              (points7[:, 1] >= -0.3) & (points7[:, 1] <= 0.3) &
    #                              (points7[:, 2] >= -0.1) & (points7[:, 2] <= 0.5)]


    
    # points8 = np.asarray(dataset.scans[7])
    # ones_column = np.ones((points8.shape[0], 1))
    # points8 = np.hstack((points8, ones_column))
    # points8 = points8.T
    # points8 = dataset.poses[7] @ real_T_fake @ points8
    # points8 = points8.T
    # points8 = points8[:, :3]
    # filtered_points8 = points8[(points8[:, 0] >= -0.3) & (points8[:, 0] <= 0.3) &
    #                            (points8[:, 1] >= -0.3) & (points8[:, 1] <= 0.3) &
    #                            (points8[:, 2] >= -0.1) & (points8[:, 2] <= 0.5)]

    

    num_points = 10  # 设置需要处理的点的数量
    filtered_points_list = []

    for i in range(1, num_points):
        points = np.asarray(dataset.scans[i])
        ones_column = np.ones((points.shape[0], 1))
        points = np.hstack((points, ones_column))
        points = points.T
        points = grip_T_tcp @ dataset.poses[i] @ real_T_fake @ points
        points = points.T
        points = points[:, :3]
        filtered_points = points[(points[:, 0] >= -0.3) & (points[:, 0] <= 0.3) &
                                (points[:, 1] >= -0.3) & (points[:, 1] <= 0.3) &
                                (points[:, 2] >= -0.1) & (points[:, 2] <= 0.5)]
        filtered_points_list.append(filtered_points)

    # merged_points = o3d.geometry.PointCloud()
    # merged_points.points = o3d.utility.Vector3dVector(np.vstack((filtered_points1, filtered_points2, filtered_points3, filtered_points4, filtered_points5, filtered_points6, filtered_points7, filtered_points8)))
    # o3d.visualization.draw_geometries([merged_points])
    # o3d.io.write_point_cloud("data/test1/res/merged_points.ply", merged_points)


    # pc.points = o3d.utility.Vector3dVector(np.vstack((points1, points2, points5, points6)))

    # o3d.visualization.draw_geometries([pc])

    source_pc = o3d.geometry.PointCloud()
    source_pc.points = o3d.utility.Vector3dVector(filtered_points1)

    # aln_matrices = []

    # for i in range(2, 9):
    #     target_pc = o3d.geometry.PointCloud()



    #     target_pc.points = o3d.utility.Vector3dVector(locals()[f"filtered_points{i}"])
    #     # result = o3d.pipelines.registration.registration_icp(source_pc, target_pc, 0.02, np.eye(4),
    #     result = o3d.pipelines.registration.registration_icp(target_pc, source_pc, 0.02, np.eye(4),
    #         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    #     aln_matrices.append(result.transformation)
    #     ones_column = np.ones((locals()[f"filtered_points{i}"].shape[0], 1))
    #     locals()[f"filtered_points{i}"] = np.hstack((locals()[f"filtered_points{i}"], ones_column))
    #     locals()[f"filtered_points{i}"] = locals()[f"filtered_points{i}"].T
    #     locals()[f"filtered_points{i}"] = aln_matrices[-1] @ locals()[f"filtered_points{i}"]
    #     locals()[f"filtered_points{i}"] = locals()[f"filtered_points{i}"].T
    #     locals()[f"filtered_points{i}"] = locals()[f"filtered_points{i}"][:, :3]
                                                   
    # 创建一个用于存储配准矩阵的列表
    aln_matrices = []

    for i, filtered_points in enumerate(filtered_points_list, start=1):
        target_pc = o3d.geometry.PointCloud()
        target_pc.points = o3d.utility.Vector3dVector(filtered_points)

        result = o3d.pipelines.registration.registration_icp(
            target_pc, source_pc, 0.02, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        aln_matrices.append(result.transformation)

        ones_column = np.ones((filtered_points.shape[0], 1))
        filtered_points = np.hstack((filtered_points, ones_column))
        filtered_points = filtered_points.T
        filtered_points = aln_matrices[-1] @ filtered_points
        filtered_points = filtered_points.T
        filtered_points = filtered_points[:, :3]
        
        # 更新过滤后的点
        filtered_points_list[i - 1] = filtered_points





    # 合并所有过滤后的点
    merged_points_array = np.vstack(filtered_points_list)
    # 创建点云并设置点
    merged_points = o3d.geometry.PointCloud()
    merged_points.points = o3d.utility.Vector3dVector(merged_points_array)

    # merged_points = o3d.geometry.PointCloud()
    # merged_points.points = o3d.utility.Vector3dVector(np.vstack((filtered_points1, filtered_points2, filtered_points3, filtered_points4, filtered_points5, filtered_points6, filtered_points7, filtered_points8)))
    o3d.visualization.draw_geometries([merged_points])


    # o3d.io.write_triangle_mesh("data/test1/res/res.ply", merged_points)



