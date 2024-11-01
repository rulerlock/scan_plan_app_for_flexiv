# Dataset class for vdb fusion
import numpy as np 
import open3d as o3d
import re
import os
import copy


class Dataset:
    def __init__(self, data_path) -> None:
        self.use_clustered = False
        self.data_path = data_path
        self.poses = self.read_txt_file(data_path)
        self.scans = self.read_ply_file(data_path)
        

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
    

def clustering_process(point_cloud, threshold=0.04, minpoints=20):

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

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return pcd_down, pcd_fpfh


def pairwise_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def refine_registration(source, target, voxel_size, trans_init):
    distance_threshold = voxel_size * 0.4
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def pairwise_global_registration(pcds, voxel_size):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            source = pcds[source_id]
            target = pcds[target_id]
            source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                source,
                o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size, max_nn=100))
            target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                target,
                o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size, max_nn=100))
            
            result = pairwise_registration(source, target, source_fpfh, target_fpfh, voxel_size)
            
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(result.transformation, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             result.transformation,
                                                             
                                                             uncertain=False,
                                                             confidence=result.fitness))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             result.transformation,
                                                             
                                                             uncertain=True,
                                                             confidence=result.fitness))
    return pose_graph


def optimize_posegraph_for_global_registration(pose_graph):
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=0.03,
        edge_prune_threshold=0.25,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)


def unionset_tri(tri):
    parent = {}
    for i in range(len(tri)):
        a = tri[i].tolist()
        a.sort()
        cur_p = []
        for x in a:
            if x not in parent:
                parent[x] = a[0]
                cur_p.append(a[0])
            else:
                cur_p.append(parent[x])
        cur_p = list(set(cur_p))
        cur_p.sort()
        if len(cur_p) > 1:
            for k in parent.keys():
                if parent[k] in cur_p:
                    parent[k] = cur_p[0]
    return parent


if __name__ == "__main__":
    import vdbfusion
    vdb_volume = vdbfusion.VDBVolume(voxel_size=0.004,
                                 sdf_trunc=0.015,
                                 space_carving=False)


    dataset = Dataset('data/scan0919')
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
    real_T_fake = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

    threshold = 0.03


    points = []
    clustered_points = []
    filtered_points = []

    if dataset.use_clustered:
        clustered_points = copy.deepcopy(dataset.scans)
    else:
        for i in range(len(dataset.scans)):
            pc = o3d.geometry.PointCloud()
            cur_points = np.asarray(dataset.scans[i])
            # pc.points = o3d.utility.Vector3dVector(cur_points)
            # o3d.visualization.draw_geometries([pc], window_name=f"3D Point Cloud {i}")
            ones_column = np.ones((cur_points.shape[0], 1))
            
            cur_points = np.hstack((cur_points, ones_column))
            cur_points = cur_points.T
            cur_points = dataset.poses[i] @ real_T_fake @ cur_points
            # cur_points = np.linalg.inv(dataset.poses[i]) @ real_T_fake @ cur_points
            cur_points = cur_points.T
            cur_points = cur_points[:, :3]
            points.append(cur_points)
            filtered_cur_points = cur_points[(cur_points[:, 0] >= -0.3) & (cur_points[:, 0] <= 0.3) &
                                                (cur_points[:, 1] >= -0.3) & (cur_points[:, 1] <= 0.3) &
                                                (cur_points[:, 2] >= -0.16) & (cur_points[:, 2] <= 0.3)]
            filtered_points.append(filtered_cur_points)
            clustered_cur_points = np.array(clustering_process(filtered_cur_points, threshold=threshold))
            clustered_points.append(clustered_cur_points)

        dataset.store_clustered(clustered_points)


    all_points = np.vstack(clustered_points)
    pc.points = o3d.utility.Vector3dVector(all_points)
    o3d.visualization.draw_geometries([pc], window_name="3D Point Cloud")

    for i in range(len(clustered_points)):
        if i == 12:
            continue
        vdb_volume.integrate(clustered_points[i], dataset.poses[i]@real_T_fake)
    vert, tri = vdb_volume.extract_triangle_mesh()

    connectivity = unionset_tri(tri)
    grouped_points = {}
    for key in connectivity.keys():
        if connectivity[key] not in grouped_points:
            grouped_points[connectivity[key]] = []
        grouped_points[connectivity[key]].append(key)
    grouped_points_list = list(grouped_points.values())
    grouped_points_list = sorted(grouped_points_list, key=lambda x: len(x), reverse=True)
    clustered_tri = []
    for i in range(len(tri)):
        if tri[i][0] in grouped_points_list[0]:
            clustered_tri.append(tri[i])
    clustered_tri = np.array(clustered_tri)

    


    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vert),
        # o3d.utility.Vector3iVector(tri),
        # o3d.utility.Vector3dVector(clustered_vert),
        o3d.utility.Vector3iVector(clustered_tri),
    )
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh("data/scan0919/res/mesh_ori.ply", mesh)


    voxel_size = 0.01

    source_points = clustered_points[0]

    source_pc = o3d.geometry.PointCloud()
    source_pc.points = o3d.utility.Vector3dVector(source_points)
    o3d.visualization.draw_geometries([source_pc])

    for i in range(1, len(clustered_points)):
        target_pc = o3d.geometry.PointCloud()
        target_pc.points = o3d.utility.Vector3dVector(clustered_points[i])
        result = o3d.pipelines.registration.registration_icp(target_pc, source_pc, 0.02, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        ones_column = np.ones((clustered_points[i].shape[0], 1))
        clustered_points[i] = np.hstack((clustered_points[i], ones_column))
        clustered_points[i] = clustered_points[i].T
        clustered_points[i] = result.transformation @ clustered_points[i]
        clustered_points[i] = clustered_points[i].T
        clustered_points[i] = clustered_points[i][:, :3]
        source_points = np.vstack((source_points, clustered_points[i]))
        source_pc = o3d.geometry.PointCloud()
        source_pc.points = o3d.utility.Vector3dVector(source_points)
        o3d.visualization.draw_geometries([source_pc])


    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(source_points)
    o3d.visualization.draw_geometries([pc])


    # clustered_points_o3dinstance = []

    # for i in range(len(clustered_points)):
    #     pc = o3d.geometry.PointCloud()
    #     pc.points = o3d.utility.Vector3dVector(clustered_points[i])
    #     pc.estimate_normals(
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
    #     clustered_points_o3dinstance.append(pc)

    # pose_graph = pairwise_global_registration(clustered_points_o3dinstance, voxel_size)
    # optimize_posegraph_for_global_registration(pose_graph)

    # for point_id in range(len(clustered_points_o3dinstance)):
    #     print(pose_graph.nodes[point_id].pose)
    #     clustered_points_o3dinstance[point_id].transform(pose_graph.nodes[point_id].pose)
    
    # # o3d.visualization.draw_geometries(down_sampled_points)
    # o3d.visualization.draw_geometries(clustered_points_o3dinstance)

        

    


