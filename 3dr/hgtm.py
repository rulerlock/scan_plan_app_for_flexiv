import numpy as np

def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    rotation_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2,   2*qx*qy - 2*qz*qw,       2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,       1 - 2*qx**2 - 2*qz**2,   2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,       2*qy*qz + 2*qx*qw,       1 - 2*qx**2 - 2*qy**2]
    ])
    return rotation_matrix

def homogeneous_transform(position, quaternion):
    x, y, z = position
    qw, qx, qy, qz = quaternion
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    translation_vector = np.array([[x], [y], [z]])
    homogeneous_matrix = np.block([
        [rotation_matrix, translation_vector],
        [np.zeros((1, 3)), 1]
    ])
    return homogeneous_matrix

# Example usage:
if __name__ == "__main__":
    p=[0.32, -0.06, 0.54, 0.78, 0.60, -0.14, 0.09]
    position = [0.4, 0, 0.2]
    quaternion = [0,1,0,0]
    homogeneous_matrix = homogeneous_transform(p[0:3], p[3:7])
    #homogeneous_matrix = homogeneous_transform(position, quaternion)
    print("Homogeneous Transformation Matrix:")
    print(homogeneous_matrix)
    hm = np.array(homogeneous_matrix)
    a = [[1, 0, 0, 0.32],[0, 0, 1, -0.54], [0, -1, 0, 0.466], [0,0,0,1]]
    b = np.array(a)
    print(np.dot(hm, b))
