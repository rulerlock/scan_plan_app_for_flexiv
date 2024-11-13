import numpy as np
from scipy.spatial.transform import Rotation as R


# Function to transform matrix to position + orientation (xyzw)
def matrix_to_pos_ori(matrix):
    position = matrix[:3, 3]
    rotation = R.from_matrix(matrix[:3, :3])
    orientation = rotation.as_quat()
    return position, orientation

# Transformation matrix from robot arm tool plane to flange plane (Ttcp_flg)
tcp_T_flange = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -0.3],  # Assuming the flange is 0.1 meters along the z-axis of the tool plane
    [0, 0, 0, 1]
])

# Position at grinder in form of position xyz and orientation (x, y, z, w)
position_grinder = np.array([0.53, -0.19, 0.2057])
orientation_grinder = np.array([0, 1, 0, 0])  # Quaternion (x, y, z, w)

# Convert table position and orientation to transformation matrix
grinder_pos = np.eye(4)
grinder_pos[:3, 3] = position_grinder
grinder_pos[:3, :3] = R.from_quat(orientation_grinder).as_matrix()

# Transformation matrix from table position to flange
parts_T_tcp = np.array([
    [9.99961830e-01, -3.79781104e-06, 8.73715397e-03, -3.05465130e-02],
    [-3.79781104e-06, 9.99999622e-01, 8.69330644e-04, -2.01453593e-02],
    [-8.73715397e-03, -8.69330644e-04, 9.99961452e-01, 1.86599632e-01],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

# Combine transformations
parts_T_flange = parts_T_tcp @ tcp_T_flange

# Calculate the transformation from table position to flange
# position_flange, orientation_flange = matrix_to_pos_ori(parts_T_flange)

# Calculate the trimming pos, i.e. transformation from grinder at grinding point to flange
trim_pos = grinder_pos @ parts_T_flange

# Extract position and orientation from the resulting transformation matrix
position_flange, orientation_flange = matrix_to_pos_ori(trim_pos)

print("Position at flange:", position_flange)
print("Orientation at flange (x,y,z,w):", orientation_flange)