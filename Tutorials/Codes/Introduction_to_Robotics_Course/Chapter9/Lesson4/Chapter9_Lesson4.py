import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Example: rotate 90 deg about z, then translate by (1,2,0) in frame A
R_AB = Rot.from_euler('z', 90, degrees=True).as_matrix()
p_AB = np.array([1.0, 2.0, 0.0])

T_AB = np.eye(4)
T_AB[0:3, 0:3] = R_AB
T_AB[0:3, 3] = p_AB

# Point in B
x_B = np.array([1.0, 0.0, 0.0])
x_B_h = np.hstack([x_B, 1.0])

# Map to A
x_A_h = T_AB @ x_B_h
x_A = x_A_h[0:3]
print("x_A =", x_A)

# Inverse using block formula
T_BA = np.eye(4)
T_BA[0:3, 0:3] = R_AB.T
T_BA[0:3, 3] = -R_AB.T @ p_AB
print("T_AB @ T_BA =\n", T_AB @ T_BA)
