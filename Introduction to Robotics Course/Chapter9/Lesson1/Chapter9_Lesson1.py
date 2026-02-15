import numpy as np

# R_BA: coordinates in A -> coordinates in B
R_BA = np.array([[0, 1],
                 [-1, 0]])  # 90 deg CCW rotation

v_A = np.array([2.0, 1.0])
v_B = R_BA @ v_A

print("v_A =", v_A)
print("v_B =", v_B)
print("lengths:", np.linalg.norm(v_A), np.linalg.norm(v_B))
