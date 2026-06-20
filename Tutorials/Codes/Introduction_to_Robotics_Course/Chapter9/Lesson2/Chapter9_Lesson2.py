import numpy as np

# Example rotation (world from body) and translation
R_WB = np.array([[0.0, -1.0, 0.0],
                 [1.0,  0.0, 0.0],
                 [0.0,  0.0, 1.0]])
t_WB = np.array([1.0, 2.0, 0.5])

# Point in body frame
p_B = np.array([0.3, 0.0, 0.2])

# Transform to world frame
p_W = R_WB @ p_B + t_WB
print("p_W =", p_W)

# Inverse transform (world to body)
p_B_recovered = R_WB.T @ (p_W - t_WB)
print("p_B recovered =", p_B_recovered)
