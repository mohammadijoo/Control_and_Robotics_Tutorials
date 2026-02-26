import numpy as np
from scipy.optimize import linear_sum_assignment

# ---------- Cooperative planar grasp ----------

def planar_grasp_matrix(contact_points):
    """
    contact_points: array of shape (m, 2) with rows [x_i, y_i].
    Returns G in R^{3 x 2m}.
    """
    contact_points = np.asarray(contact_points, dtype=float)
    m = contact_points.shape[0]
    G = np.zeros((3, 2 * m))
    for i in range(m):
        x_i, y_i = contact_points[i]
        col = 2 * i
        # Force components
        G[0, col] = 1.0      # f_x_i
        G[1, col + 1] = 1.0  # f_y_i
        # Moment around z: tau_z_i = x_i * f_y_i - y_i * f_x_i
        G[2, col] = -y_i
        G[2, col + 1] = x_i
    return G

# Two symmetric contacts on a bar of length 1.0
contacts = np.array([[0.5, 0.0],
                     [-0.5, 0.0]])

G = planar_grasp_matrix(contacts)
m = contacts.shape[0]
W = np.eye(2 * m)

# Desired wrench: support gravity of an object of mass m_obj at rest
m_obj = 2.0    # kg
g = 9.81       # m/s^2
w_des = np.array([0.0, m_obj * g, 0.0])

# Solve WLS: f_c* = W^{-1} G^T (G W^{-1} G^T)^{-1} w_des
GWG = G @ np.linalg.inv(W) @ G.T
lambda_vec = np.linalg.solve(GWG, w_des)
f_c_star = np.linalg.inv(W) @ G.T @ lambda_vec

print("Grasp matrix G:\n", G)
print("Optimal contact forces f_c*:", f_c_star)

# ---------- Team task allocation via linear_sum_assignment ----------

# Example: 3 robots, 3 tasks, cost = approximate execution time in seconds
C = np.array([
    [4.0, 2.0, 3.5],
    [2.5, 3.0, 2.0],
    [3.0, 4.0, 1.5]
])

row_ind, col_ind = linear_sum_assignment(C)
total_cost = C[row_ind, col_ind].sum()

print("Assignment (robot_i, task_j):")
for i, j in zip(row_ind, col_ind):
    print(f"  robot {i} -> task {j}, cost = {C[i, j]}")

print("Total cost:", total_cost)
      
