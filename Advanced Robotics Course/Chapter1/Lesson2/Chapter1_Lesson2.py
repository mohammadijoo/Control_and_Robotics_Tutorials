import numpy as np

# 2-DOF planar arm
L1, L2 = 1.0, 1.0
obstacle_center = np.array([0.8, 0.4])
obstacle_radius = 0.25

def forward_kinematics(q):
    """q = (q1, q2) in radians. Return joint and end-effector positions."""
    q1, q2 = q
    p0 = np.array([0.0, 0.0])
    p1 = np.array([L1 * np.cos(q1), L1 * np.sin(q1)])
    p2 = p1 + np.array([L2 * np.cos(q1 + q2), L2 * np.sin(q1 + q2)])
    return p0, p1, p2

def segment_point_distance(a, b, p):
    """Distance from point p to segment ab in R^2."""
    ab = b - a
    t = np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12)
    t_clamped = np.clip(t, 0.0, 1.0)
    proj = a + t_clamped * ab
    return np.linalg.norm(p - proj)

def in_collision(q):
    """Simple collision: any link too close to the circular obstacle."""
    p0, p1, p2 = forward_kinematics(q)
    d1 = segment_point_distance(p0, p1, obstacle_center)
    d2 = segment_point_distance(p1, p2, obstacle_center)
    return (d1 <= obstacle_radius) or (d2 <= obstacle_radius)

# Sample a grid in C = [-pi, pi] x [-pi, pi]
def sample_cspace(n_per_dim=80):
    q1_vals = np.linspace(-np.pi, np.pi, n_per_dim)
    q2_vals = np.linspace(-np.pi, np.pi, n_per_dim)
    free_mask = np.zeros((n_per_dim, n_per_dim), dtype=bool)
    for i, q1 in enumerate(q1_vals):
        for j, q2 in enumerate(q2_vals):
            if not in_collision((q1, q2)):
                free_mask[i, j] = True
    return q1_vals, q2_vals, free_mask

if __name__ == "__main__":
    q1_vals, q2_vals, free_mask = sample_cspace(n_per_dim=100)
    # Approximate number of connected components by BFS on 4-neighborhood
    visited = np.zeros_like(free_mask, dtype=bool)
    n_components = 0
    for i in range(free_mask.shape[0]):
        for j in range(free_mask.shape[1]):
            if free_mask[i, j] and not visited[i, j]:
                n_components += 1
                stack = [(i, j)]
                visited[i, j] = True
                while stack:
                    ci, cj = stack.pop()
                    for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                        ni, nj = ci + di, cj + dj
                        if 0 <= ni < free_mask.shape[0] and 0 <= nj < free_mask.shape[1]:
                            if free_mask[ni, nj] and not visited[ni, nj]:
                                visited[ni, nj] = True
                                stack.append((ni, nj))
    print("Approximate number of connected components of C_free:", n_components)
      
