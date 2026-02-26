import numpy as np
from numpy.linalg import svd, norm

# -------------------------------------------------------
# Contact and wrench utilities
# -------------------------------------------------------

def skew(p):
    """Return 3x3 skew-symmetric matrix for p in R^3."""
    px, py, pz = p
    return np.array([[0.0, -pz,  py],
                     [pz,  0.0, -px],
                     [-py, px,  0.0]])

def grasp_map_block(p_i, R_i):
    """
    Build 6x3 grasp map block G_i for a point contact at p_i (object frame),
    with local-to-object rotation R_i.
    """
    upper = R_i
    lower = skew(p_i) @ R_i
    return np.vstack((upper, lower))

def linearized_friction_directions(mu, k_dirs=8):
    """
    Build k_dirs generator directions for a friction cone with coefficient mu.
    Returns 3 x k_dirs matrix whose columns are unit directions in local frame.
    Normal is along +z in local coordinates.
    """
    dirs = []
    # Tangential radius in local coordinates
    alpha = np.arctan(mu)
    # Normal component cos(alpha), tangential radius sin(alpha)
    fn = np.cos(alpha)
    ft = np.sin(alpha)
    for j in range(k_dirs):
        theta = 2.0 * np.pi * j / k_dirs
        tx = ft * np.cos(theta)
        ty = ft * np.sin(theta)
        # Normal along z-axis
        dirs.append(np.array([tx, ty, fn]))
    return np.column_stack(dirs)

def primitive_wrenches(p_list, R_list, mu_list, k_dirs=8):
    """
    Compute primitive wrenches for all contacts.
    p_list: list of 3-vectors (object frame contact points)
    R_list: list of 3x3 rotation matrices (local->object)
    mu_list: list of friction coefficients
    Returns:
      W: 6 x N matrix of primitive wrenches
    """
    W_cols = []
    for p_i, R_i, mu_i in zip(p_list, R_list, mu_list):
        G_i = grasp_map_block(p_i, R_i)
        D_i = linearized_friction_directions(mu_i, k_dirs=k_dirs)
        # Each column of D_i gives a primitive wrench
        W_i = G_i @ D_i
        for j in range(W_i.shape[1]):
            W_cols.append(W_i[:, j])
    return np.column_stack(W_cols)  # 6 x N

# -------------------------------------------------------
# Matrix-based metric from G
# -------------------------------------------------------

def grasp_map_from_primitive(W):
    """
    Construct a simple contact-force->wrench map from primitive wrenches W.
    Here we treat each primitive direction as an independent actuator
    (purely for quality metrics like isotropy).
    W: 6 x N matrix of primitive wrenches
    """
    # If primitive forces are unit-norm, G can be approximated by W itself
    return W

def isotropy_metric(G):
    """
    Q_iso = sigma_min(G) / sigma_max(G).
    """
    U, s, Vt = svd(G, full_matrices=False)
    s_min, s_max = np.min(s), np.max(s)
    if s_max < 1e-12:
        return 0.0
    return float(s_min / s_max)

# -------------------------------------------------------
# Sampled epsilon metric approximation
# -------------------------------------------------------

def epsilon_metric_sampled(W, n_directions=2000, rng=None):
    """
    Approximate epsilon metric by sampling directions on the unit sphere in R^6.
    W: 6 x N matrix of primitive wrenches (defining conv(W))
    Returns:
      q_eps: approximate epsilon value
    """
    if rng is None:
        rng = np.random.default_rng()

    # Normalize primitive wrenches (for a unit "effort" polytope)
    Wn = W / np.maximum(norm(W, axis=0, keepdims=True), 1e-12)

    q_eps = np.inf
    for _ in range(n_directions):
        v = rng.normal(size=6)
        v /= norm(v)
        # Support value in direction v: max_k v^T w_k
        support = np.max(v @ Wn)
        q_eps = min(q_eps, support)
    return float(q_eps)

# -------------------------------------------------------
# Example usage
# -------------------------------------------------------

if __name__ == "__main__":
    # Example: 3-finger grasp on a spherical object
    # Contact points (object frame)
    p_list = [
        np.array([0.05, 0.0, 0.0]),
        np.array([-0.025, 0.043301, 0.0]),
        np.array([-0.025, -0.043301, 0.0]),
    ]
    # Local normals pointing approximately toward the origin
    R_list = []
    for p in p_list:
        n = -p / (norm(p) + 1e-12)
        # Build a simple frame with n as z-axis
        z = n
        # Pick arbitrary x-axis not parallel to z
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(tmp, z)) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0])
        x = tmp - np.dot(tmp, z) * z
        x /= norm(x)
        y = np.cross(z, x)
        R = np.column_stack((x, y, z))
        R_list.append(R)

    mu_list = [0.8, 0.8, 0.8]

    W = primitive_wrenches(p_list, R_list, mu_list, k_dirs=8)
    G = grasp_map_from_primitive(W)

    q_iso = isotropy_metric(G)
    q_eps = epsilon_metric_sampled(W, n_directions=2000)

    print("Isotropy metric Q_iso:", q_iso)
    print("Approximate epsilon metric Q_eps:", q_eps)
      
