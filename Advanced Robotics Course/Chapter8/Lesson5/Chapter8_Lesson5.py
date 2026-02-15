import numpy as np
import trimesh

def skew(v):
    """Return skew-symmetric matrix for cross product."""
    x, y, z = v
    return np.array([[0.0, -z,   y],
                     [z,   0.0, -x],
                     [-y,  x,   0.0]])

def build_grasp_map(contact_points):
    """
    contact_points: list of 3D np.array, p_i in object frame.
    Parallel-jaw grasp with point contacts (for now no torsional friction).
    """
    m = len(contact_points)
    G_top = np.zeros((3, 3 * m))
    G_bottom = np.zeros((3, 3 * m))
    for i, p in enumerate(contact_points):
        idx = 3 * i
        G_top[:, idx:idx+3] = np.eye(3)
        G_bottom[:, idx:idx+3] = skew(p)
    G = np.vstack((G_top, G_bottom))
    return G

def epsilon_surrogate(G):
    """Use smallest singular value as surrogate quality."""
    s = np.linalg.svd(G, compute_uv=False)
    return float(np.min(s))

# Example: load a box mesh and define a simple two-finger grasp
mesh = trimesh.creation.box(extents=(0.1, 0.06, 0.04))

# Two opposing contacts on +y and -y faces
p1 = np.array([0.0,  0.03, 0.0])
p2 = np.array([0.0, -0.03, 0.0])

G = build_grasp_map([p1, p2])
q = epsilon_surrogate(G)
print("Quality surrogate sigma_min(G):", q)

def rank_and_filter_grasps(candidates, min_quality=1e-3):
    """
    candidates: list of lists of contact points
    Returns: list of (candidate, quality) sorted by quality descending.
    """
    scored = []
    for contacts in candidates:
        G = build_grasp_map(contacts)
        q = epsilon_surrogate(G)
        if q >= min_quality:
            scored.append((contacts, q))
    scored.sort(key=lambda x: x[1], reverse=True)
    return scored
      
