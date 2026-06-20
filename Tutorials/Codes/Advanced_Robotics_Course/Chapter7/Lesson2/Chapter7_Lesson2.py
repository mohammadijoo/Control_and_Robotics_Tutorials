import numpy as np
from math import atan, cos, sin
from scipy.optimize import linprog

def friction_directions_2d(n, mu):
    """
    Given a 2D unit normal n (pointing into the object) and scalar friction
    coefficient mu, return two unit vectors approximating the friction cone
    boundaries in the contact frame.
    """
    # Tangent vector (2D): rotate normal by +90 degrees
    t = np.array([-n[1], n[0]])
    phi = atan(mu)
    d1 = cos(phi) * n + sin(phi) * t
    d2 = cos(phi) * n - sin(phi) * t
    return [d1, d2]

def build_wrench_matrix_2d(positions, normals, mu):
    """
    positions: list of 2D np.array([x, y])
    normals:   list of unit 2D np.array for each contact
    mu:        scalar friction coefficient
    returns:   G in R^{3 x M}
    """
    wrenches = []
    for p, n in zip(positions, normals):
        dirs = friction_directions_2d(n, mu)
        for d in dirs:
            # primitive force direction d at contact p
            fx, fy = d[0], d[1]
            x, y = p[0], p[1]
            m = x * fy - y * fx  # planar moment about origin
            w = np.array([fx, fy, m])
            wrenches.append(w)
    G = np.stack(wrenches, axis=1)  # shape (3, M)
    return G

def is_force_closure_2d(positions, normals, mu, eps=1e-4):
    """
    Test force closure for planar object using LP:
      find alpha >= eps, sum(alpha) = 1, G alpha = 0.
    Returns True if LP is feasible and rank(G) == 3.
    """
    G = build_wrench_matrix_2d(positions, normals, mu)
    m, M = G.shape

    # Rank condition
    if np.linalg.matrix_rank(G) < 3:
        return False

    # LP: minimize 0 subject to A_eq * alpha = b_eq, bounds
    A_eq = np.vstack([G, np.ones((1, M))])  # shape (4, M)
    b_eq = np.zeros(m + 1)
    b_eq[-1] = 1.0  # sum(alpha) = 1

    c = np.zeros(M)
    bounds = [(eps, None) for _ in range(M)]

    res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method="highs")
    return res.success

if __name__ == "__main__":
    # Example: 3-finger planar grasp on a circle of radius 1
    positions = [
        np.array([1.0, 0.0]),
        np.array([-0.5, np.sqrt(3) / 2.0]),
        np.array([-0.5, -np.sqrt(3) / 2.0])
    ]
    # outward normals pointing roughly outward from the circle
    normals = [p / np.linalg.norm(p) for p in positions]
    mu = 0.8

    fc = is_force_closure_2d(positions, normals, mu)
    print("Force closure:", fc)
      
