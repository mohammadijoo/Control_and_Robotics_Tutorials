import numpy as np

def unit(v):
    """Return v normalized to unit length."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0.0:
        raise ValueError("Zero vector cannot be normalized.")
    return v / n


def angle_between(a, b):
    """Return the angle between nonzero vectors a and b in radians."""
    a_u = unit(a)
    b_u = unit(b)
    cos_th = np.clip(np.dot(a_u, b_u), -1.0, 1.0)
    return np.arccos(cos_th)


def is_antipodal(p1, n1, p2, n2, mu, tol=1e-3):
    """
    Test the sufficient antipodal condition from Sections 2-3.

    Parameters
    ----------
    p1, p2 : array_like, shape (3,)
        Contact positions in world frame.
    n1, n2 : array_like, shape (3,)
        Outward surface normals at contacts.
    mu : float
        Coulomb friction coefficient.
    tol : float
        Numerical tolerance for angular inequalities.

    Returns
    -------
    bool
        True if the antipodal condition is satisfied.
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    n1 = unit(n1)
    n2 = unit(n2)

    d = unit(p2 - p1)
    phi = np.arctan(mu)

    theta1 = angle_between(-n1, d)
    theta2 = angle_between(n2, -d)
    theta_n = angle_between(n1, -n2)

    return (
        theta1 <= phi + tol
        and theta2 <= phi + tol
        and theta_n >= np.pi - 0.3  # normals roughly opposite
    )


def grasp_map(points, normals):
    """
    Construct a 6 x (3m) grasp map for point contacts with friction.

    For each contact i we build a 6x3 block [I; [p_i]_x] mapping contact-frame
    forces (expressed in world coordinates) into object wrenches.

    Parameters
    ----------
    points : array_like, shape (m, 3)
        Contact positions in object/world frame.
    normals : array_like, shape (m, 3)
        Outward surface normals (not used directly here but typically needed
        for friction-cone discretization).

    Returns
    -------
    G : ndarray, shape (6, 3m)
        Grasp map matrix.
    """
    pts = np.asarray(points, dtype=float)
    m = pts.shape[0]
    G = np.zeros((6, 3 * m))

    def skew(p):
        px, py, pz = p
        return np.array([[0.0, -pz,  py],
                         [pz,  0.0, -px],
                         [-py, px,  0.0]])

    for i, p in enumerate(pts):
        I = np.eye(3)
        px = skew(p)
        block = np.vstack((I, px))  # 6 x 3
        col_start = 3 * i
        col_end = col_start + 3
        G[:, col_start:col_end] = block

    return G


if __name__ == "__main__":
    # Example: two candidate contacts on a box-like object
    p1 = [0.05, 0.0, 0.0]
    p2 = [-0.05, 0.0, 0.0]
    n1 = [-1.0, 0.0, 0.0]
    n2 = [1.0, 0.0, 0.0]
    mu = 0.5

    print("Antipodal:", is_antipodal(p1, n1, p2, n2, mu))

    points = np.array([p1, p2])
    normals = np.array([n1, n2])
    G = grasp_map(points, normals)
    print("Grasp map shape:", G.shape)
      
