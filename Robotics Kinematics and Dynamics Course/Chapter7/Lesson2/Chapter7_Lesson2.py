import numpy as np

def skew3(w):
    """Return 3x3 skew-symmetric matrix of a 3-vector."""
    wx, wy, wz = w
    return np.array([[0.0, -wz,  wy],
                     [wz,   0.0, -wx],
                     [-wy,  wx,  0.0]])

def adjoint(T):
    """Adjoint matrix Ad_T for T in SE(3)."""
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_hat = skew3(p)
    upper = np.hstack((R, np.zeros((3, 3))))
    lower = np.hstack((p_hat @ R, R))
    return np.vstack((upper, lower))

def exp_twist(xi, theta):
    """Matrix exponential exp([xi]^ wedge * theta) in SE(3)."""
    w = xi[0:3]
    v = xi[3:6]
    w_norm = np.linalg.norm(w)
    if w_norm < 1e-9:  # prismatic
        R = np.eye(3)
        p = v * theta
    else:  # revolute
        w_unit = w / w_norm
        w_hat = skew3(w_unit)
        R = (np.eye(3)
             + np.sin(theta * w_norm) * w_hat
             + (1.0 - np.cos(theta * w_norm)) * (w_hat @ w_hat))
        # Rodrigues formula for translation
        V = (np.eye(3) * theta
             + (1.0 - np.cos(theta * w_norm)) / w_norm * w_hat
             + (theta * w_norm - np.sin(theta * w_norm)) / (w_norm**2) * (w_hat @ w_hat))
        p = V @ v
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def spatial_jacobian(xi_list, q):
    """
    Compute spatial Jacobian Js(q).
    xi_list: list of 6-vectors (twists) in space frame.
    q: numpy array of joint coordinates.
    """
    n = len(xi_list)
    Js = np.zeros((6, n))
    T_prev = np.eye(4)
    for j in range(n):
        if j == 0:
            Js[:, j] = xi_list[0]
        else:
            Ad = adjoint(T_prev)
            Js[:, j] = Ad @ xi_list[j]
        # update T_prev = T_prev * exp(xi_j * q_j)
        T_prev = T_prev @ exp_twist(xi_list[j], q[j])
    return Js

# 2R planar example (axes along z, home pose with q1 = q2 = 0)
l1, l2 = 1.0, 0.8

# Joint 1 at origin, axis z
xi1 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

# Joint 2 at (l1, 0, 0), axis z
q2_point = np.array([l1, 0.0, 0.0])
omega = np.array([0.0, 0.0, 1.0])
v2 = -np.cross(omega, q2_point)
xi2 = np.hstack((omega, v2))

xi_list = [xi1, xi2]
q = np.array([0.5, -0.3])

Js = spatial_jacobian(xi_list, q)
print("Spatial Jacobian Js(q) for 2R planar:")
print(Js)
      
