import numpy as np

def skew3(omega):
    """Return 3x3 skew-symmetric matrix hat(omega)."""
    wx, wy, wz = omega
    return np.array([
        [0.0, -wz,  wy],
        [wz,  0.0, -wx],
        [-wy, wx,  0.0]
    ])

def exp_twist(xi, q, eps=1e-9):
    """
    Exponential map for a twist xi = (omega, v) in R^6.
    Returns a 4x4 SE(3) matrix exp(hat(xi) q).
    """
    omega = np.asarray(xi[:3]).reshape(3,)
    v = np.asarray(xi[3:]).reshape(3,)
    theta = q
    if np.linalg.norm(omega) < eps:  # prismatic
        R = np.eye(3)
        p = v * theta
    else:
        omega = omega / np.linalg.norm(omega)
        w_hat = skew3(omega)
        R = (
            np.eye(3)
            + np.sin(theta) * w_hat
            + (1.0 - np.cos(theta)) * (w_hat @ w_hat)
        )
        V = (
            np.eye(3) * theta
            + (1.0 - np.cos(theta)) * w_hat
            + (theta - np.sin(theta)) * (w_hat @ w_hat)
        )
        p = V @ v
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T
      
