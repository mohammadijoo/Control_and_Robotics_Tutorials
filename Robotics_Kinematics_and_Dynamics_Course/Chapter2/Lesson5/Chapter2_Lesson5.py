import numpy as np
from numpy.linalg import norm
from scipy.linalg import expm

def hat_so3(omega):
    """Map R^3 to so(3)."""
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def hat_se3(xi):
    """Map R^6 (omega, v) to se(3)."""
    omega = xi[:3]
    v = xi[3:]
    Xi_hat = np.zeros((4, 4))
    Xi_hat[:3, :3] = hat_so3(omega)
    Xi_hat[:3, 3] = v
    return Xi_hat

def exp_se3(xi, theta=1.0):
    """
    Exponential map for se(3) using closed form for screw motions.
    xi = (omega, v), theta is motion parameter.
    """
    omega = xi[:3]
    v = xi[3:]
    th = theta

    if norm(omega) < 1e-9:
        # Pure translation
        g = np.eye(4)
        g[:3, 3] = v * th
        return g

    # Normalize
    w = omega / norm(omega)
    w_hat = hat_so3(w)
    th = th * norm(omega)

    R = expm(w_hat * th)

    I = np.eye(3)
    A = I * th + (1.0 - np.cos(th)) * w_hat + (th - np.sin(th)) * (w_hat @ w_hat)
    p = A @ v

    g = np.eye(4)
    g[:3, :3] = R
    g[:3, 3] = p
    return g

def adjoint_SE3(g):
    """
    Compute Ad_g for SE(3).
    g is 4x4 homogeneous transform.
    """
    R = g[:3, :3]
    p = g[:3, 3]
    p_hat = hat_so3(p)

    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, :3] = p_hat @ R
    Ad[3:, 3:] = R
    return Ad

if __name__ == "__main__":
    omega = np.array([0.0, 0.0, 1.0])
    v = np.array([0.1, 0.2, 0.0])
    xi = np.hstack((omega, v))

    g = exp_se3(xi, theta=0.5)
    print("g(theta)=\\n", g)

    Adg = adjoint_SE3(g)
    print("Ad_g=\\n", Adg)
      
