import numpy as np

def hat_omega(omega):
    """Skew-symmetric matrix from a 3-vector."""
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def vee_omega(omega_hat):
    """Inverse of hat_omega."""
    return np.array([omega_hat[2, 1],
                     omega_hat[0, 2],
                     omega_hat[1, 0]])

def so3_exp(omega, theta):
    """Exponential map for SO(3) given unit axis omega and scalar theta."""
    omega = np.asarray(omega, dtype=float)
    if np.linalg.norm(omega) < 1e-9:
        return np.eye(3)
    omega = omega / np.linalg.norm(omega)
    omega_hat = hat_omega(omega)
    I = np.eye(3)
    return (I
            + np.sin(theta) * omega_hat
            + (1.0 - np.cos(theta)) * (omega_hat @ omega_hat))

def so3_log(R):
    """Logarithm map for SO(3) returning (omega, theta)."""
    R = np.asarray(R, dtype=float)
    assert R.shape == (3, 3)
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3), 0.0
    omega_hat = (theta / (2.0 * np.sin(theta))) * (R - R.T)
    omega = vee_omega(omega_hat)
    return omega / np.linalg.norm(omega), theta

def hat_xi(xi):
    """Hat operator for a 6-vector xi = [v; omega]."""
    xi = np.asarray(xi, dtype=float).reshape(6)
    v = xi[:3]
    omega = xi[3:]
    omega_hat = hat_omega(omega)
    Xi_hat = np.zeros((4, 4))
    Xi_hat[:3, :3] = omega_hat
    Xi_hat[:3, 3] = v
    return Xi_hat

def se3_exp(xi, theta):
    """Exponential map on SE(3): returns a 4x4 homogeneous transform."""
    xi = np.asarray(xi, dtype=float).reshape(6)
    v = xi[:3]
    omega = xi[3:]
    if np.linalg.norm(omega) < 1e-9:
        # pure translation
        T = np.eye(4)
        T[:3, 3] = v * theta
        return T
    omega = omega / np.linalg.norm(omega)
    omega_hat = hat_omega(omega)
    R = so3_exp(omega, theta)
    omega_hat2 = omega_hat @ omega_hat
    I = np.eye(3)
    J = (I * theta
         + (1.0 - np.cos(theta)) * omega_hat
         + (theta - np.sin(theta)) * omega_hat2)
    p = J @ v
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def se3_log(T):
    """Logarithm on SE(3): returns (xi, theta)."""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    p = T[:3, 3]
    omega, theta = so3_log(R)
    if theta < 1e-9:
        # pure translation
        theta = np.linalg.norm(p)
        if theta < 1e-9:
            return np.zeros(6), 0.0
        v = p / theta
        xi = np.hstack((v, np.zeros(3)))
        return xi, theta
    omega_hat = hat_omega(omega)
    omega_hat2 = omega_hat @ omega_hat
    I = np.eye(3)
    J_inv = (I
             - 0.5 * omega_hat
             + (1.0 / (theta ** 2)
                - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta)))
               * omega_hat2)
    v = J_inv @ p
    xi = np.hstack((v, omega))
    return xi, theta
      
