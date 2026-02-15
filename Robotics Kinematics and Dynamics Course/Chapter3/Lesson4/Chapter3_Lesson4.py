import numpy as np
from numpy.linalg import norm

def skew(omega):
    """Return skew-symmetric matrix [omega]_x."""
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def axis_angle_to_R(omega_hat, theta):
    """Axis-angle to rotation matrix via Rodrigues."""
    omega_hat = np.asarray(omega_hat, dtype=float)
    omega_hat = omega_hat / norm(omega_hat)
    K = skew(omega_hat)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

def R_to_axis_angle(R, eps=1e-8):
    """Rotation matrix to axis-angle."""
    R = np.asarray(R, dtype=float)
    # Ensure approximate orthogonality if needed
    trace = np.trace(R)
    # Clamp for numerical safety
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < eps:
        return np.array([1.0, 0.0, 0.0]), 0.0
    if np.pi - theta < 1e-6:
        # Near pi: use diagonal entries
        diag = np.diag(R)
        omega = np.sqrt(np.maximum((diag + 1.0) / 2.0, 0.0))
        # Fix signs using off-diagonals
        if R[0, 1] < 0.0:
            omega[1] = -omega[1]
        if R[0, 2] < 0.0:
            omega[2] = -omega[2]
        omega_hat = omega / norm(omega)
        return omega_hat, theta
    omega_hat = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2.0 * np.sin(theta))
    return omega_hat, theta

def quat_to_R(q):
    """Unit quaternion (w, x, y, z) to rotation matrix."""
    q = np.asarray(q, dtype=float)
    q = q / norm(q)
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
    ])
    return R

def R_to_quat(R):
    """Rotation matrix to quaternion (w, x, y, z)."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(1.0 + t) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # Find the major diagonal term
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= norm(q)
    return q

def euler_zyx_to_R(phi, theta, psi):
    """Euler ZYX (roll, pitch, yaw) to rotation matrix."""
    c1, s1 = np.cos(psi), np.sin(psi)
    c2, s2 = np.cos(theta), np.sin(theta)
    c3, s3 = np.cos(phi), np.sin(phi)
    R = np.array([
        [c1*c2,            s3*s2*c1 - s1*c3,  s3*s1 + s2*c3*c1],
        [s1*c2,            s3*s1*s2 + c3*c1, -s3*c1 + s1*s2*c3],
        [-s2,              s3*c2,             c3*c2]
    ])
    return R

def R_to_euler_zyx(R, eps=1e-8):
    """Rotation matrix to Euler ZYX (roll, pitch, yaw)."""
    R = np.asarray(R, dtype=float)
    r31 = R[2, 0]
    if abs(r31) < 1.0 - eps:
        theta = np.arcsin(-r31)
        phi = np.arctan2(R[2, 1], R[2, 2])
        psi = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock
        if r31 < 0:  # theta ~ +pi/2
            theta = np.pi / 2.0
            phi = 0.0
            psi = np.arctan2(-R[0, 1], R[1, 1])
        else:        # theta ~ -pi/2
            theta = -np.pi / 2.0
            phi = 0.0
            psi = np.arctan2(R[0, 1], R[1, 1])
    return phi, theta, psi

# Example: Using SciPy's Rotation class (if available)
try:
    from scipy.spatial.transform import Rotation as Rsc

    def quat_to_R_scipy(q):
        return Rsc.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

    def R_to_quat_scipy(R):
        r = Rsc.from_matrix(R)
        x, y, z, w = r.as_quat()
        return np.array([w, x, y, z])

except ImportError:
    pass
      
