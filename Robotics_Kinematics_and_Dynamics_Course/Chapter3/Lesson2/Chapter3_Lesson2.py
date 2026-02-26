import numpy as np

def skew(omega: np.ndarray) -> np.ndarray:
    """
    Return the skew-symmetric matrix omega^ from a 3-vector omega.
    """
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def axis_angle_to_R(omega: np.ndarray, theta: float) -> np.ndarray:
    """
    Rodrigues formula: R = I + sin(theta) K + (1 - cos(theta)) K^2
    where K = skew(omega), ||omega|| = 1.
    """
    omega = np.asarray(omega, dtype=float).reshape(3)
    n = np.linalg.norm(omega)
    if n < 1e-12:
        # Identity rotation
        return np.eye(3)
    omega = omega / n
    K = skew(omega)
    I = np.eye(3)
    return I + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)

def R_to_axis_angle(R: np.ndarray):
    """
    Inverse map: given R in SO(3), return (omega, theta).
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    # Ensure R is close to orthogonal if coming from noisy data
    trace = np.trace(R)
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    eps = 1e-8

    if theta < eps:
        # Very small rotation: axis arbitrary, choose x-axis
        return np.array([1.0, 0.0, 0.0]), 0.0

    # Use the skew-symmetric part
    omega = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2.0 * np.sin(theta))

    # Normalize for robustness
    omega = omega / np.linalg.norm(omega)
    return omega, theta

if __name__ == "__main__":
    # Example: rotation of 90 deg about z-axis
    omega = np.array([0.0, 0.0, 1.0])
    theta = np.pi / 2.0
    R = axis_angle_to_R(omega, theta)
    print("R =\n", R)

    omega_rec, theta_rec = R_to_axis_angle(R)
    print("Recovered omega:", omega_rec)
    print("Recovered theta:", theta_rec)
      
