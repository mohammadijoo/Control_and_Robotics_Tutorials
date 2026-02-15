import numpy as np

def rotm_to_quat(R):
    """Convert rotation matrix to quaternion [w, x, y, z]."""
    # Assumes R is a proper rotation
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0  # 4 * w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # Compute largest diagonal element for stability
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
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
    return q / np.linalg.norm(q)

def continuous_quat(q_prev, q_new):
    """Flip sign of q_new if needed to keep continuity with q_prev."""
    if np.dot(q_prev, q_new) < 0.0:
        q_new = -q_new
    return q_new / np.linalg.norm(q_new)

def rotm_to_euler_zyx(R, eps=1e-6):
    """Rotation matrix to ZYX Euler, with near-gimbal detection."""
    # theta is pitch
    theta = -np.arcsin(R[2, 0])
    cos_theta = np.cos(theta)
    near_gimbal = abs(cos_theta) < eps

    if not near_gimbal:
        phi = np.arctan2(R[1, 0], R[0, 0])    # yaw
        psi = np.arctan2(R[2, 1], R[2, 2])    # roll
    else:
        # In gimbal lock, we choose a convention (e.g. set phi = 0)
        phi = 0.0
        if theta > 0:
            psi = np.arctan2(R[0, 1], R[1, 1])
        else:
            psi = -np.arctan2(R[0, 1], R[1, 1])

    return phi, theta, psi, near_gimbal

# Example usage
R = np.eye(3)
q_prev = np.array([1.0, 0.0, 0.0, 0.0])
q_raw = rotm_to_quat(R)
q_cont = continuous_quat(q_prev, q_raw)
phi, theta, psi, near_gimbal = rotm_to_euler_zyx(R)
print("q_cont:", q_cont)
print("Euler (phi, theta, psi):", phi, theta, psi, "near_gimbal:", near_gimbal)
      
