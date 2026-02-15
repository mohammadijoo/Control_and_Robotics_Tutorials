import numpy as np

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)

def quat_conjugate(q):
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_multiply(q1, q2):
    """Hamilton product q = q1 ⊗ q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def quat_from_axis_angle(axis, theta):
    """Axis is a 3D vector, theta in radians."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * theta
    s = np.sin(half)
    return np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s])

def quat_rotate(q, v):
    """Rotate 3D vector v by unit quaternion q."""
    q = quat_normalize(q)
    v = np.asarray(v, dtype=float)
    vq = np.concatenate([[0.0], v])
    return quat_multiply(quat_multiply(q, vq), quat_conjugate(q))[1:]

def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between q0 and q1."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))

    # Use the shortest arc on S3
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # Clamp to handle numerical drift
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot)

    # If the angle is tiny, fall back to normalized lerp
    if theta < 1e-6:
        q = (1.0 - t) * q0 + t * q1
        return quat_normalize(q)

    sin_theta = np.sin(theta)
    w0 = np.sin((1.0 - t) * theta) / sin_theta
    w1 = np.sin(t * theta) / sin_theta
    q = w0 * q0 + w1 * q1
    return quat_normalize(q)

# Example: interpolate between two axis-angle orientations
if __name__ == "__main__":
    q0 = quat_from_axis_angle([0, 0, 1], 0.0)
    q1 = quat_from_axis_angle([0, 0, 1], np.deg2rad(90.0))
    t = 0.5
    qm = quat_slerp(q0, q1, t)
    print("Mid quaternion:", qm)
    v = np.array([1.0, 0.0, 0.0])
    print("Rotated vector at t=0.5:", quat_rotate(qm, v))
      
