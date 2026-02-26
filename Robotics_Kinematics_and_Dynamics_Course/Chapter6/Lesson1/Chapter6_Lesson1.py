import numpy as np

def wrap_to_pi(angle):
    """Wrap angle to (-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def ik_2r(x, y, l1, l2, joint_limits=None):
    """
    Analytic IK for a planar 2R arm.

    Parameters
    ----------
    x, y : float
        Desired end-effector position.
    l1, l2 : float
        Link lengths (assumed > 0).
    joint_limits : tuple (lower, upper), optional
        Each is a length-2 array-like of joint limits [theta1, theta2].

    Returns
    -------
    solutions : list of np.ndarray
        Each entry is [theta1, theta2].
    """
    r2 = x**2 + y**2
    # Existence check using cosine law
    c2 = (r2 - l1**2 - l2**2) / (2.0 * l1 * l2)

    if np.abs(c2) > 1.0:
        # No real solution
        return []

    # Two possible signs for sin(theta2)
    s2_pos = np.sqrt(max(0.0, 1.0 - c2**2))
    s2_neg = -s2_pos

    solutions = []
    for s2 in (s2_pos, s2_neg):
        theta2 = np.arctan2(s2, c2)
        k1 = l1 + l2 * c2
        k2 = l2 * s2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        theta1 = wrap_to_pi(theta1)
        theta2 = wrap_to_pi(theta2)
        q = np.array([theta1, theta2])

        if joint_limits is not None:
            lower = np.array(joint_limits[0])
            upper = np.array(joint_limits[1])
            if np.any(q < lower) or np.any(q > upper):
                continue

        solutions.append(q)

    # Remove duplicates in the degenerate boundary case
    unique = []
    for q in solutions:
        if not any(np.allclose(q, q2, atol=1e-9) for q2 in unique):
            unique.append(q)

    return unique

if __name__ == "__main__":
    sols = ik_2r(0.5, 0.5, 0.4, 0.4,
                 joint_limits=([-np.pi, -np.pi], [np.pi, np.pi]))
    print("Number of solutions:", len(sols))
    for idx, q in enumerate(sols):
        print(f"Solution {idx}: theta1={q[0]:.3f}, theta2={q[1]:.3f}")
      
