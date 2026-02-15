import numpy as np

def planar3_fk(q, link_lengths):
    """
    Simple planar 3R forward kinematics returning end-effector position.
    q: array-like, shape (3,)
    link_lengths: array-like, shape (3,)
    """
    q1, q2, q3 = q
    l1, l2, l3 = link_lengths
    c1 = np.cos(q1); s1 = np.sin(q1)
    c12 = np.cos(q1 + q2); s12 = np.sin(q1 + q2)
    c123 = np.cos(q1 + q2 + q3); s123 = np.sin(q1 + q2 + q3)

    x = l1 * c1 + l2 * c12 + l3 * c123
    y = l1 * s1 + l2 * s12 + l3 * s123
    return np.array([x, y])

def planar3_jacobian(q, link_lengths):
    """
    Jacobian J(q) mapping joint velocities to end-effector linear velocity.
    Shape: (2, 3).
    """
    q1, q2, q3 = q
    l1, l2, l3 = link_lengths
    s1 = np.sin(q1); c1 = np.cos(q1)
    s12 = np.sin(q1 + q2); c12 = np.cos(q1 + q2)
    s123 = np.sin(q1 + q2 + q3); c123 = np.cos(q1 + q2 + q3)

    # Partial derivatives of x, y with respect to q1, q2, q3
    # x = l1 c1 + l2 c12 + l3 c123
    # y = l1 s1 + l2 s12 + l3 s123
    J = np.zeros((2, 3))
    J[0, 0] = -l1 * s1 - l2 * s12 - l3 * s123
    J[1, 0] =  l1 * c1 + l2 * c12 + l3 * c123
    J[0, 1] = -l2 * s12 - l3 * s123
    J[1, 1] =  l2 * c12 + l3 * c123
    J[0, 2] = -l3 * s123
    J[1, 2] =  l3 * c123
    return J

def damped_pseudoinverse(J, lam=1e-3):
    """
    Damped pseudoinverse: J^T (J J^T + lam^2 I)^(-1).
    Suitable for m <= n and full (or nearly full) row rank.
    """
    m, n = J.shape
    JJt = J @ J.T
    return J.T @ np.linalg.inv(JJt + (lam ** 2) * np.eye(m))

def joint_limit_cost(q, q_min, q_max, w=None):
    """
    Quadratic joint-limit cost H(q).
    """
    q_mid = 0.5 * (q_min + q_max)
    span = q_max - q_min
    if w is None:
        w = np.ones_like(q)
    normed = (q - q_mid) / span
    return 0.5 * np.sum(w * normed * normed)

def joint_limit_grad(q, q_min, q_max, w=None):
    """
    Gradient of the joint-limit cost with respect to q.
    """
    q_mid = 0.5 * (q_min + q_max)
    span = q_max - q_min
    if w is None:
        w = np.ones_like(q)
    normed = (q - q_mid) / span
    return w * normed / (span)

def redundancy_resolution_step(q, xdot_des, link_lengths,
                               q_min, q_max,
                               alpha=0.1, lam=1e-3):
    """
    One Euler integration step of redundancy-resolved IK.

    q: current joint configuration, shape (3,)
    xdot_des: desired end-effector velocity (2,)
    """
    J = planar3_jacobian(q, link_lengths)
    J_pinv = damped_pseudoinverse(J, lam=lam)

    # Primary task
    qdot_0 = J_pinv @ xdot_des

    # Null-space projector
    N = np.eye(J.shape[1]) - J_pinv @ J

    # Secondary objective: joint-limit avoidance
    gradH = joint_limit_grad(q, q_min, q_max)
    qdot_H = -alpha * (N @ gradH)

    qdot = qdot_0 + qdot_H
    return qdot

# Example usage (single time step)
if __name__ == "__main__":
    link_lengths = np.array([0.4, 0.3, 0.2])
    q = np.array([0.0, 0.3, -0.2])
    q_min = np.array([-np.pi, -np.pi, -np.pi])
    q_max = np.array([ np.pi,  np.pi,  np.pi])

    # Track a small velocity in x direction
    xdot_des = np.array([0.05, 0.0])

    qdot = redundancy_resolution_step(q, xdot_des, link_lengths,
                                      q_min, q_max,
                                      alpha=0.2, lam=1e-3)
    q_next = q + 0.01 * qdot  # simple Euler step
    print("qdot:", qdot)
    print("q_next:", q_next)
      
