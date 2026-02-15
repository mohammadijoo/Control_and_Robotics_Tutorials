import numpy as np

def planar3r_fk(q, lengths):
    """
    Forward kinematics for planar 3R arm (position only).
    q: array-like, shape (3,)
    lengths: array-like, [l1, l2, l3]
    """
    q1, q2, q3 = q
    l1, l2, l3 = lengths
    c1 = np.cos(q1); s1 = np.sin(q1)
    c12 = np.cos(q1 + q2); s12 = np.sin(q1 + q2)
    c123 = np.cos(q1 + q2 + q3); s123 = np.sin(q1 + q2 + q3)
    x = l1 * c1 + l2 * c12 + l3 * c123
    y = l1 * s1 + l2 * s12 + l3 * s123
    return np.array([x, y])

def planar3r_jacobian(q, lengths):
    """
    Planar 3R position Jacobian J_p(q) in R^{2x3}.
    """
    q1, q2, q3 = q
    l1, l2, l3 = lengths
    s1 = np.sin(q1); c1 = np.cos(q1)
    s12 = np.sin(q1 + q2); c12 = np.cos(q1 + q2)
    s123 = np.sin(q1 + q2 + q3); c123 = np.cos(q1 + q2 + q3)

    # Partial derivatives
    dx_dq1 = -l1 * s1 - l2 * s12 - l3 * s123
    dx_dq2 = -l2 * s12 - l3 * s123
    dx_dq3 = -l3 * s123

    dy_dq1 =  l1 * c1 + l2 * c12 + l3 * c123
    dy_dq2 =  l2 * c12 + l3 * c123
    dy_dq3 =  l3 * c123

    J = np.array([[dx_dq1, dx_dq2, dx_dq3],
                  [dy_dq1, dy_dq2, dy_dq3]])
    return J

def minimum_norm_qdot(q, lengths, xdot_des):
    """
    Compute minimum-norm joint velocity for a desired planar velocity.
    xdot_des: array-like, shape (2,)
    """
    J = planar3r_jacobian(q, lengths)
    # Compute pseudoinverse using formula for full-row-rank J (2x3)
    JJt = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt)
    qdot = J_pinv @ xdot_des
    return qdot

def projected_gradient_step(q, lengths, xdot_des, grad_h, alpha=0.1):
    """
    One differential step with a null-space gradient term.
    grad_h(q): function returning gradient of posture cost h(q).
    """
    J = planar3r_jacobian(q, lengths)
    JJt = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt)
    I = np.eye(3)
    P_null = I - J_pinv @ J

    qdot_task = J_pinv @ xdot_des
    qdot_null = -alpha * P_null @ grad_h(q)
    return qdot_task + qdot_null

# Example usage
if __name__ == "__main__":
    lengths = np.array([0.5, 0.4, 0.3])
    q = np.array([0.2, 0.5, -0.4])
    xdot_des = np.array([0.05, -0.02])

    # Simple quadratic posture objective: keep q near q0
    q0 = np.array([0.0, 0.0, 0.0])
    def grad_h(q):
        return q - q0  # gradient of (1/2)||q - q0||^2

    qdot = projected_gradient_step(q, lengths, xdot_des, grad_h, alpha=0.2)
    print("qdot:", qdot)
      
