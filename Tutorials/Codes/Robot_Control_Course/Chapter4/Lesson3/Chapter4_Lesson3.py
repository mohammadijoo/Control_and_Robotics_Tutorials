
import numpy as np

def pseudoinverse(J, damping=0.0):
    """
    Compute Moore-Penrose or damped pseudoinverse of J (m x n, m <= n).
    """
    m, n = J.shape
    if damping <= 0.0:
        # Standard right pseudoinverse: J^T (J J^T)^{-1}
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt)
    else:
        # Damped pseudoinverse: J^T (J J^T + lambda^2 I)^{-1}
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + (damping ** 2) * np.eye(m))

def null_projector(J, damping=0.0):
    J_sharp = pseudoinverse(J, damping)
    n = J.shape[1]
    return np.eye(n) - J_sharp @ J

def velocity_level_control(q, x, xdot, x_d, xdot_d_ff, Kp, Kd,
                           secondary_grad, k_h=1.0, damping=0.0):
    """
    q: (n,) joint positions
    x: (m,) current task position
    xdot: (m,) current task velocity
    x_d: (m,) desired task position
    xdot_d_ff: (m,) desired feedforward task velocity
    Kp, Kd: (m x m) gains
    secondary_grad: function grad_h(q) returning (n,)
    """
    # User-provided Jacobian J(q) (m x n)
    J = jacobian(q)  # assumed available from kinematics library

    # Task-space velocity command (PD in task space)
    x_error = x_d - x
    xdot_error = xdot_d_ff - xdot
    xdot_d = xdot_d_ff + Kp @ x_error + Kd @ xdot_error

    # Kinematic redundancy resolution
    J_sharp = pseudoinverse(J, damping)
    N = np.eye(J.shape[1]) - J_sharp @ J

    qdot_task = J_sharp @ xdot_d
    grad_h = secondary_grad(q)

    qdot_null = -k_h * grad_h
    qdot = qdot_task + N @ qdot_null
    return qdot

def dynamic_pseudoinverse(J, M):
    """
    Dynamically consistent generalized inverse:
    J_sharp_M = M^{-1} J^T (J M^{-1} J^T)^{-1}
    """
    Minv = np.linalg.inv(M)
    JM = J @ Minv @ J.T
    return Minv @ J.T @ np.linalg.inv(JM)

def dynamic_null_projector(J, M):
    J_sharp_M = dynamic_pseudoinverse(J, M)
    n = J.shape[1]
    return np.eye(n) - J_sharp_M @ J

def torque_level_control(q, qdot, x, xdot,
                         x_d, xdot_d, xddot_d,
                         Kp_task, Kd_task,
                         tau0_func):
    """
    Basic torque-level operational-space controller with null-space torque.

    tau = J^T F_star + N_M^T tau_0
    """
    # Robot model access (from a dynamics library)
    M = inertia_matrix(q)             # (n x n)
    C = coriolis_matrix(q, qdot)      # (n x n) or (n,) depending on convention
    g = gravity_vector(q)             # (n,)

    J = jacobian(q)                   # (m x n)
    Jdot = jacobian_dot(q, qdot)      # (m x n)

    # Operational-space mass matrix: Lambda = (J M^{-1} J^T)^{-1}
    Minv = np.linalg.inv(M)
    Lambda_inv = J @ Minv @ J.T
    Lambda = np.linalg.inv(Lambda_inv)

    # Task-space dynamics terms (simplified):
    # mu = Lambda * (J M^{-1} C qdot - Jdot qdot), p = Lambda * J M^{-1} g
    mu = Lambda @ (J @ Minv @ (C @ qdot) - Jdot @ qdot)
    p = Lambda @ (J @ Minv @ g)

    # Task-space PD+feedforward wrench
    x_error = x_d - x
    xdot_error = xdot_d - xdot
    F_star = Lambda @ (xddot_d + Kp_task @ x_error + Kd_task @ xdot_error) + mu + p

    # Primary torque
    tau_task = J.T @ F_star

    # Null-space projector and secondary torque
    N_M = dynamic_null_projector(J, M)
    tau_0 = tau0_func(q, qdot)  # e.g. posture regulator torque
    tau_null = N_M.T @ tau_0

    tau = tau_task + tau_null
    # Final torque to apply at the actuators
    return tau
