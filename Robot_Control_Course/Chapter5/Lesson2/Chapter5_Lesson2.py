
import numpy as np

def compute_projected_torque(M, h, Jc, bc, tau0, eps=1e-9):
    """
    Equality-constrained torque projection in Python (NumPy).

    Parameters
    ----------
    M : (n, n) array
        Joint-space inertia matrix.
    h : (n,) array
        Nonlinear term h(q, qd).
    Jc : (m, n) array
        Constraint Jacobian.
    bc : (m,) array
        Constraint bias term b_c(q, qd, t).
    tau0 : (n,) array
        Unconstrained torque command.
    eps : float
        Damping for numerical robustness.

    Returns
    -------
    tau : (n,) array
        Constraint-consistent torque.
    lam : (m,) array
        Lagrange multipliers (generalized constraint forces).
    """
    # Solve M x = y using a linear solver for numerical stability
    Minv_tau0_minus_h = np.linalg.solve(M, tau0 - h)
    JMJT = Jc @ np.linalg.solve(M, Jc.T)

    # (J_c M^{-1} J_c^T)^(-1) with small Tikhonov regularization
    JMJT_reg = JMJT + eps * np.eye(JMJT.shape[0])
    Lambda_c = np.linalg.inv(JMJT_reg)

    # Lagrange multipliers enforcing the acceleration constraint
    lam = -Lambda_c @ (Jc @ Minv_tau0_minus_h + bc)

    # Final constrained torque
    tau = tau0 + Jc.T @ lam
    return tau, lam

# Example usage: (Assuming get_M_h_Jc_bc is provided by a robotics library)
def controller_step(q, qd, q_ref, qd_ref, qdd_ref):
    # User-provided functions from dynamics library (e.g., pinocchio)
    M, h = get_M_h(q, qd)
    Jc, bc = get_constraint_jacobian_and_bias(q, qd)

    # Unconstrained computed-torque PD controller
    Kp = np.diag([100.0, 80.0, 60.0])
    Kd = np.diag([20.0,  16.0, 12.0])
    e  = q_ref - q
    ed = qd_ref - qd
    v  = qdd_ref + Kd @ ed + Kp @ e        # desired joint acceleration
    tau0 = M @ v + h                       # inverse-dynamics torque

    tau, lam = compute_projected_torque(M, h, Jc, bc, tau0)
    return tau
