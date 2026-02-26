
import numpy as np

def op_space_control_step(q, qd, x_d, xd_d, xdd_d,
                          Kp, Kd,
                          M, h, J, Jdot):
    """
    One step of operational-space control.

    Parameters
    ----------
    q, qd : (n,) arrays
        Joint positions and velocities.
    x_d, xd_d, xdd_d : (m,) arrays
        Desired task position, velocity, acceleration.
    Kp, Kd : (m, m) arrays
        Task-space gain matrices (typically diagonal and positive).
    M : (n, n) array
        Joint-space inertia at q.
    h : (n,) array
        Joint-space nonlinear term C(q,qd)@qd + g(q).
    J : (m, n) array
        Task Jacobian at q.
    Jdot : (m, n) array
        Time derivative of J(q) evaluated at (q, qd).

    Returns
    -------
    tau : (n,) array
        Joint torques for this step.
    """
    # Task-space state
    # Here x, xd should be computed from the robot model; we pass them in or compute outside.
    # Assume we are given current x and xd:
    # x = f(q); xd = J @ qd
    # For modularity, let caller provide x, xd if needed.
    # For this core routine, we derive them from J:
    # WARNING: this assumes x and xd are known; here we only need xd for error derivative.

    # Caller should compute x and xd; for demo:
    # x = x_current
    # xd = J @ qd
    # For this snippet, assume x, xd known outside and precomputed.
    raise_not_implemented = False
    if raise_not_implemented:
        raise NotImplementedError("Provide current x and xd from the kinematics model.")

    # Example: assume x and xd are globals or closure variables (pseudo-code)
    # x, xd = current_task_state(q, qd)

    # Placeholders to indicate interface:
    x = np.zeros_like(x_d)
    xd = np.zeros_like(x_d)

    # Task-space error
    e = x - x_d
    ed = xd - xd_d

    # Desired task acceleration
    xdd_ref = xdd_d - Kd @ ed - Kp @ e

    # Lambda = (J M^-1 J^T)^-1
    Minv = np.linalg.inv(M)
    JMJT = J @ Minv @ J.T
    Lambda = np.linalg.inv(JMJT)

    # Task-space mu and p
    # Split h into Coriolis-like and gravity parts if available.
    # For the structure here we assume h = C(q,qd)@qd + g(q) and the library provides them separately.
    # Here, just demonstrate the structure:
    mu = Lambda @ (J @ Minv @ h) - Lambda @ (Jdot @ qd)

    # If you have g(q) explicitly, replace the next two lines by:
    # h_c = h_coriolis(q, qd)
    # g_vec = g(q)
    # mu = Lambda @ (J @ Minv @ h_c) - Lambda @ (Jdot @ qd)
    # p  = Lambda @ (J @ Minv @ g_vec)

    p = np.zeros_like(x_d)  # placeholder if gravity is already compensated in h

    # Task-space wrench
    F = Lambda @ xdd_ref + mu + p

    # Joint torques
    tau = J.T @ F
    return tau
