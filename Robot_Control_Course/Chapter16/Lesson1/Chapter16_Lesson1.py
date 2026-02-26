
import numpy as np

def compute_errors(q, q_d):
    """
    q, q_d: arrays of shape (N, n_joints)
    returns e, e_norm2
    """
    e = q - q_d
    e_norm2 = np.sum(e**2, axis=1)
    return e, e_norm2

def compute_time_domain_metrics(t, e_scalar, eps=0.02, r_step=1.0):
    """
    e_scalar: 1D array of tracking error for a single joint.
    eps: settling band as fraction of step amplitude.
    r_step: step amplitude (for overshoot and steady-state error).
    """
    dt = np.diff(t)
    # overshoot
    Mp = np.max(np.abs(e_scalar)) / abs(r_step) * 100.0

    # steady-state error (last sample)
    e_ss = e_scalar[-1]

    # settling time: first instant after which error remains within eps*|r|
    band = eps * abs(r_step)
    settled_idx = None
    for k in range(len(e_scalar)):
        if np.all(np.abs(e_scalar[k:]) <= band):
            settled_idx = k
            break
    t_s = t[settled_idx] if settled_idx is not None else np.inf

    return {"Mp_percent": Mp, "e_ss": e_ss, "t_s": t_s}

def compute_integral_metrics(t, e_norm2, u):
    """
    e_norm2: ||e(t)||^2 at sampled times.
    u: array of shape (N, n_joints) with control torques.
    """
    # ISE
    J_ISE = np.trapz(e_norm2, t)

    # IAE (using L1 norm)
    e_norm1 = np.sum(np.abs(e_norm2**0.5), axis=0) if e_norm2.ndim > 1 else np.abs(e_norm2)
    J_IAE = np.trapz(e_norm1, t)

    # control effort (ISU)
    u_norm2 = np.sum(u**2, axis=1)
    J_ISU = np.trapz(u_norm2, t)

    return {"J_ISE": J_ISE, "J_IAE": J_IAE, "J_ISU": J_ISU}

# Example usage with simulated data (e.g. from roboticstoolbox)
# t, q_d, q, tau = simulate_controller(...)
# e, e_norm2 = compute_errors(q, q_d)
# metrics_td = compute_time_domain_metrics(t, e[:, 0])  # first joint
# metrics_int = compute_integral_metrics(t, e_norm2, tau)
