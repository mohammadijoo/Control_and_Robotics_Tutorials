
import numpy as np

def smc_torque(q, dq, q_d, dq_d, ddq_d, Lambda, K, phi):
    """
    Sliding-mode control for joint-space tracking.

    Parameters
    ----------
    q, dq : np.ndarray, shape (n,)
        Current joint position and velocity.
    q_d, dq_d, ddq_d : np.ndarray, shape (n,)
        Desired joint position, velocity, acceleration.
    Lambda : np.ndarray, shape (n, n)
        Positive definite sliding surface gain matrix.
    K : np.ndarray, shape (n, n)
        Positive diagonal switching gain matrix.
    phi : np.ndarray, shape (n,)
        Boundary-layer half-widths (positive entries).
    """
    # Tracking error
    e = q - q_d
    e_dot = dq - dq_d

    # Sliding variable s = e_dot + Lambda * e
    s = e_dot + Lambda @ e

    # Robot model (user-provided or from a robotics library)
    M = M_robot(q)              # shape (n, n)
    C = C_robot(q, dq)          # shape (n, n)
    g = g_robot(q)              # shape (n,)

    # Equivalent control tau_eq
    tau_eq = M @ (ddq_d - Lambda @ e_dot) + C @ dq + g

    # Smooth sign using tanh, approximate sat(s/phi)
    # Avoid elementwise division by zero
    phi_safe = np.maximum(phi, 1e-6)
    sat_arg = s / phi_safe
    # Elementwise hyperbolic tangent
    sat_val = np.tanh(sat_arg)

    tau_sw = -K @ sat_val

    tau = tau_eq + tau_sw
    return tau

# Example of usage inside a control loop (pseudo-code)
def control_loop():
    Lambda = np.diag([15.0, 15.0])   # example for 2 DOF
    K = np.diag([5.0, 5.0])
    phi = np.array([0.05, 0.05])

    while True:
        q, dq = get_joint_state()            # from sensors
        q_d, dq_d, ddq_d = desired_trajectory()
        tau = smc_torque(q, dq, q_d, dq_d, ddq_d, Lambda, K, phi)
        send_joint_torque(tau)               # to low-level driver
