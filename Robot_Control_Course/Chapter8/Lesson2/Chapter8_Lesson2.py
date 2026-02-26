
import numpy as np

# Controller gains
Lambda = 10.0
k_D = 5.0

# Adaptation gain matrix (diagonal for simplicity)
Gamma = np.diag([5.0, 1.0, 1.0])

def regressor_1dof(q, qd, qrd, qrdd):
    """
    Y(q, qd, qrd, qrdd) for 1-DOF rotary joint.
    Here we approximate qrd by qd_r and qrdd by qdd_r.
    """
    return np.array([[qrdd, qrd, np.cos(q)]])  # shape (1,3)

def reference_signals(q, qd, q_d, qd_d, qdd_d, Lambda):
    tilde_q = q - q_d
    tilde_qd = qd - qd_d
    qrd = qd_d - Lambda * tilde_q      # q_rdot
    qrdd = qdd_d - Lambda * tilde_qd   # q_rddot
    s = qd - qrd
    return qrd, qrdd, s, tilde_q, tilde_qd

def adaptive_ct_step(q, qd, theta_hat, q_d, qd_d, qdd_d, dt):
    # Compute reference quantities
    qrd, qrdd, s, tilde_q, tilde_qd = reference_signals(q, qd, q_d, qd_d, qdd_d, Lambda)

    # Build regressor and torque
    Y = regressor_1dof(q, qd, qrd, qrdd)         # (1,3)
    tau = float(Y @ theta_hat + k_D * s)         # scalar torque

    # Simple gradient adaptation law (detailed Lyapunov design in next lesson)
    # theta_hat_dot = -Gamma @ Y.T * s
    theta_hat_dot = -Gamma @ (Y.T * s)
    theta_hat_next = theta_hat + theta_hat_dot * dt

    return tau, theta_hat_next, s

# Example simulation loop
def simulate_adaptive_ct(T=10.0, dt=0.001):
    n_steps = int(T / dt)
    # True parameters (unknown to controller)
    theta_true = np.array([[2.0, 0.5, 3.0]]).T  # (3,1)

    # Initial estimates
    theta_hat = np.array([[0.5, 0.1, 0.5]]).T

    # State: q, qd
    q = 0.0
    qd = 0.0

    # Desired trajectory: constant 0.5 rad
    def q_des(t):
        return 0.5, 0.0, 0.0  # q_d, qd_d, qdd_d

    qs = []
    qds = []
    qds_d = []
    s_vals = []

    for k in range(n_steps):
        t = k * dt
        q_d, qd_d, qdd_d = q_des(t)

        tau, theta_hat, s = adaptive_ct_step(q, qd, theta_hat, q_d, qd_d, qdd_d, dt)

        # Plant dynamics: tau = theta_true^T * [qdd; qd; cos(q)]
        a, b, c = float(theta_true[0]), float(theta_true[1]), float(theta_true[2])
        # Solve for qdd
        qdd = (tau - b * qd - c * np.cos(q)) / a

        # Integrate with explicit Euler
        qd = qd + qdd * dt
        q = q + qd * dt

        qs.append(q)
        qds.append(qd)
        qds_d.append(q_d)
        s_vals.append(s)

    return np.array(qs), np.array(qds), np.array(qds_d), np.array(s_vals)

if __name__ == "__main__":
    qs, qds, qds_d, s_vals = simulate_adaptive_ct()
    # Plotting can be added with matplotlib if desired.
