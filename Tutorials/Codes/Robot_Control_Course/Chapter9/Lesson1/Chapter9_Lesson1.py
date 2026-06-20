
import numpy as np

def quadratic_tracking_cost(q, q_d, Q, dt):
    """
    Joint-space tracking cost:
    J_track = 0.5 * sum_k (q_k - q_d_k)^T Q (q_k - q_d_k) * dt
    q, q_d: arrays of shape (N+1, n)
    Q: (n, n) symmetric positive semidefinite
    """
    e = q - q_d                      # shape (N+1, n)
    # compute v_k = e_k^T Q e_k for all k
    v = np.einsum("ki,ij,kj->k", e, Q, e)
    return 0.5 * dt * np.sum(v)

def quadratic_effort_cost(tau, R, dt):
    """
    Effort cost:
    J_effort = 0.5 * sum_k tau_k^T R tau_k * dt
    tau: shape (N, n)
    R: (n, n) symmetric positive semidefinite
    """
    v = np.einsum("ki,ij,kj->k", tau, R, tau)
    return 0.5 * dt * np.sum(v)

def control_smoothness_cost(tau, S_u):
    """
    Smoothness cost on torque increments:
    J_du = 0.5 * sum_k (tau_k - tau_{k-1})^T S_u (tau_k - tau_{k-1})
    tau: shape (N, n)
    """
    dtau = tau[1:] - tau[:-1]        # shape (N-1, n)
    v = np.einsum("ki,ij,kj->k", dtau, S_u, dtau)
    return 0.5 * np.sum(v)

def total_cost(q, q_d, tau, Q, R, S_u, dt, lambdas):
    """
    Combine all terms with scalar weights:
      J = lambda_track * J_track + lambda_effort * J_effort
          + lambda_smooth * J_du
    lambdas: dict with keys "track", "effort", "smooth"
    """
    J_tr = quadratic_tracking_cost(q, q_d, Q, dt)
    J_eff = quadratic_effort_cost(tau, R, dt)
    J_du = control_smoothness_cost(tau, S_u)

    return (lambdas["track"] * J_tr
            + lambdas["effort"] * J_eff
            + lambdas["smooth"] * J_du)

# Example usage:
N = 100
n = 6
dt = 0.01
q = np.zeros((N+1, n))
q_d = np.zeros_like(q)
tau = np.zeros((N, n))
Q = np.eye(n)
R = 0.01 * np.eye(n)
S_u = 0.1 * np.eye(n)
lambdas = {"track": 1.0, "effort": 1.0, "smooth": 0.5}

J = total_cost(q, q_d, tau, Q, R, S_u, dt, lambdas)
print("Total cost:", J)
