
import numpy as np

# Dimensions
nq = 2
nx = 2 * nq
nu = nq

dt = 0.005  # sampling time

def manipulator_dynamics(x, u):
    """
    Simple control-oriented dynamics for a 2-DOF manipulator:
    x = [q1, q2, dq1, dq2]
    M(q) ddq + C(q,dq) dq + g(q) = u
    Here we use a toy model with diagonal M and simple damping.
    In practice, call a robotics library (e.g., Pinocchio) here.
    """
    q = x[:nq]
    dq = x[nq:]
    # Inertia matrix (approximate)
    M = np.diag(1.0 + 0.3 * np.cos(q))
    # Damping
    D = 0.05 * np.eye(nq)
    # Gravity
    g = np.array([9.81 * np.cos(q[0]),
                  9.81 * np.cos(q[1])])
    # Solve for ddq: M ddq = u - D dq - g
    ddq = np.linalg.solve(M, u - D @ dq - g)
    xdot = np.concatenate([dq, ddq])
    return xdot

def f_discrete(x, u):
    # Forward Euler discretization
    return x + dt * manipulator_dynamics(x, u)

# Quadratic stage cost
Q = np.diag([100.0, 100.0, 1.0, 1.0])
R = 0.01 * np.eye(nu)

def stage_cost(x, u, x_ref, u_ref):
    dx = x - x_ref
    du = u - u_ref
    return dx.T @ Q @ dx + du.T @ R @ du

def terminal_cost(x, x_ref):
    P_N = 10.0 * Q  # crude approximation of P_infinity
    dx = x - x_ref
    return dx.T @ P_N @ dx

def ilqr_step(x0, x_ref_traj, u_init, n_iter_rt=3):
    """
    Run a small, fixed number of iLQR iterations.
    x_ref_traj: array of shape (N+1, nx)
    u_init: initial guess, shape (N, nu)
    n_iter_rt: maximum iterations (real-time budget)
    """
    N = u_init.shape[0]
    u = u_init.copy()
    alpha = 0.5  # line-search parameter (fixed here)

    for it in range(n_iter_rt):
        # Forward rollout
        x_traj = np.zeros((N + 1, nx))
        x_traj[0] = x0
        cost = 0.0
        for k in range(N):
            x_traj[k + 1] = f_discrete(x_traj[k], u[k])
            cost += stage_cost(x_traj[k], u[k], x_ref_traj[k], np.zeros(nu))
        cost += terminal_cost(x_traj[-1], x_ref_traj[-1])

        # Backward pass (very simplified, no second-order terms)
        K = np.zeros((N, nu, nx))
        du_ff = np.zeros((N, nu))
        P = 10.0 * Q  # terminal cost matrix (same as in terminal_cost)

        for k in reversed(range(N)):
            # Linearize dynamics numerically around (x_k, u_k)
            xk = x_traj[k]
            uk = u[k]
            Ak = np.zeros((nx, nx))
            Bk = np.zeros((nx, nu))
            eps = 1e-5
            for i in range(nx):
                dx = np.zeros(nx); dx[i] = eps
                Ak[:, i] = (f_discrete(xk + dx, uk) - f_discrete(xk - dx, uk)) / (2.0 * eps)
            for j in range(nu):
                du_j = np.zeros(nu); du_j[j] = eps
                Bk[:, j] = (f_discrete(xk, uk + du_j) - f_discrete(xk, uk - du_j)) / (2.0 * eps)

            # Quadratic approximation of cost
            Qx = 2.0 * Q @ (xk - x_ref_traj[k])
            Qu = 2.0 * R @ (uk - np.zeros(nu))
            Qxx = 2.0 * Q
            Quu = 2.0 * R
            Qux = np.zeros((nu, nx))

            # Compute Q-function derivatives
            Qx = Qx + Ak.T @ P @ (f_discrete(xk, uk) - x_traj[k + 1])
            Qu = Qu + Bk.T @ P @ (f_discrete(xk, uk) - x_traj[k + 1])
            Qxx = Qxx + Ak.T @ P @ Ak
            Quu = Quu + Bk.T @ P @ Bk
            Qux = Qux + Bk.T @ P @ Ak

            # Feedback and feedforward gains
            Quu_reg = Quu + 1e-4 * np.eye(nu)
            K[k] = -np.linalg.solve(Quu_reg, Qux)
            du_ff[k] = -np.linalg.solve(Quu_reg, Qu)

            # Update P
            P = Qxx + K[k].T @ Quu @ K[k] + \
                K[k].T @ Qux + Qux.T @ K[k]

        # Control update with simple line search
        u_new = u.copy()
        x_new = x0.copy()
        for k in range(N):
            du = du_ff[k] + K[k] @ (x_new - x_traj[k])
            u_new[k] = u[k] + alpha * du
            x_new = f_discrete(x_new, u_new[k])

        u = u_new

    return u
