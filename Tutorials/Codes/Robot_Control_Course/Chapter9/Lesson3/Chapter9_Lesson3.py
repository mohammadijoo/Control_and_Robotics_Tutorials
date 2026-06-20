
import numpy as np

# Pendulum parameters
m = 1.0
l = 1.0
g = 9.81
dt = 0.01

def dynamics(x, u):
    """Discrete-time pendulum dynamics x_{k+1} = f(x_k, u_k)."""
    theta, dtheta = x
    # continuous-time acceleration
    ddtheta = (-g / l) * np.sin(theta) + u / (m * l**2)
    # Euler integration
    theta_next = theta + dt * dtheta
    dtheta_next = dtheta + dt * ddtheta
    return np.array([theta_next, dtheta_next])

def linearize_dynamics(x, u):
    """Return f_x (2x2) and f_u (2x1) at (x, u)."""
    theta, dtheta = x
    # partial derivatives
    ddtheta_dtheta = (-g / l) * np.cos(theta)
    ddtheta_du = 1.0 / (m * l**2)

    fx = np.eye(2)
    fx[0, 1] = dt  # d theta_next / d dtheta
    fx[1, 0] = dt * ddtheta_dtheta
    fx[1, 1] = 1.0  # d dtheta_next / d dtheta

    fu = np.zeros((2, 1))
    fu[1, 0] = dt * ddtheta_du
    return fx, fu

# Cost matrices
Q = np.diag([10.0, 1.0])
R = np.array([[0.1]])
P_N = np.diag([50.0, 5.0])

def stage_cost(x, u, x_ref):
    dx = x - x_ref
    return 0.5 * dx.T @ Q @ dx + 0.5 * u.T @ R @ u

def terminal_cost(x, x_ref):
    dx = x - x_ref
    return 0.5 * dx.T @ P_N @ dx

def ilqr(x0, x_ref_traj, N, max_iter=50):
    # Initial (zero) control sequence
    n = x0.size
    m = 1
    U = np.zeros((N, m))
    X = np.zeros((N + 1, n))
    X[0] = x0

    # Rollout to initialize X
    for k in range(N):
        X[k + 1] = dynamics(X[k], U[k])

    for it in range(max_iter):
        # Backward pass
        Vx = P_N @ (X[N] - x_ref_traj[N])
        Vxx = P_N.copy()

        K = np.zeros((N, m, n))
        k_feedforward = np.zeros((N, m))

        regularization = 1e-6

        for k in range(N - 1, -1, -1):
            xk = X[k]
            uk = U[k]
            x_ref = x_ref_traj[k]

            fx, fu = linearize_dynamics(xk, uk)

            # Cost derivatives (quadratic tracking cost)
            dx = xk - x_ref
            lx = Q @ dx
            lu = R @ uk
            lxx = Q
            luu = R
            lux = np.zeros((m, n))

            # Q derivatives for iLQR
            Qx = lx + fx.T @ Vx
            Qu = lu + fu.T @ Vx
            Qxx = lxx + fx.T @ Vxx @ fx
            Quu = luu + fu.T @ Vxx @ fu + regularization * np.eye(m)
            Qux = lux + fu.T @ Vxx @ fx

            # Compute gains
            Quu_inv = np.linalg.inv(Quu)
            k_ff = -Quu_inv @ Qu
            K_fb = -Quu_inv @ Qux

            k_feedforward[k] = k_ff.ravel()
            K[k] = K_fb

            # Update value function
            Vx = Qx + K_fb.T @ Quu @ k_ff + K_fb.T @ Qu + Qux.T @ k_ff
            Vxx = Qxx + K_fb.T @ Quu @ K_fb + K_fb.T @ Qux + Qux.T @ K_fb
            # Symmetrize for numerical stability
            Vxx = 0.5 * (Vxx + Vxx.T)

        # Forward pass with line search
        alpha_list = [1.0, 0.5, 0.25, 0.1]
        cost_old = (
            sum(stage_cost(X[k], U[k], x_ref_traj[k]) for k in range(N))
            + terminal_cost(X[N], x_ref_traj[N])
        )
        improved = False
        for alpha in alpha_list:
            X_new = np.zeros_like(X)
            U_new = np.zeros_like(U)
            X_new[0] = x0
            for k in range(N):
                du = alpha * k_feedforward[k] + K[k] @ (X_new[k] - X[k])
                U_new[k] = U[k] + du
                X_new[k + 1] = dynamics(X_new[k], U_new[k])
            cost_new = (
                sum(stage_cost(X_new[k], U_new[k], x_ref_traj[k]) for k in range(N))
                + terminal_cost(X_new[N], x_ref_traj[N])
            )
            if cost_new < cost_old:
                X, U = X_new, U_new
                improved = True
                break

        if not improved:
            # Could increase regularization here; for simplicity we just stop
            break

    return X, U

# Example usage: stabilize around upright theta = 0
N = 300
x0 = np.array([0.5, 0.0])  # initial angle 0.5 rad
x_ref_traj = np.zeros((N + 1, 2))
X_opt, U_opt = ilqr(x0, x_ref_traj, N)
