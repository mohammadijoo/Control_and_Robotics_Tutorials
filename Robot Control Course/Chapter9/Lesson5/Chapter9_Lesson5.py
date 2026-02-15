
import numpy as np

# Physical parameters
m = 1.0
ell = 1.0
I = m * ell**2
b = 0.1
g = 9.81
dt = 0.01

def dynamics(x, u):
    """Continuous-time dynamics xdot = f(x, u), x = [q, dq]."""
    q, dq = x
    ddq = (u - b * dq - m * g * ell * np.sin(q)) / I
    return np.array([dq, ddq])

def step(x, u):
    """Discrete-time step with forward Euler."""
    return x + dt * dynamics(x, u)

# Linearization around equilibrium (q*, dq* = 0, u* = m g ell sin(q*))
q_star = 0.0  # downward equilibrium
dq_star = 0.0
u_star = m * g * ell * np.sin(q_star)
x_star = np.array([q_star, dq_star])

A_c = np.array([[0.0, 1.0],
                [-(m * g * ell / I) * np.cos(q_star), -b / I]])
B_c = np.array([[0.0],
                [1.0 / I]])

A_d = np.eye(2) + dt * A_c
B_d = dt * B_c

# LQR gains via finite-horizon Riccati recursion
N = 500  # horizon
Q = np.diag([10.0, 1.0])
R = np.array([[0.1]])
Qf = np.diag([50.0, 5.0])

P = [None] * (N + 1)
K = [None] * N
P[N] = Qf

for k in range(N - 1, -1, -1):
    S = R + B_d.T @ P[k + 1] @ B_d
    K[k] = -np.linalg.solve(S, B_d.T @ P[k + 1] @ A_d)
    Acl = A_d + B_d @ K[k]
    P[k] = Q + Acl.T @ P[k + 1] @ Acl

def simulate_lqr(x0, steps=500):
    xs = np.zeros((steps + 1, 2))
    us = np.zeros(steps)
    xs[0] = x0
    for k in range(steps):
        dx = xs[k] - x_star
        u = float(u_star + K[min(k, N - 1)] @ dx)
        xs[k + 1] = step(xs[k], u)
        us[k] = u
    return xs, us

# Minimal iLQR implementation for tracking a reference
def rollout(x0, us):
    xs = [x0]
    for u in us:
        xs.append(step(xs[-1], u))
    return np.array(xs)

def ilqr(x0, x_ref, u_init, Q, R, Qf, max_iter=50, alpha_list=None):
    if alpha_list is None:
        alpha_list = [1.0, 0.5, 0.25, 0.1]
    N = len(u_init)
    n = x0.shape[0]
    us = u_init.copy()
    xs = rollout(x0, us)
    for it in range(max_iter):
        Vx = 2.0 * Qf @ (xs[-1] - x_ref[-1])
        Vxx = 2.0 * Qf.copy()
        ks = [None] * N
        Ks = [None] * N

        # Backward pass (finite-difference Jacobians for simplicity)
        for k in range(N - 1, -1, -1):
            xk = xs[k]
            uk = us[k]
            xr = x_ref[k]
            ur = 0.0  # here reference torque is zero

            # Cost derivatives (quadratic tracking)
            lx = 2.0 * Q @ (xk - xr)
            lu = 2.0 * R @ np.array([[uk - ur]])
            lxx = 2.0 * Q
            luu = 2.0 * R.copy()
            lux = np.zeros((1, n))

            # Numerical linearization of dynamics
            eps = 1e-5
            A = np.zeros((n, n))
            B = np.zeros((n, 1))
            fx = step(xk, uk)
            for i in range(n):
                dx = np.zeros(n)
                dx[i] = eps
                A[:, i] = (step(xk + dx, uk) - fx) / eps
            du = 1e-5
            B[:, 0] = (step(xk, uk + du) - fx) / du

            Qx = lx + A.T @ Vx
            Qu = lu + B.T @ Vx
            Qxx = lxx + A.T @ Vxx @ A
            Quu = luu + B.T @ Vxx @ B
            Qux = lux + B.T @ Vxx @ A

            # Regularization for numerical stability
            Quu_reg = Quu + 1e-6 * np.eye(Quu.shape[0])
            k_ff = -np.linalg.solve(Quu_reg, Qu)
            K_fb = -np.linalg.solve(Quu_reg, Qux)

            ks[k] = k_ff
            Ks[k] = K_fb

            Vx = Qx + K_fb.T @ Quu @ k_ff + K_fb.T @ Qu + Qux.T @ k_ff
            Vxx = Qxx + K_fb.T @ Quu @ K_fb + K_fb.T @ Qux + Qux.T @ K_fb
            Vxx = 0.5 * (Vxx + Vxx.T)  # symmetrize

        # Forward pass with line search
        cost_old = compute_cost(xs, us, x_ref, Q, R, Qf)
        improved = False
        for alpha in alpha_list:
            xs_new = [x0]
            us_new = []
            for k in range(N):
                dx = xs_new[-1] - xs[k]
                du = alpha * ks[k] + Ks[k] @ dx
                u_new = us[k] + float(du)
                us_new.append(u_new)
                xs_new.append(step(xs_new[-1], u_new))
            xs_new = np.array(xs_new)
            us_new = np.array(us_new)
            cost_new = compute_cost(xs_new, us_new, x_ref, Q, R, Qf)
            if cost_new < cost_old:
                xs, us = xs_new, us_new
                improved = True
                break
        if not improved:
            break
    return xs, us

def compute_cost(xs, us, x_ref, Q, R, Qf):
    J = 0.0
    for k in range(len(us)):
        dx = xs[k] - x_ref[k]
        du = us[k]
        J += dx.T @ Q @ dx + R * du * du
    dxN = xs[-1] - x_ref[-1]
    J += dxN.T @ Qf @ dxN
    return float(J)
