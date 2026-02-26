import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Problem setup
nq = 2
nv = 2
nx = nq + nv
nu = 2

T = 2.0           # horizon [s]
N = 40            # number of segments
h = T / N

# Simple planar 2-link arm parameters (one possible parameterization)
m1, m2 = 1.0, 1.0
l1, l2 = 1.0, 1.0
lc1, lc2 = 0.5, 0.5
I1, I2 = 0.12, 0.12
g = 9.81

def two_link_dynamics(x, u):
    """
    x = [q1, q2, v1, v2]
    u = [u1, u2]
    returns xdot = [v1, v2, a1, a2]
    """
    q1, q2, v1, v2 = x[0], x[1], x[2], x[3]
    u1, u2 = u[0], u[1]

    s2 = ca.sin(q2)
    c2 = ca.cos(q2)

    # Inertia matrix M(q)
    M11 = I1 + I2 + m2 * l1**2 + 2 * m2 * l1 * lc2 * c2
    M12 = I2 + m2 * l1 * lc2 * c2
    M21 = M12
    M22 = I2
    M = ca.vertcat(
        ca.hcat([M11, M12]),
        ca.hcat([M21, M22])
    )

    # Coriolis/centrifugal matrix C(q, v)
    h_c = m2 * l1 * lc2 * s2
    C11 = 0.0
    C12 = -2.0 * h_c * v2
    C21 = h_c * v1
    C22 = 0.0
    C = ca.vertcat(
        ca.hcat([C11, C12]),
        ca.hcat([C21, C22])
    )

    # Gravity vector g(q)
    g1 = (m1 * lc1 + m2 * l1) * g * ca.cos(q1) + m2 * lc2 * g * ca.cos(q1 + q2)
    g2 = m2 * lc2 * g * ca.cos(q1 + q2)
    G = ca.vertcat(g1, g2)

    v = ca.vertcat(v1, v2)
    tau = ca.vertcat(u1, u2)

    # M(q) * a + C(q, v) * v + g(q) = tau
    a = ca.solve(M, tau - C @ v - G)

    return ca.vertcat(v, a)

# Collocation optimization with CasADi Opti
opti = ca.Opti()

X = opti.variable(nx, N + 1)  # states at nodes
U = opti.variable(nu, N + 1)  # controls at nodes

# Boundary conditions
q_init = np.array([0.0, 0.0])
v_init = np.array([0.0, 0.0])
q_goal = np.array([np.pi / 2.0, 0.0])
v_goal = np.array([0.0, 0.0])

x_init = np.concatenate([q_init, v_init])
x_goal = np.concatenate([q_goal, v_goal])

opti.subject_to(X[:, 0] == x_init)
opti.subject_to(X[:, N] == x_goal)

# Cost (control effort)
R = np.eye(nu)
J = 0
for k in range(N):
    uk = U[:, k]
    uk1 = U[:, k + 1]
    J += 0.5 * h * (ca.mtimes([uk.T, R, uk]) + ca.mtimes([uk1.T, R, uk1]))
opti.minimize(J)

# Collocation constraints (trapezoidal)
for k in range(N):
    xk = X[:, k]
    xk1 = X[:, k + 1]
    uk = U[:, k]
    uk1 = U[:, k + 1]

    fk = two_link_dynamics(xk, uk)
    fk1 = two_link_dynamics(xk1, uk1)

    opti.subject_to(xk1 == xk + 0.5 * h * (fk + fk1))

# Optional simple box constraints on states and controls
q_min = -np.pi
q_max = np.pi
v_max = 4.0
u_max = 10.0

q = X[0:2, :]
v = X[2:4, :]

opti.subject_to(q >= q_min)
opti.subject_to(q <= q_max)
opti.subject_to(v >= -v_max)
opti.subject_to(v <= v_max)
opti.subject_to(U >= -u_max)
opti.subject_to(U <= u_max)

# Initial guess
opti.set_initial(X, np.tile(x_init.reshape((-1, 1)), (1, N + 1)))
opti.set_initial(U, 0.0)

# Solver
opti.solver("ipopt", {"print_time": False}, {"max_iter": 200})

sol = opti.solve()
X_opt = sol.value(X)
U_opt = sol.value(U)

# Plot joint trajectories
t_grid = np.linspace(0.0, T, N + 1)
q1_opt = X_opt[0, :]
q2_opt = X_opt[1, :]

plt.figure()
plt.plot(t_grid, q1_opt, label="q1")
plt.plot(t_grid, q2_opt, label="q2")
plt.xlabel("time [s]")
plt.ylabel("joint position [rad]")
plt.legend()
plt.grid(True)
plt.show()
      
