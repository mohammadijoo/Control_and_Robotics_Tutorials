
import numpy as np
import scipy.sparse as sp
import osqp

# Discrete-time linear model x_{k+1} = A x_k + B u_k
A = np.array([[1.0, 0.01],
              [0.0, 1.0]])      # simple joint position-velocity model
B = np.array([[0.0],
              [0.01]])          # joint torque or velocity input
nx, nu = A.shape[0], B.shape[1]

N = 20                         # prediction horizon
Q = np.diag([100.0, 1.0])
R = np.diag([0.1])
P = Q.copy()

# Build condensed matrices for a regulation problem x_ref = 0, u_ref = 0
# X = S x0 + T U
S = np.zeros(( (N+1)*nx, nx ))
T = np.zeros(( (N+1)*nx, N*nu ))

Ak = np.eye(nx)
for i in range(N+1):
    S[i*nx:(i+1)*nx, :] = Ak
    if i > 0:
        # contribution of previous controls
        Aj = np.eye(nx)
        for j in range(i):
            row = i*nx
            col = j*nu
            T[row:row+nx, col:col+nu] = Aj @ B
            Aj = Aj @ A
    Ak = Ak @ A

# Cost: 0.5 * U.T H U + x0.T F.T U + const
Qbar = sp.block_diag([sp.kron(sp.eye(N), Q), P]).toarray()
TQ = T.T @ Qbar
H = TQ @ T + sp.kron(sp.eye(N), R).toarray()
F = TQ @ S

# Simple box constraints on u: u_min <= u_k <= u_max
u_min = -0.5
u_max = 0.5
G_u = np.vstack([np.eye(N*nu), -np.eye(N*nu)])
g_u = np.hstack([u_max * np.ones(N*nu), -u_min * np.ones(N*nu)])

# OSQP setup (P must be sparse and symmetric)
P_qp = sp.csc_matrix((H + H.T) * 0.5)   # numerical symmetrization
A_qp = sp.csc_matrix(G_u)

# Initial state
x = np.array([0.5, 0.0])   # initial joint position error and velocity

# Warm-start buffers
U_prev = np.zeros(N*nu)
lam_prev = np.zeros(G_u.shape[0])

prob = osqp.OSQP()
prob.setup(P=P_qp,
           q=F.T @ x,
           A=A_qp,
           l=-np.inf * np.ones(G_u.shape[0]),
           u=g_u,
           warm_start=True,
           verbose=False)

for k in range(100):
    # Update linear term with new state
    q = F.T @ x
    prob.update(q=q)

    # Warm-start with previous primal and dual solutions
    prob.warm_start(x=U_prev, y=lam_prev)

    # Solve QP
    res = prob.solve()
    U_opt = res.x
    lam_opt = res.y

    # Extract first control and update state
    u = U_opt[0:nu]
    x = A @ x + B @ u

    # Shift warm-start for next step
    U_prev = np.roll(U_opt, -nu)
    U_prev[-nu:] = U_prev[-2*nu:-nu]    # repeat last element
    lam_prev = lam_opt.copy()

    # Here you would send u to the robot's actuator interface
