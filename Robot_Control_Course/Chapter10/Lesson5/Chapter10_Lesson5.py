
import numpy as np
from scipy import sparse
import osqp

# Joint-space double integrator model for n = 2
n = 2
Ts = 0.02

I = np.eye(n)
A = np.block([[I, Ts * I],
              [np.zeros((n, n)), I]])
B = np.block([[np.zeros((n, n))],
              [Ts * I]])

nx = A.shape[0]
nu = B.shape[1]

# MPC parameters
N = 20  # prediction horizon
Q = np.diag([100.0, 100.0, 10.0, 10.0])  # position, velocity weights
R = 0.1 * np.eye(n)
P = Q  # terminal weight (could be DARE solution)

# Build block-diagonal Qbar, Rbar
Q_blocks = [Q] * (N - 1) + [P]
Qbar = sparse.block_diag(Q_blocks, format="csc")
Rbar = sparse.block_diag([R] * N, format="csc")

# Build prediction matrices Phi, Gamma
def build_prediction_matrices(A, B, N):
    nx, nu = A.shape[0], B.shape[1]
    Phi = np.zeros((N * nx, nx))
    Gamma = np.zeros((N * nx, N * nu))
    A_power = np.eye(nx)
    for i in range(N):
        A_power = A_power @ A
        Phi[i * nx:(i + 1) * nx, :] = A_power
        for j in range(i + 1):
            A_power_j = np.linalg.matrix_power(A, i - j)
            Gamma[i * nx:(i + 1) * nx,
                  j * nu:(j + 1) * nu] = A_power_j @ B
    return sparse.csc_matrix(Phi), sparse.csc_matrix(Gamma)

Phi, Gamma = build_prediction_matrices(A, B, N)

# Cost matrices: H and parametric linear term f(x0)
H = 2.0 * (Gamma.T @ Qbar @ Gamma + Rbar)
H = (H + H.T) * 0.5  # make numerically symmetric
H = sparse.csc_matrix(H)

# Constraints on virtual accelerations u: u_min <= u_k <= u_max
u_min = -5.0 * np.ones(n)
u_max = 5.0 * np.ones(n)
u_min_vec = np.tile(u_min, N)
u_max_vec = np.tile(u_max, N)

G_u = sparse.vstack([
    sparse.eye(N * nu),
    -sparse.eye(N * nu)
])
h_u = np.hstack([u_max_vec, -u_min_vec])

# No explicit state constraints in this first implementation
G = G_u
h = h_u

# OSQP problem prepared once; only q vector changes with x0
prob = osqp.OSQP()
# Dummy initial q vector, updated later
q_init = np.zeros(N * nu)
prob.setup(P=H, q=q_init, A=G, l=-np.inf * np.ones(G.shape[0]),
           u=h, verbose=False, warm_start=True)

def compute_q(x0, xref_seq=None):
    """
    Compute linear term q for OSQP given current state x0 and reference sequence.
    For simplicity, we consider state reference x_ref = 0 (pure regulation).
    """
    if xref_seq is None:
        Xref = np.zeros(Phi.shape[0])
    else:
        Xref = xref_seq.reshape(-1)
    # f(x0) = 2 * Gamma^T Qbar (Phi x0 - Xref)
    fx = 2.0 * Gamma.T @ Qbar @ (Phi @ x0 - Xref)
    return np.array(fx).reshape(-1)

# Simulation
T_sim = 4.0  # seconds
steps = int(T_sim / Ts)

# Initial error: joints start at zero, want to reach some target (q_ref)
q_ref_final = np.array([0.5, -0.3])
x = np.zeros(nx)

# Reference is ramped for the first second, then constant
def joint_reference(k):
    t = k * Ts
    if t < 1.0:
        alpha = t / 1.0
    else:
        alpha = 1.0
    return alpha * q_ref_final

traj_q = []
traj_u0 = []

for k in range(steps):
    # Build reference sequence in state space (here: e_q = q - q_ref)
    xref_seq = []
    q_ref_k = joint_reference(k)
    dq_ref_k = np.zeros(n)
    for i in range(1, N + 1):
        q_ref = q_ref_k
        dq_ref = dq_ref_k
        e_q_ref = np.zeros_like(q_ref)
        de_q_ref = np.zeros_like(dq_ref)
        xref_seq.append(np.hstack([e_q_ref, de_q_ref]))
    xref_seq = np.array(xref_seq)

    # Current error state x = [e_q; e_qdot]
    # (Here we maintain x directly; in a real system we would measure q, dq.)

    q_vec = compute_q(x, xref_seq)
    prob.update(q=q_vec)

    res = prob.solve()
    if res.info.status_val not in (1, 2):
        raise RuntimeError("OSQP did not converge")

    U_opt = res.x
    u0 = U_opt[:nu]

    # Apply u0 to the double-integrator model
    x = A @ x + B @ u0

    traj_q.append(x[:n].copy())
    traj_u0.append(u0.copy())

traj_q = np.array(traj_q)
traj_u0 = np.array(traj_u0)

print("Final joint error:", traj_q[-1])
