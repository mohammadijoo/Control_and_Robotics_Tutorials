
import numpy as np

# 1-DOF joint: state x = [q, qdot]^T, input u = torque
Ts = 0.02  # 20 ms sampling
I = 1.0    # inertia

A = np.array([[1.0, Ts],
              [0.0, 1.0]])
B = np.array([[0.5 * Ts**2 / I],
              [Ts / I]])

Q = np.diag([10.0, 1.0])
R = np.array([[0.1]])
P = Q.copy()   # simple choice; could use DARE solution for P

N = 20  # prediction horizon

n = A.shape[0]
m = B.shape[1]

# Build prediction matrices A_bar, B_bar
A_bar = np.zeros((N * n, n))
B_bar = np.zeros((N * n, N * m))

for i in range(N):
    A_power = np.linalg.matrix_power(A, i + 1)
    A_bar[i*n:(i+1)*n, :] = A_power
    for j in range(i + 1):
        A_ij = np.linalg.matrix_power(A, i - j)
        B_bar[i*n:(i+1)*n, j*m:(j+1)*m] = A_ij @ B

# Block-diagonal weights (no explicit reference tracking here, r_k = 0)
Q_bar = np.kron(np.eye(N - 1), Q)
Q_bar = np.block([
    [Q_bar, np.zeros(((N - 1) * n, n))],
    [np.zeros((n, (N - 1) * n)), P]
])
R_bar = np.kron(np.eye(N), R)

H = B_bar.T @ Q_bar @ B_bar + R_bar
F = B_bar.T @ Q_bar @ A_bar

# Pre-factorize H for speed (Cholesky or LDL can be used instead)
H_inv = np.linalg.inv(H)

def mpc_control(x):
    """
    One-step MPC: given current state x (2x1), compute u_k.
    Unconstrained linear-quadratic MPC; r_k = 0.
    """
    # Compute optimal sequence U_k^* = -H^{-1} F x_k
    U_star = - H_inv @ (F @ x)
    # Extract first control move
    u_k = U_star[0]
    return float(u_k)

# Simulate closed-loop response
T_final = 1.0
N_steps = int(T_final / Ts)

x = np.array([[0.5],   # initial position error (rad)
              [0.0]])  # initial velocity
trajectory = []
for k in range(N_steps):
    u = mpc_control(x)
    trajectory.append((k * Ts, float(x[0]), float(x[1]), u))
    # Plant update (discrete-time model)
    x = A @ x + B * u

# trajectory now contains (time, q, qdot, u) samples
