
import numpy as np

# Single-joint parameters
I = 0.5     # inertia
b = 0.1     # viscous damping

# Continuous-time A, B matrices
A = np.array([[0.0,      1.0],
              [0.0, -b / I]])
B = np.array([[0.0],
              [1.0 / I]])

# LQR weights
q_w = 100.0
qd_w = 10.0
r_w = 1.0
Q = np.diag([q_w, qd_w])
R = np.array([[r_w]])

# Horizon and discretization
T = 2.0           # total time [s]
dt = 0.002        # sampling period [s]
N = int(T / dt)   # number of steps

# Simple forward-Euler discretization (for illustration)
n = A.shape[0]
Ad = np.eye(n) + A * dt
Bd = B * dt

# Backward Riccati recursion
P = np.zeros((N + 1, n, n))
K = np.zeros((N, B.shape[1], n))

# Terminal cost
P[N] = Q.copy()

for k in range(N - 1, -1, -1):
    P_next = P[k + 1]
    # Compute gain K_k
    S = R + Bd.T @ P_next @ Bd
    K[k] = np.linalg.solve(S, Bd.T @ P_next @ Ad)
    # Update P_k
    P[k] = Q + Ad.T @ P_next @ Ad - Ad.T @ P_next @ Bd @ K[k]

# Simulation of closed-loop system
x = np.array([[0.3],   # initial position error [rad]
              [0.0]])  # initial velocity error [rad/s]

traj = []
for k in range(N):
    u = -K[k] @ x
    dx = A @ x + B @ u
    x = x + dt * dx
    traj.append((k * dt, float(x[0, 0]), float(x[1, 0]), float(u[0, 0])))

# 'traj' holds time, position, velocity, torque; can be plotted or exported
