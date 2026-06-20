import numpy as np
from scipy.linalg import solve_continuous_are, expm
from scipy.signal import cont2discrete

# Physical parameters
m = 1.0
b = 0.5
k = 2.0

A = np.array([[0.0,       1.0],
              [-k/m,   -b/m]])
B = np.array([[0.0],
              [1.0/m]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])

# LQR weights
Q = np.diag([10.0, 1.0])   # penalize position and velocity
R = np.array([[1.0]])      # penalize control effort

# Solve the continuous-time ARE
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

print("LQR gain K:", K)

# Discretize with sampling period Ts
Ts = 0.01  # 10 ms sampling
Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), Ts, method="zoh")

print("Ad:\n", Ad)
print("Bd:\n", Bd)

# Simulate one step of discrete-time closed-loop dynamics
x = np.array([[0.1],
              [0.0]])  # initial state
u = -K @ x            # state feedback
x_next = Ad @ x + Bd @ u
print("Next state x_next:", x_next)
