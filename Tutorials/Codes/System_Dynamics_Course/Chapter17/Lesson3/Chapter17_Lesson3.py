# Chapter17_Lesson3.py
# Response of Linear Systems to Random Inputs: Mean and Variance Propagation
# Python implementation: continuous-time covariance propagation + Monte Carlo verification

import numpy as np

np.random.seed(7)

# Mass-spring-damper (state-space)
# x1 = displacement, x2 = velocity
m = 1.0
c = 0.6
k = 4.0

A = np.array([[0.0, 1.0],
              [-k/m, -c/m]])
Gamma = np.array([[0.0],
                  [1.0/m]])   # white-noise force channel
C = np.array([[1.0, 0.0]])    # observe displacement

# Deterministic mean input enters through B*u_mean
B = np.array([[0.0],
              [1.0/m]])
u_mean = 1.0  # constant mean force

# White noise force spectral intensity (continuous-time)
q = 0.8
Q = np.array([[q]])

# Simulation horizon
T = 20.0
dt = 1e-3
N = int(T / dt)

# Mean and covariance propagation (Euler integration of moment ODEs)
mx = np.zeros((2, 1))
P = np.zeros((2, 2))

mean_hist = np.zeros((N, 2))
var_x1_hist = np.zeros(N)

for n in range(N):
    mdot = A @ mx + B * u_mean
    Pdot = A @ P + P @ A.T + Gamma @ Q @ Gamma.T
    mx = mx + dt * mdot
    P = P + dt * Pdot

    mean_hist[n, :] = mx[:, 0]
    var_x1_hist[n] = P[0, 0]

# Monte Carlo (Euler-Maruyama) to verify the covariance result
M = 4000  # trajectories
x = np.zeros((M, 2))
x1_mean_mc = np.zeros(N)
x1_var_mc = np.zeros(N)

sqrt_qdt = np.sqrt(q * dt)

for n in range(N):
    # white-noise increment: w(t) dt ~ sqrt(q dt) * N(0,1)
    noise = np.random.randn(M, 1)
    drift = (x @ A.T) + (u_mean * np.ones((M, 1))) @ B.T
    diffusion = noise @ (sqrt_qdt * Gamma.T)
    x = x + dt * drift + diffusion

    x1 = x[:, 0]
    x1_mean_mc[n] = np.mean(x1)
    x1_var_mc[n] = np.var(x1, ddof=1)

# Steady-state covariance by solving the continuous Lyapunov equation numerically
# A P + P A^T + Gamma Q Gamma^T = 0
# We vectorize for a 2x2 system: vec(AP + PA^T) = (I kron A + A kron I) vec(P)
I2 = np.eye(2)
L = np.kron(I2, A) + np.kron(A, I2)
rhs = -(Gamma @ Q @ Gamma.T).reshape(-1, 1)
vecP_ss = np.linalg.solve(L, rhs)
P_ss = vecP_ss.reshape(2, 2)

print("Final propagated mean (theory):", mean_hist[-1, :])
print("Final propagated displacement variance (theory):", var_x1_hist[-1])
print("Final displacement mean (Monte Carlo):", x1_mean_mc[-1])
print("Final displacement variance (Monte Carlo):", x1_var_mc[-1])
print("Steady-state covariance from Lyapunov solve:")
print(P_ss)

# A compact discrete-time example (scalar) for lecture continuity
# x[k+1] = a_d x[k] + b_d * eta[k], eta[k] ~ N(0, sigma_eta^2)
a_d = 0.92
b_d = 1.0
sigma_eta2 = 0.5
P_k = 0.0
for kstep in range(50):
    P_k = a_d**2 * P_k + b_d**2 * sigma_eta2
print("Scalar discrete-time variance after 50 steps:", P_k)
print("Scalar steady-state variance:", (b_d**2 * sigma_eta2) / (1 - a_d**2))
