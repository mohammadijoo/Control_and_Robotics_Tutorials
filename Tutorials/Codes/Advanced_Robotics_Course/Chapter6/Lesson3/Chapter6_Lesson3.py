import numpy as np
import cvxpy as cp
from scipy.stats import norm

# Discrete-time double integrator parameters
dt = 0.1
A = np.array([[1.0, dt],
              [0.0, 1.0]])
B = np.array([[0.5 * dt**2],
              [dt]])

n = 2  # state dimension
m = 1  # control dimension
N = 30  # horizon length

# Process noise covariance and initial belief
W = np.diag([1e-4, 1e-4])
mu0 = np.array([0.0, 0.0])      # initial mean [p0, v0]
Sigma0 = np.diag([0.05**2, 0.02**2])

# Corridor upper bound and risk parameters
p_max = 2.0
alpha_total = 0.05      # total allowed path-wise risk
epsilon_k = alpha_total / N  # uniform per-step risk allocation
beta_eps = norm.ppf(1.0 - epsilon_k)

# Pre-compute covariance sequence (independent of controls)
Sigma_list = [Sigma0]
for k in range(N):
    Sigma_next = A @ Sigma_list[-1] @ A.T + W
    Sigma_list.append(Sigma_next)

sigma_p = [np.sqrt(Sigma_list[k][0, 0]) for k in range(N + 1)]

# Optimization variables
X = cp.Variable((n, N + 1))
U = cp.Variable((m, N))

# Objective: quadratic control effort + terminal position error
p_target = 1.5
cost = 0.0
constraints = []

# Initial condition on mean state
constraints.append(X[:, 0] == mu0)

for k in range(N):
    # Linear dynamics on mean
    constraints.append(X[:, k+1] == A @ X[:, k] + B @ U[:, k])

    # Chance constraint for position: P(p_k+1 <= p_max) >= 1 - epsilon_k
    # Using: mu_p + beta * sigma_p <= p_max  (sigma_p is precomputed constant)
    mu_p_k1 = X[0, k+1]
    constraints.append(mu_p_k1 + beta_eps * sigma_p[k+1] <= p_max)

    # Optional input bounds
    constraints.append(cp.abs(U[:, k]) <= 2.0)

    # Stage cost: penalize control effort and deviation from target position
    cost += 0.1 * cp.sum_squares(U[:, k])

# Terminal cost: encourage p_N close to p_target
cost += 10.0 * cp.sum_squares(X[0, N] - p_target)

prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP)

print("Optimal cost:", prob.value)
print("Planned positions:", X.value[0, :])
print("Planned controls:", U.value[0, :])
      
