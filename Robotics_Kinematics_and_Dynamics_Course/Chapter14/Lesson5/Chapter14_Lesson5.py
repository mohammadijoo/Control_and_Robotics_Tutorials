import numpy as np

def pendulum_regressor(q, dq, ddq):
    """
    Regressor for I*ddq + b*dq + k*sin(q) = tau.
    Returns Y (1x3) so that tau = Y @ theta, theta = [I, b, k].
    """
    return np.array([[ddq, dq, np.sin(q)]], dtype=float)

# Nominal parameters
theta_hat = np.array([0.05, 0.01, 0.5])  # [I, b, k]

# Interval radii (absolute)
delta = np.array([0.005, 0.002, 0.05])   # +-10% for I, 20% for b, 10% for k

# Example trajectory point
q = np.deg2rad(30.0)
dq = 0.5
ddq = 1.0
Y = pendulum_regressor(q, dq, ddq)

# Nominal torque
tau_hat = float(Y @ theta_hat)

# Worst-case interval bound (element-wise)
tau_interval_radius = float(np.sum(np.abs(Y) * delta))
print("Nominal tau:", tau_hat)
print("Interval bound on |tau - tau_hat|:", tau_interval_radius)

# Norm-bounded uncertainty: treat theta error vector with 2-norm bound
theta_bar = np.linalg.norm(delta, ord=2)  # crude upper bound
Y_norm = np.linalg.norm(Y, ord=2)
tau_norm_bound = Y_norm * theta_bar
print("2-norm based bound on ||tau - tau_hat||:", tau_norm_bound)

# Monte Carlo sampling inside the parameter box to empirically check bounds
rng = np.random.default_rng(0)
N = 10000
errors = []
for _ in range(N):
    # Sample each parameter uniformly inside [theta_hat - delta, theta_hat + delta]
    theta_sample = theta_hat + rng.uniform(low=-delta, high=delta)
    tau_sample = float(Y @ theta_sample)
    errors.append(abs(tau_sample - tau_hat))

errors = np.array(errors)
print("Max error over samples:", errors.max())
print("Interval bound is valid?:", errors.max() <= tau_interval_radius + 1e-9)
      
