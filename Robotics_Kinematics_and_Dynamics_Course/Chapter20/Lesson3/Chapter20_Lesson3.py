import numpy as np

# Regressor for 1-DOF rotary joint
def Y_pendulum(q, qd, qdd):
    return np.array([[qdd, np.sin(q)]])  # shape (1, 2)

# Parameter distribution (Gaussian)
theta_mean = np.array([1.5, 0.8])                # a, b
Sigma_theta = np.array([[0.04, 0.0],
                        [0.0,  0.01]])           # covariance

# Operating point
q   = 0.5       # rad
qd  = 0.0       # rad/s
qdd = 2.0       # rad/s^2

Y = Y_pendulum(q, qd, qdd)                       # (1, 2)

# Analytical mean and variance of tau
tau_mean = (Y @ theta_mean)[0]
tau_var  = (Y @ Sigma_theta @ Y.T)[0, 0]

print("Analytical E[tau] =", tau_mean)
print("Analytical Var[tau] =", tau_var)

# Monte Carlo check
rng = np.random.default_rng(0)
Nmc = 100000
theta_samples = rng.multivariate_normal(theta_mean, Sigma_theta, size=Nmc)
tau_samples = (Y @ theta_samples.T).ravel()

print("MC mean approx =", tau_samples.mean())
print("MC var approx  =", tau_samples.var())

# Example: include additive torque noise (independent Gaussian)
sigma_eps = 0.05
tau_noisy_samples = tau_samples + rng.normal(0.0, sigma_eps, size=Nmc)
tau_total_var = tau_var + sigma_eps**2
print("Total theoretical Var[tau+eps] =", tau_total_var)
      
