# Chapter17_Lesson1.py
# Random Variables, Random Processes, and Stationarity Concepts
# Python implementation for System Dynamics (Chapter 17, Lesson 1)

import numpy as np
import matplotlib.pyplot as plt

np.random.seed(17)

# ------------------------------------------------------------
# Example 1: Random-phase harmonic process (wide-sense stationary)
# X(t) = A cos(omega t + Theta), Theta ~ Uniform(0, 2*pi)
# ------------------------------------------------------------
A = 2.0
omega = 3.0
M = 2000                  # number of realizations
t = np.linspace(0.0, 4.0, 401)

theta = np.random.uniform(0.0, 2.0 * np.pi, size=M)
X = np.array([A * np.cos(omega * t + th) for th in theta])   # shape: (M, Nt)

# Ensemble mean estimate (function of time)
mean_est = X.mean(axis=0)

# Ensemble variance estimate (function of time)
var_est = X.var(axis=0, ddof=0)

# Autocorrelation estimate at a few lags (ensemble-based)
def ensemble_autocorr(X_ensemble, lag_index):
    x1 = X_ensemble[:, :-lag_index] if lag_index > 0 else X_ensemble
    x2 = X_ensemble[:, lag_index:]  if lag_index > 0 else X_ensemble
    return np.mean(x1 * x2)

lags = [0, 10, 30, 60]
dt = t[1] - t[0]
print("Random-phase harmonic process estimates:")
print("Mean at selected times:", mean_est[::100])
print("Variance at selected times:", var_est[::100])
for L in lags:
    tau = L * dt
    R_hat = ensemble_autocorr(X, L)
    R_theory = 0.5 * A**2 * np.cos(omega * tau)
    print(f"lag={L:3d}, tau={tau:.3f}, R_hat={R_hat:.5f}, R_theory={R_theory:.5f}")

# ------------------------------------------------------------
# Example 2: Nonstationary process
# Y(t) = alpha * t + beta, alpha and beta random (constant per realization)
# mean and variance depend on t
# ------------------------------------------------------------
alpha = np.random.normal(loc=0.5, scale=0.2, size=M)
beta = np.random.normal(loc=0.0, scale=1.0, size=M)
Y = np.array([a * t + b for a, b in zip(alpha, beta)])

Y_mean = Y.mean(axis=0)
Y_var = Y.var(axis=0, ddof=0)

print("\nNonstationary affine-in-time process estimates:")
for idx in [0, 100, 200, 400]:
    print(f"t={t[idx]:.2f}, mean={Y_mean[idx]:.4f}, var={Y_var[idx]:.4f}")

# ------------------------------------------------------------
# Example 3: Discrete-time AR(1) process (stationary regime if |a| < 1)
# x[k] = a*x[k-1] + w[k], w[k] ~ N(0, sigma_w^2)
# ------------------------------------------------------------
a = 0.8
sigma_w = 1.0
K = 400
M_ar = 500

# Initialize from invariant Gaussian distribution for stationarity:
# Var[x] = sigma_w^2 / (1 - a^2)
sigma_x0 = sigma_w / np.sqrt(1.0 - a**2)
x = np.zeros((M_ar, K))
x[:, 0] = np.random.normal(0.0, sigma_x0, size=M_ar)
for k in range(1, K):
    w = np.random.normal(0.0, sigma_w, size=M_ar)
    x[:, k] = a * x[:, k - 1] + w

mean_ar = x.mean(axis=0)
var_ar = x.var(axis=0, ddof=0)

print("\nAR(1) stationary regime estimates:")
print("Mean at k = [0, 100, 200, 399]:", [float(mean_ar[i]) for i in [0, 100, 200, 399]])
print("Var  at k = [0, 100, 200, 399]:", [float(var_ar[i]) for i in [0, 100, 200, 399]])
print("Theoretical stationary variance:", sigma_w**2 / (1.0 - a**2))

# ------------------------------------------------------------
# Visualization
# ------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(9, 8), constrained_layout=True)

# Random-phase process statistics
ax[0].plot(t, mean_est, label="Estimated ensemble mean")
ax[0].plot(t, np.zeros_like(t), "--", label="Theory mean = 0")
ax[0].set_title("Random-phase harmonic process: ensemble mean")
ax[0].set_xlabel("t [s]")
ax[0].set_ylabel("Mean")
ax[0].grid(True)
ax[0].legend()

# Nonstationary process variance
ax[1].plot(t, Y_var, label="Estimated Var[Y(t)]")
ax[1].set_title("Nonstationary process Y(t)=alpha t + beta: variance changes with time")
ax[1].set_xlabel("t [s]")
ax[1].set_ylabel("Variance")
ax[1].grid(True)
ax[1].legend()

plt.show()
