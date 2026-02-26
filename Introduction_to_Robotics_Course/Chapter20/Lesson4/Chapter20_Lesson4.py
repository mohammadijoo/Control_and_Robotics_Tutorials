import numpy as np

# Example: step response tracking
# t: time stamps [s], r: reference signal, y: measured output
T = 10.0
N = 1001
t = np.linspace(0.0, T, N)
r = np.ones_like(t)                  # unit step reference
y = 1.0 - np.exp(-t) * np.cos(2.0*t) # example response
e = r - y

dt = t[1] - t[0]

# Signal-based metrics
iae = np.sum(np.abs(e)) * dt
ise = np.sum(e**2) * dt
itae = np.sum(t * np.abs(e)) * dt
rmse = np.sqrt(np.mean(e**2))
max_err = np.max(np.abs(e))

print(f"IAE  = {iae:.4f}")
print(f"ISE  = {ise:.4f}")
print(f"ITAE = {itae:.4f}")
print(f"RMSE = {rmse:.4f}")
print(f"max |e| = {max_err:.4f}")

# Suppose we have multiple trials for a path-tracking experiment:
# errors_trials: list of numpy arrays of per-sample errors
errors_trials = [e + 0.01*np.random.randn(N) for _ in range(10)]
rmse_trials = [np.sqrt(np.mean(err**2)) for err in errors_trials]

rmse_mean = np.mean(rmse_trials)
rmse_std = np.std(rmse_trials, ddof=1)

print(f"Mean RMSE over trials = {rmse_mean:.4f}")
print(f"Std dev of RMSE       = {rmse_std:.4f}")
      
