import numpy as np

# Continuous-time "true" signal sampled very finely for simulation
fs_true = 5000.0
T_true = 1.0 / fs_true
t = np.arange(0, 2.0, T_true)
x = np.sin(2*np.pi*3*t) + 0.4*np.sin(2*np.pi*40*t)  # low + high freq content

# Sensor noise and drift models
sigma_n = 0.05
sigma_b = 1e-4
n = np.random.normal(0, sigma_n, size=t.size)

b = np.zeros_like(t)
for k in range(1, t.size):
    b[k] = b[k-1] + np.random.normal(0, sigma_b)

y_analog = x + n + b

# Sampling
fs = 100.0
Ts = 1.0 / fs
idx = (t / Ts).astype(int)
idx = np.unique(idx)
t_s = t[idx]
y_s = y_analog[idx]

# Quantization (B bits on range [-2,2])
B = 10
ymin, ymax = -2.0, 2.0
Delta = (ymax - ymin) / (2**B)
y_q = Delta * np.round(y_s / Delta)

print("Quantization step Delta:", Delta)
print("Measured variance (noise+quant):", np.var(y_q - np.sin(2*np.pi*3*t_s)))

# Simple bias estimation by averaging over a known "static" interval
# Here we pretend first 0.2s is static so x≈0 there.
static_mask = t_s < 0.2
b_hat = np.mean(y_q[static_mask])
y_comp = y_q - b_hat
print("Estimated bias:", b_hat)
