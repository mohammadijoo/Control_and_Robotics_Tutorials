import numpy as np

# Simulated analog signal x(t) sampled at Ts
Ts = 0.01
t = np.arange(0, 2, Ts)
x_true = np.sin(2*np.pi*1.0*t)  # 1 Hz signal

# Add sensor noise (zero-mean Gaussian)
sigma = 0.2
y = x_true + np.random.normal(0, sigma, size=t.shape)

# Quantization (b-bit ADC over [-1.5, 1.5])
b = 8
ymin, ymax = -1.5, 1.5
Delta = (ymax - ymin) / (2**b)
yq = ymin + Delta * np.round((y - ymin) / Delta)

# First-order low-pass filter
tau = 0.1
alpha = Ts / (tau + Ts)  # matches alpha = Ts/(tau+Ts)
s_hat = np.zeros_like(yq)
for k in range(1, len(yq)):
    s_hat[k] = (1 - alpha) * s_hat[k-1] + alpha * yq[k]

print("Delta =", Delta)
print("Filtered estimate variance approx =", np.var(s_hat - x_true))
