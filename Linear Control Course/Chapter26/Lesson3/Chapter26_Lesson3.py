import numpy as np

# Parameters
dt   = 0.001          # sampling period [s]
T    = 2.0            # total simulation time [s]
tau  = 0.02           # filter time constant [s]
omega_signal = 2.0    # rad/s (slow motion)
noise_std    = 0.05   # sensor noise standard deviation

# Discrete-time filter coefficient (ZOH-discretized first-order LPF)
alpha = np.exp(-dt / tau)

# Time vector
t = np.arange(0.0, T, dt)
N = len(t)

# True signal: smooth sinusoidal motion (e.g., joint angle)
y_true = 1.0 * np.sin(omega_signal * t)

# Noisy measurement (white Gaussian noise)
rng = np.random.default_rng(seed=0)
noise = noise_std * rng.standard_normal(N)
y_meas = y_true + noise

# Filtered measurement
y_filt = np.zeros_like(y_meas)
y_filt[0] = y_meas[0]
for k in range(1, N):
    y_filt[k] = alpha * y_filt[k - 1] + (1.0 - alpha) * y_meas[k]

# Example: compute RMS of noise before and after filtering
noise_meas = y_meas - y_true
noise_filt = y_filt - y_true
rms_meas = np.sqrt(np.mean(noise_meas**2))
rms_filt = np.sqrt(np.mean(noise_filt**2))
print("RMS noise before filter:", rms_meas)
print("RMS noise after  filter:", rms_filt)

# In a ROS2 node (rclpy), this recursion can be applied inside the callback
# that receives sensor messages, publishing y_filt instead of raw y_meas.
