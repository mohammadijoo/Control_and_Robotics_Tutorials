# Chapter17_Lesson4.py
# Noise Modeling in Sensors and Actuators for Dynamic Systems
# Python simulation: mass-spring-damper with actuator colored noise,
# process noise, sensor bias drift, measurement noise, and quantization.

import numpy as np

np.random.seed(17)

# Physical parameters
m = 1.2
c = 0.35
k = 4.0
Ts = 0.01
N = 6000

# Continuous-time matrices (x = [position, velocity]^T)
A_c = np.array([[0.0, 1.0],
                [-k/m, -c/m]])
B_c = np.array([[0.0],
                [1.0/m]])
G_c = np.array([[0.0],
                [1.0/m]])  # process disturbance input channel
C = np.array([[1.0, 0.0]]) # position sensor

# Simple discretization (forward Euler for teaching/demo)
A = np.eye(2) + Ts * A_c
B = Ts * B_c
G = Ts * G_c

# Noise parameters
sigma_w = 0.20             # process force noise std (white)
sigma_v = 0.01             # sensor white noise std
sigma_bias_rw = 0.002      # bias random-walk intensity (per sqrt(s))
delta_q = 0.001            # quantization step
rho_a = 0.97               # actuator colored-noise AR(1) coefficient
sigma_a_ss = 0.08          # actuator noise steady-state std

# Covariances
Qw = np.array([[sigma_w**2]])
Qa = np.array([[sigma_a_ss**2]])
Rv = np.array([[sigma_v**2]])
Rq = np.array([[delta_q**2 / 12.0]])  # uniform quantization model

# Input command
t = np.arange(N) * Ts
u_cmd = 0.8 * np.sin(2.0 * np.pi * 0.7 * t)

# Storage
x = np.zeros((2, N))
y = np.zeros(N)
y_true = np.zeros(N)
bias = 0.0
a_noise = 0.0

# Covariance recursion (mean-subtracted, independent noises)
P = np.zeros((2, 2))
P_hist = np.zeros((N, 2, 2))
Syy_hist = np.zeros(N)

for k_idx in range(N - 1):
    # Colored actuator noise (AR(1))
    a_noise = rho_a * a_noise + np.sqrt(1.0 - rho_a**2) * sigma_a_ss * np.random.randn()

    # Process white force noise
    w_k = sigma_w * np.random.randn()

    # Plant update
    u_actual = u_cmd[k_idx] + a_noise
    x[:, k_idx + 1] = (A @ x[:, k_idx]
                       + (B[:, 0] * u_actual)
                       + (G[:, 0] * w_k))

    # Sensor bias drift (random walk)
    bias = bias + sigma_bias_rw * np.sqrt(Ts) * np.random.randn()

    # Sensor white noise + quantization
    v_k = sigma_v * np.random.randn()
    y_analog = (C @ x[:, k_idx + 1])[0] + bias + v_k
    q_k = delta_q * np.round(y_analog / delta_q) - y_analog
    y[k_idx + 1] = y_analog + q_k
    y_true[k_idx + 1] = (C @ x[:, k_idx + 1])[0]

    # Theoretical covariance propagation for the plant state
    # P_{k+1} = A P_k A^T + G Qw G^T + B Qa B^T
    P = A @ P @ A.T + G @ Qw @ G.T + B @ Qa @ B.T
    P_hist[k_idx + 1] = P

    # Predicted measurement variance excluding bias random walk accumulation
    Syy = C @ P @ C.T + Rv + Rq
    Syy_hist[k_idx + 1] = Syy[0, 0]

# Empirical statistics after transient
burn = 1000
y_centered = y[burn:] - np.mean(y[burn:])
y_true_centered = y_true[burn:] - np.mean(y_true[burn:])

emp_var_y = np.var(y_centered, ddof=1)
emp_var_ytrue = np.var(y_true_centered, ddof=1)
pred_var_y = np.mean(Syy_hist[burn:])  # excludes long-term bias random walk growth

# Estimate quantization variance empirically
q_est = y[burn:] - (np.round(y[burn:] / delta_q) * delta_q)  # residual in [-delta_q/2, delta_q/2]
q_est = q_est - np.mean(q_est)
emp_var_q = np.var(q_est, ddof=1)

print("=== Noise Modeling Demo (Python) ===")
print(f"Sampling time Ts = {Ts:.4f} s, N = {N}")
print(f"Empirical var(y_true)        : {emp_var_ytrue:.6e}")
print(f"Empirical var(y_measured)    : {emp_var_y:.6e}")
print(f"Predicted var(y) (no bias RW): {pred_var_y:.6e}")
print(f"Quantization var theory      : {delta_q**2/12.0:.6e}")
print(f"Quantization var empirical   : {emp_var_q:.6e}")

# Optional simple PSD-like diagnostic using FFT (periodogram)
Y = np.fft.rfft(y_centered)
freqs = np.fft.rfftfreq(len(y_centered), d=Ts)
psd = (np.abs(Y) ** 2) / (len(y_centered) / Ts)

# Print a few dominant frequencies
idx = np.argsort(psd)[-5:][::-1]
print("\nTop spectral peaks (Hz, PSD):")
for i in idx:
    print(f"{freqs[i]:8.3f} Hz, {psd[i]:.6e}")
