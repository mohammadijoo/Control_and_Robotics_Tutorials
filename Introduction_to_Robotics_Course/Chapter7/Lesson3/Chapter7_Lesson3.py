import numpy as np

# ----- (A) Time-of-flight estimation by correlation -----
fs = 200_000  # sample rate (Hz), typical for ultrasonic processing
T_true = 0.0075  # true round-trip delay (s)
v_air = 343.0    # assumed speed (m/s)

# Transmit pulse: windowed sinusoid
t = np.arange(0, 0.01, 1/fs)
f0 = 40_000
s = np.sin(2*np.pi*f0*t) * (t < 0.001)

# Echo: delayed + attenuated + noise
delay_samples = int(T_true * fs)
alpha = 0.6
r = np.zeros_like(s)
r[delay_samples:] = alpha * s[:-delay_samples]
r += 0.02 * np.random.randn(len(r))

# Correlation-based delay estimate
C = np.correlate(r, s, mode="full")
lags = np.arange(-len(s)+1, len(s))
T_hat = lags[np.argmax(C)] / fs

d_hat = v_air * T_hat / 2
print("Estimated delay (s):", T_hat)
print("Estimated range (m):", d_hat)

# ----- (B) LiDAR scan to Cartesian points -----
K = 360
phi = np.deg2rad(np.arange(K))
r_k = 2.0 + 0.2*np.sin(3*phi)  # synthetic range profile (m)

x = r_k * np.cos(phi)
y = r_k * np.sin(phi)

points = np.stack([x, y], axis=1)
print("First 5 points:\n", points[:5])
