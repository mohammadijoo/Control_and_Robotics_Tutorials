import numpy as np

# -----------------------------
# (A) Scalar weighted fusion
# -----------------------------
z = np.array([1.02, 0.97, 1.10])   # three sensors
w = np.array([4.0, 2.0, 1.0])     # confidences
x_hat = (w @ z) / np.sum(w)
print("Scalar fused estimate:", x_hat)

# -----------------------------
# (B) Vector WLS fusion
# z_i = H_i x + e_i
# -----------------------------
# state x = [x_position, y_position]^T
H1 = np.eye(2)                    # GPS-like direct position
z1 = np.array([2.0, 1.0])

H2 = np.array([[1.0, 0.0]])       # range-only sensor to x-axis
z2 = np.array([1.8])

W1 = np.eye(2) * 5.0              # higher confidence
W2 = np.eye(1) * 2.0

A = H1.T @ W1 @ H1 + H2.T @ W2 @ H2
b = H1.T @ W1 @ z1 + H2.T @ W2 @ z2
x_hat_vec = np.linalg.solve(A, b)
print("Vector fused estimate:", x_hat_vec)

# -----------------------------
# (C) Discrete complementary filter
# -----------------------------
T = 0.01
omega_c = 2.0
alpha = (omega_c * T) / (1.0 + omega_c * T)

# y1: low-frequency measurement (e.g., encoder position)
# u2: high-frequency rate (e.g., IMU velocity increment)
y1 = np.sin(np.linspace(0, 2, 200)) + 0.05*np.random.randn(200)
u2 = np.gradient(y1, T) + 0.2*np.random.randn(200)

x_hat = np.zeros_like(y1)
for k in range(1, len(y1)):
    x_hat[k] = (1-alpha)*(x_hat[k-1] + T*u2[k]) + alpha*y1[k]

print("Complementary filter done.")
