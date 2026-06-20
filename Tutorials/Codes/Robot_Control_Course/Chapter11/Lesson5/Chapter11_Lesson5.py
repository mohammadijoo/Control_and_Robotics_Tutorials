
import numpy as np
import matplotlib.pyplot as plt

# Physical and model parameters
M = 2.0
b = 0.8
k = 5.0

A = np.array([[0.0, 1.0],
              [-k / M, -b / M]])
B = np.array([[0.0],
              [1.0 / M]])
C = np.array([[1.0, 0.0]])

# State-feedback and observer gains from Section 4
K = np.array([[27.0, 13.6]])        # 1x2
L = np.array([[28.4],
              [242.14]])            # 2x1

# Simulation settings
dt = 1e-3
T_end = 5.0
N = int(T_end / dt)

# Reference (regulation around zero; you can change q_ref to test tracking)
q_ref = 0.5
x_ref = np.array([[q_ref],
                  [0.0]])

# State initialization (true and estimated)
x = np.array([[0.0],
              [0.0]])
x_hat = np.array([[0.0],
                  [0.0]])

t_vec = np.zeros(N)
q_vec = np.zeros(N)
q_hat_vec = np.zeros(N)
u_vec = np.zeros(N)

for k_step in range(N):
    t = k_step * dt
    t_vec[k_step] = t

    # Output and estimation error in output
    y = C @ x
    y_hat = C @ x_hat
    y_err = y - y_hat

    # Control law (reference in state coordinates)
    u = -K @ (x_hat - x_ref)

    # Store for plotting
    q_vec[k_step] = y.item()
    q_hat_vec[k_step] = y_hat.item()
    u_vec[k_step] = u.item()

    # Continuous-time derivatives
    x_dot = A @ x + B * u
    x_hat_dot = A @ x_hat + B * u + L * y_err

    # Euler integration
    x = x + dt * x_dot
    x_hat = x_hat + dt * x_hat_dot

# Plot results
plt.figure()
plt.plot(t_vec, q_vec, label="q (true)")
plt.plot(t_vec, q_hat_vec, linestyle="--", label="q_hat (estimate)")
plt.axhline(q_ref, linestyle=":", label="q_ref")
plt.xlabel("Time [s]")
plt.ylabel("Joint position [rad]")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(t_vec, u_vec)
plt.xlabel("Time [s]")
plt.ylabel("Torque command u [N*m]")
plt.grid(True)

plt.show()
