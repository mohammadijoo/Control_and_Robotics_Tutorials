import numpy as np
import matplotlib.pyplot as plt

# Sampling and simulation horizon
Ts = 1e-4
T_end = 0.2
N = int(T_end / Ts)

# Motor parameters (simplified)
J = 0.01     # inertia
B = 0.001    # viscous damping
L = 1e-3     # inductance
R = 0.5      # resistance
Kt = 0.05    # torque constant
Ke = 0.05    # back-EMF constant

# PI gains for the three loops
Kp_i, Ki_i = 50.0, 5e3      # current loop
Kp_w, Ki_w = 2.0, 200.0     # speed loop
Kp_th, Ki_th = 20.0, 200.0  # position loop

# States
i = 0.0       # current
w = 0.0       # speed
theta = 0.0   # position

xi_i = 0.0    # integral current
xi_w = 0.0    # integral speed
xi_th = 0.0   # integral position

theta_ref = np.deg2rad(90.0)  # 90 degrees step

time = np.zeros(N)
theta_hist = np.zeros(N)
w_hist = np.zeros(N)
i_hist = np.zeros(N)

for k in range(N):
    # Outer position loop
    e_th = theta_ref - theta
    xi_th += e_th * Ts
    w_ref = Kp_th * e_th + Ki_th * xi_th

    # Middle speed loop
    e_w = w_ref - w
    xi_w += e_w * Ts
    i_ref = Kp_w * e_w + Ki_w * xi_w

    # Inner current loop
    e_i = i_ref - i
    xi_i += e_i * Ts
    v = Kp_i * e_i + Ki_i * xi_i

    # Motor dynamics (continuous-time model discretized with Euler)
    di = (-R * i + v - Ke * w) / L
    dw = (-B * w + Kt * i) / J
    dtheta = w

    i += di * Ts
    w += dw * Ts
    theta += dtheta * Ts

    time[k] = k * Ts
    theta_hist[k] = theta
    w_hist[k] = w
    i_hist[k] = i

# Plot position response
plt.figure()
plt.plot(time, theta_hist, label="theta(t)")
plt.axhline(theta_ref, linestyle="--", label="theta_ref")
plt.xlabel("Time [s]")
plt.ylabel("Position [rad]")
plt.legend()
plt.grid(True)
plt.show()
