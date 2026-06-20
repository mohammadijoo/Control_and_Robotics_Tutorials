
import numpy as np
import matplotlib.pyplot as plt

# Physical parameters
m = 1.0          # kg
k_env = 5000.0   # N/m environment stiffness
x0 = 0.05        # initial position (m), positive = above surface

# Impedance parameters
m_c = 0.5
k_c = 2000.0
# critical damping approximation: d_c = 2 * sqrt(m_c * k_c)
d_c = 2.0 * np.sqrt(m_c * k_c)

# Desired normal force (positive compressive)
f_n_des = 20.0   # N
x_d = 0.0        # desired contact position at the surface

# Simulation parameters
T_s = 0.0005     # sampling period (s)
T_end = 0.8      # total simulation time (s)
N = int(T_end / T_s)

# State vectors
x = np.zeros(N)
v = np.zeros(N)
f_env = np.zeros(N)
f_cmd = np.zeros(N)

# Error history for discrete derivatives
e = np.zeros(N)

# Initial conditions
x[0] = x0
v[0] = 0.0
e_prev1 = 0.0
e_prev2 = 0.0

for k in range(2, N):
    # Environment force (unilateral spring)
    if x[k-1] > 0.0:
        f_env[k-1] = 0.0
    else:
        f_env[k-1] = -k_env * x[k-1]

    # Position error
    e[k] = x[k-1] - x_d

    # Discrete-time derivatives
    de = (e[k] - e_prev1) / T_s
    dde = (e[k] - 2.0 * e_prev1 + e_prev2) / (T_s ** 2)

    # Impedance law (control force)
    f_c = m_c * dde + d_c * de + k_c * e[k] - f_env[k-1] + f_n_des

    # Simple saturation (safety)
    f_c = np.clip(f_c, -200.0, 200.0)

    f_cmd[k] = f_c

    # Plant integration (semi-implicit Euler)
    a = (f_c + f_env[k-1]) / m
    v[k] = v[k-1] + T_s * a
    x[k] = x[k-1] + T_s * v[k]

    # Shift error history
    e_prev2 = e_prev1
    e_prev1 = e[k]

# Plot results
t = np.arange(N) * T_s

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, x, label="x (m)")
plt.axhline(0.0, linestyle="--", label="surface")
plt.ylabel("Position (m)")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, f_env, label="Contact force (N)")
plt.axhline(f_n_des, linestyle="--", label="desired force")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.tight_layout()
plt.show()
