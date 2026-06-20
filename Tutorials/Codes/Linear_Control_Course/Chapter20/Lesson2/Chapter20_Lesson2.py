import numpy as np
import matplotlib.pyplot as plt

# First-order plant: dy/dt = -(1/T)*y + (1/T)*u
T = 0.5

# PID gains (already tuned as 1-DOF)
Kp = 2.0
Ki = 5.0
Kd = 0.1

# Set-point weights
b = 0.6   # proportional weight
c = 0.0   # derivative weight (no derivative kick)

# Simulation settings
dt = 0.001
t_final = 5.0
n_steps = int(t_final / dt)

t = np.linspace(0.0, t_final, n_steps)
y = np.zeros(n_steps)
u = np.zeros(n_steps)
r = np.ones(n_steps)  # unit step reference

# States for integral and derivative action
xi = 0.0
e_d_prev = 0.0

for k in range(n_steps - 1):
    # Current signals
    rk = r[k]
    yk = y[k]

    # 2-DOF pseudo-errors
    e_p = b * rk - yk           # proportional pseudo-error
    e_i = rk - yk               # integral error (unweighted)
    e_d = c * rk - yk           # derivative pseudo-error

    # Integrator update (rectangle rule)
    xi = xi + e_i * dt

    # Derivative of e_d, with simple backward difference
    de_d = (e_d - e_d_prev) / dt
    e_d_prev = e_d

    # Control signal
    uk = Kp * e_p + Ki * xi + Kd * de_d
    u[k] = uk

    # Plant integration (forward Euler)
    dy = (-(1.0 / T) * yk + (1.0 / T) * uk) * dt
    y[k + 1] = yk + dy

# Plot results
plt.figure()
plt.plot(t, r, label="reference r")
plt.plot(t, y, label="output y")
plt.xlabel("time [s]")
plt.ylabel("signals")
plt.legend()
plt.title("2-DOF PID with set-point weighting (Python)")
plt.grid(True)
plt.show()
