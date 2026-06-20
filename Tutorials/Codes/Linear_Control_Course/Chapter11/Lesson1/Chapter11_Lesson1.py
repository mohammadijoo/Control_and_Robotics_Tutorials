import numpy as np
import matplotlib.pyplot as plt

# Plant parameters (e.g., normalized DC motor)
k = 1.0       # static gain
tau = 0.5     # time constant (s)

# P controller gain
Kp = 5.0

# Simulation parameters
dt = 0.001
t_final = 5.0
n_steps = int(t_final / dt)

# Reference (unit step)
r = 1.0

# State and logs
y = 0.0
u = 0.0
umax = 10.0  # actuator saturation (for realism)

t_list = [0.0]
y_list = [y]
u_list = [u]

t = 0.0
for k_step in range(n_steps):
    # P control law
    e = r - y
    u = Kp * e

    # simple saturation
    u = max(min(u, umax), -umax)

    # first-order plant dynamics (explicit Euler)
    dy = (-1.0 / tau) * y + (k / tau) * u
    y = y + dy * dt

    t = t + dt
    t_list.append(t)
    y_list.append(y)
    u_list.append(u)

# Plot results
plt.figure()
plt.plot(t_list, y_list, label="Output y(t)")
plt.axhline(r, linestyle="--", label="Reference r(t)")
plt.xlabel("Time [s]")
plt.ylabel("Output / Reference")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(t_list, u_list, label="Control input u(t)")
plt.xlabel("Time [s]")
plt.ylabel("Control input")
plt.legend()
plt.grid(True)

plt.show()
