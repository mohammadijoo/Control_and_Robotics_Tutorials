
import numpy as np
import matplotlib.pyplot as plt

# Parameters
Kp = 5.0       # proportional gain
tau = 0.2      # time constant
Ts = 0.001     # sampling time
T_end = 1.0
n_steps = int(T_end / Ts)

# Preallocate
t = Ts * np.arange(n_steps + 1)
r = np.ones_like(t)   # unit step reference
y = np.zeros_like(t)  # output
u = np.zeros_like(t)  # control

# Discrete-time first-order model: y[k+1] = y[k] + Ts * ( -y[k]/tau + u[k]/tau )
for k in range(n_steps):
    e_k = r[k] - y[k]
    u[k] = Kp * e_k
    dy = (-y[k] + u[k]) / tau
    y[k + 1] = y[k] + Ts * dy

# Plot
plt.figure()
plt.plot(t, r, linestyle="--", label="reference")
plt.plot(t, y, label="output")
plt.xlabel("time (s)")
plt.ylabel("joint position")
plt.legend()
plt.grid(True)
plt.show()
