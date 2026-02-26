import numpy as np
import matplotlib.pyplot as plt

# Plant and controller parameters
a = 1.0      # plant parameter > 0
b = 2.0      # plant parameter > 0
K = 3.0      # proportional gain
r0 = 1.0     # constant reference

# Simulation settings
h = 0.001          # time step
t_final = 2.0
N = int(t_final / h)

t = np.linspace(0.0, t_final, N + 1)
y = np.zeros(N + 1)
u = np.zeros(N + 1)

# Initial condition
y[0] = 0.0

for k in range(N):
    e_k = r0 - y[k]
    u[k] = K * e_k
    dy = -(a + b * K) * y[k] + b * K * r0
    y[k + 1] = y[k] + h * dy

u[-1] = u[-2]

plt.figure()
plt.plot(t, y, label="y(t)")
plt.axhline(r0, linestyle="--", label="reference r0")
plt.xlabel("time [s]")
plt.ylabel("output")
plt.legend()
plt.title("First-order closed-loop response (Python simulation)")
plt.show()

# Note:
# For more advanced work with robotics models, see:
#   - python-control: https://python-control.readthedocs.io
#   - roboticstoolbox-python: https://github.com/petercorke/robotics-toolbox-python
