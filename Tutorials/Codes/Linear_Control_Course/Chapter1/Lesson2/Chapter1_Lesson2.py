import numpy as np
import matplotlib.pyplot as plt

# Plant parameters
a = 1.0
b = 1.0

# Controller gain
K = 2.0

# Discretization
h = 0.01      # time step [s]
N = 1000      # number of steps
r = 1.0       # constant reference

t = np.linspace(0.0, N * h, N + 1)

y_open = np.zeros(N + 1)
y_closed = np.zeros(N + 1)

# Choose open-loop input to roughly achieve y ≈ 1 at steady state
u_open = a / b * r

for k in range(N):
    # Open-loop update
    y_open[k + 1] = y_open[k] + h * (-a * y_open[k] + b * u_open)

    # Closed-loop update
    e_k = r - y_closed[k]
    u_closed = K * e_k
    y_closed[k + 1] = y_closed[k] + h * (-a * y_closed[k] + b * u_closed)

# Plot for visualization
plt.plot(t, y_open, label="open-loop")
plt.plot(t, y_closed, label="closed-loop")
plt.axhline(r, linestyle="--", label="reference")
plt.xlabel("time [s]")
plt.ylabel("output y(t)")
plt.grid(True)
plt.legend()
plt.show()
