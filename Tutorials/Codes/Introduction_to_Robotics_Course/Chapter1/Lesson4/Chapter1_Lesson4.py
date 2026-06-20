
import numpy as np

# Plant parameters
a, b = 1.02, 0.05
K = 1.5
Ts = 0.01
N = 2000

# Noise std devs
sigma_w = 0.02
sigma_v = 0.05

x = 0.0
xs, ys, us = [], [], []

for k in range(N):
    r = 1.0  # step reference
    v = np.random.randn() * sigma_v
    y = x + v                       # Sense
    u = -K * y + r                  # Think
    w = np.random.randn() * sigma_w
    x = a * x + b * u + w           # Act/Plant
    xs.append(x); ys.append(y); us.append(u)

print("Final state:", xs[-1])
      