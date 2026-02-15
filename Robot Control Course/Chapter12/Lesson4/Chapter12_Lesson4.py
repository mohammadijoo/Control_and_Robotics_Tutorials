
import time
import math
import random
import numpy as np

# Simple scalar joint model: x = [position; velocity]
A = np.array([[0.0, 1.0],
              [0.0, -2.0]])   # some damping
B = np.array([[0.0],
              [1.0]])         # torque input

# Nominal sampling period (seconds) and jitter bound
h_nom = 0.002   # 2 ms
jitter_max = 0.0005  # +/- 0.5 ms

# PD gains
Kp = 50.0
Kd = 5.0

x = np.array([[0.5],
              [0.0]])  # initial position error
xref = 0.0

def step_euler(x, u, h):
    return x + h * (A @ x + B * u)

t = 0.0
for k in range(2000):
    # Compute control
    pos = x[0, 0]
    vel = x[1, 0]
    e = xref - pos
    u = Kp * e - Kd * vel

    # Draw jittered step size
    h = h_nom + random.uniform(-jitter_max, jitter_max)

    # Integrate
    x = step_euler(x, u, h)
    t += h

    if k % 100 == 0:
        print(f"k = {k}, t = {t:.4f}, pos = {pos:.4f}, h = {h:.6f}")

    # In a real RT system the sleep is driven by a timer interrupt
    time.sleep(h_nom)
