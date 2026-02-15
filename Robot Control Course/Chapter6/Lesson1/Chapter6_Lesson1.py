
import numpy as np

m = 1.0       # mass
b = 1.0       # viscous damping
k_p = 200.0   # position gain
k_d = 20.0    # velocity gain
k_e = 5e3     # environment stiffness (very stiff)
x_s = 0.0     # wall at x = 0
F_max = 80.0  # crude force limit

dt = 0.0005
T = 1.0
N = int(T / dt)

# desired trajectory: step from -0.02 to +0.03 m (penetrates wall)
def x_d(t):
    return -0.02 if t < 0.1 else 0.03

x = -0.02
v = 0.0

xs = np.zeros(N)
Fs = np.zeros(N)
us = np.zeros(N)
ts = np.linspace(0.0, T, N)

for i, t in enumerate(ts):
    # environment force
    if x <= x_s:
        F_e = 0.0
    else:
        F_e = k_e * (x - x_s)

    # PD position control
    e = x_d(t) - x
    u = k_p * e - k_d * v

    # crude force limit
    if F_e > F_max:
        # reduce actuation in same direction as penetration
        u = u - (F_e - F_max)

    # dynamics integration (explicit Euler)
    a = (u - F_e - b * v) / m
    v = v + dt * a
    x = x + dt * v

    xs[i] = x
    Fs[i] = F_e
    us[i] = u

# xs: position trajectory
# Fs: contact forces
# us: control inputs
# These can be plotted (time vs x and time vs F_e) to visualize force spikes.
