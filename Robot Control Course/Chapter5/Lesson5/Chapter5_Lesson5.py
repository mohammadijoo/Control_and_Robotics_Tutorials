
import numpy as np
import matplotlib.pyplot as plt

# Physical parameters
m = 1.0                # kg
k_env = 5000.0         # N/m (worst-case environment stiffness)
F_safe = 80.0          # N (allowed peak force)
a_brake = 30.0         # m/s^2 (approx. max decel)
v_max = 1.0            # m/s (global speed limit)

# Derived parameters
v_force = F_safe / np.sqrt(m * k_env)

# Simulation parameters
h = 0.0005             # s
T = 1.0                # total time
N = int(T / h)

# Controller gains (nominal PD in position)
x_target = 0.0         # wall at x=0
k_p = 50.0
k_d = 5.0
k_v = 200.0            # velocity servo gain

# Initial conditions
x = 0.25               # 25 cm away from wall
v = 0.0

xs = np.zeros(N)
vs = np.zeros(N)
Fs = np.zeros(N)
ts = np.linspace(0.0, T, N)

for k in range(N):
    # Distance to wall
    d = x

    # Nominal desired velocity from PD on position
    # Negative sign moves toward the wall at x=0
    v_des = -k_p * (x - x_target) - k_d * v

    # Compute distance-based braking bound (non-negative)
    v_brake = np.sqrt(max(0.0, 2.0 * a_brake * d))

    # Safe velocity bound
    v_safe_bound = min(v_force, v_brake, v_max)

    # Apply safety filter only when moving toward wall (v_des < 0)
    if v_des < -v_safe_bound:
        v_safe_cmd = -v_safe_bound
    else:
        v_safe_cmd = v_des

    # Simple velocity-tracking control (u = m k_v (v_safe_cmd - v))
    u = m * k_v * (v_safe_cmd - v)

    # Integrate dynamics
    v = v + (h / m) * u
    x = x + h * v

    # Contact force model (penetration x < 0)
    F = 0.0
    if x < 0.0:
        F = -k_env * x

    xs[k] = x
    vs[k] = v
    Fs[k] = F

# Post-simulation checks
print("Max penetration (m):", np.min(xs))
print("Max contact force (N):", np.max(np.abs(Fs)))
print("Theoretical safe bound F_safe (N):", F_safe)

plt.figure()
plt.subplot(3,1,1)
plt.plot(ts, xs)
plt.ylabel("x [m]")

plt.subplot(3,1,2)
plt.plot(ts, vs)
plt.ylabel("v [m/s]")

plt.subplot(3,1,3)
plt.plot(ts, Fs)
plt.ylabel("F [N]")
plt.xlabel("t [s]")
plt.tight_layout()
plt.show()
