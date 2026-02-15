
import numpy as np
import matplotlib.pyplot as plt

# Physical parameters of a simple pendulum-like joint
J = 0.05      # inertia [kg m^2]
b = 0.01      # viscous friction [N m s/rad]
m = 1.0       # link mass [kg]
ell = 0.5     # link COM distance [m]
g = 9.81      # gravity [m/s^2]

def gravity_torque(q):
    # simple planar joint: tau_g = m g l sin(q)
    return m * g * ell * np.sin(q)

# Desired position: step from 0 to qd at t = 0
qd = np.deg2rad(45.0)  # 45 degrees

# Continuous-time design: choose wn, zeta and derive Kp, Kd
wn = 8.0   # rad/s
zeta = 0.7
Kp = J * wn**2
Kd = 2.0 * J * zeta * wn - b

print("Kp =", Kp, "Kd =", Kd)

# Discrete simulation
Ts = 0.001          # controller and integration step [s]
t_final = 2.0
N = int(t_final / Ts)

q = 0.0             # initial position
dq = 0.0            # initial velocity

qs = np.zeros(N)
dqs = np.zeros(N)
taus = np.zeros(N)
ts = np.linspace(0.0, t_final, N)

for k in range(N):
    # controller (no integral term)
    e = qd - q
    de = 0.0 - dq
    tau = Kp * e + Kd * de + gravity_torque(q)

    # simple saturation
    tau_max = 10.0
    tau_min = -10.0
    tau = np.clip(tau, tau_min, tau_max)

    # joint dynamics: J ddq + b dq + g(q) = tau
    ddq = (tau - b * dq - gravity_torque(q)) / J

    # Euler integration
    dq = dq + Ts * ddq
    q = q + Ts * dq

    qs[k] = q
    dqs[k] = dq
    taus[k] = tau

# Plot results
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(ts, qs, label="q")
plt.axhline(qd, linestyle="--", label="q_d")
plt.ylabel("position [rad]")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(ts, dqs)
plt.ylabel("velocity [rad/s]")

plt.subplot(3, 1, 3)
plt.plot(ts, taus)
plt.ylabel("tau [N m]")
plt.xlabel("time [s]")

plt.tight_layout()
plt.show()
