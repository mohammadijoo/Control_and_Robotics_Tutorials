
import numpy as np

# Physical parameters
J = 0.05  # joint inertia [kg m^2]
K = 10.0  # virtual stiffness [Nm/rad]
D = 2.0   # virtual damping [Nms/rad]

# Desired trajectory: step from 0 to 0.5 rad
def q_d(t):
    return 0.5 if t >= 0.1 else 0.0

def qd_d(t):
    return 0.0

def qdd_d(t):
    return 0.0

# External torque (e.g. contact disturbance)
def tau_ext(t):
    # short pulse disturbance
    return 0.2 if 0.3 <= t <= 0.35 else 0.0

# Simulation setup
dt = 0.001
T = 1.0
N = int(T / dt)

q = 0.0
qd = 0.0

ts = []
qs = []
qds = []
taus = []

for k in range(N):
    t = k * dt

    # Desired signals
    q_ref = q_d(t)
    qd_ref = qd_d(t)
    qdd_ref = qdd_d(t)

    e = q - q_ref
    ed = qd - qd_ref

    # Impedance control law:
    # tau_m = J * qdd_ref - K * e - D * ed
    tau_m = J * qdd_ref - K * e - D * ed

    # Plant dynamics: J * qdd = tau_m + tau_ext
    qdd = (tau_m + tau_ext(t)) / J

    # Euler integration (for illustration)
    qd = qd + dt * qdd
    q = q + dt * qd

    ts.append(t)
    qs.append(q)
    qds.append(qd)
    taus.append(tau_m)

# Plotting (if matplotlib is available)
try:
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(ts, qs, label="q(t)")
    plt.plot(ts, [q_d(t) for t in ts], "--", label="q_d(t)")
    plt.xlabel("time [s]")
    plt.ylabel("position [rad]")
    plt.legend()
    plt.title("1-DOF Joint Impedance Control (Python)")
    plt.show()
except ImportError:
    print("matplotlib not installed; skipping plots.")
