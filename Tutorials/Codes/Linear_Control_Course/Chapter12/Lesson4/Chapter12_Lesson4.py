import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Simple second-order joint model: G(s) = 1 / (J s^2 + B s + K)
J = 0.01
B = 0.1
K = 0.0
numG = [1.0]
denG = [J, B, K + 1.0]  # add unity feedback stiffness term
G = ctrl.TransferFunction(numG, denG)

# PID gains
Kp = 50.0
Ki = 0.0
Kd = 2.0
N  = 20.0   # derivative filter parameter

# PID controller with filtered derivative: C(s) = Kp + Ki/s + Kd * (N s)/(1 + N s)
C = ctrl.pid(Kp, Ki, Kd, N)

# Closed-loop from reference to output (unity feedback)
T = ctrl.feedback(C*G, 1)

t = np.linspace(0.0, 2.0, 2000)
t, y = ctrl.step_response(T, T=t)

# Approximate actuator signal u(t) via simulation with measurement noise
dt = t[1] - t[0]
r = np.ones_like(t)       # unit step reference
noise_std = 0.01          # measurement noise standard deviation
rng = np.random.default_rng(0)

xG = np.zeros(len(denG) - 1)
u_hist = []
y_hist = []
e_prev = 0.0
d_filt = 0.0

alpha = np.exp(-dt * N)   # discrete low-pass coefficient for derivative filter

for k in range(len(t)):
    y_meas = xG[-1] + noise_std * rng.normal()
    e = r[k] - y_meas

    # discrete-time filtered derivative of error
    d_raw = (e - e_prev) / dt
    d_filt = alpha * d_filt + (1.0 - alpha) * d_raw

    u = Kp * e + Kd * d_filt  # (Ki = 0 in this example)
    u_hist.append(u)
    y_hist.append(y_meas)

    # simple plant integration: forward Euler on state-space form
    # J q_dd + B q_d + K q = u
    # state x = [q, q_d]
    q = xG[0]
    qd = xG[1]
    qdd = (u - B * qd - K * q) / J
    qd_new = qd + dt * qdd
    q_new = q + dt * qd
    xG = np.array([q_new, qd_new])

    e_prev = e

u_hist = np.array(u_hist)

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, y_hist, label="measured position")
plt.plot(t, r, linestyle="--", label="reference")
plt.ylabel("position")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, u_hist)
plt.xlabel("time [s]")
plt.ylabel("control torque u(t)")
plt.tight_layout()
plt.show()
