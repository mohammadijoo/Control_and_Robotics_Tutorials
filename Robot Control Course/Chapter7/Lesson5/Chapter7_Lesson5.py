
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Physical parameters of 1-DOF rotary link
m = 1.0     # kg
l = 0.5     # m
g = 9.81    # m/s^2
I = m * l**2

# Robust control gains
lam = 4.0       # lambda in s = e_dot + lam * e
k_s = 5.0       # sliding gain
phi = 0.1       # boundary layer width
alpha = 30.0    # disturbance observer bandwidth

# Reference trajectory and derivatives
def q_d(t):
    return 0.5 * np.sin(1.0 * t)

def dq_d(t):
    return 0.5 * np.cos(1.0 * t)

def ddq_d(t):
    return -0.5 * np.sin(1.0 * t)

# Disturbance model: smooth + Coulomb-like component
def disturbance(t, q, dq):
    return 1.5 * np.sin(3.0 * t) + 0.5 * np.sign(dq + 1e-6)

def sat(s, phi):
    sigma = s / phi
    if sigma > 1.0:
        return 1.0
    elif sigma < -1.0:
        return -1.0
    else:
        return sigma

def dynamics(t, x):
    # state x = [q, dq, d_hat]
    q, dq, d_hat = x

    # tracking errors
    e = q - q_d(t)
    de = dq - dq_d(t)
    s = de + lam * e

    # nominal control
    # gravity term for pendulum-like link
    g_term = m * g * l * np.sin(q)
    tau_nom = I * (ddq_d(t) - lam * de) + g_term

    # robust sliding term with boundary layer
    tau_rob = -k_s * sat(s, phi)

    # total control: with disturbance compensation
    tau = tau_nom + tau_rob - d_hat

    # true disturbance
    d = disturbance(t, q, dq)

    # plant dynamics: I * ddq + g(q) = tau + d
    ddq = (tau + d - g_term) / I

    # approximate q_ddot using model (for observer)
    # here we use model-based estimate: q_ddot_est = ddq
    q_ddot_est = ddq

    # disturbance observer dynamics
    d_hat_dot = -alpha * d_hat + alpha * (I * q_ddot_est + g_term - tau)

    return [dq, ddq, d_hat_dot]

# Initial conditions
x0 = [0.0, 0.0, 0.0]  # q(0), dq(0), d_hat(0)

tf = 20.0
sol = solve_ivp(dynamics, [0.0, tf], x0, max_step=1e-3, rtol=1e-6, atol=1e-8)
t = sol.t
q = sol.y[0]
dq = sol.y[1]
d_hat = sol.y[2]

# Compute reference and errors for analysis
qd = q_d(t)
dqd = dq_d(t)
e = q - qd
de = dq - dqd
s = de + lam * e

# Plot results
plt.figure()
plt.plot(t, qd, label="q_d(t)")
plt.plot(t, q, "--", label="q(t)")
plt.xlabel("t [s]")
plt.ylabel("position [rad]")
plt.legend()
plt.title("Robust tracking with disturbance observer")

plt.figure()
plt.plot(t, e, label="e(t)")
plt.plot(t, s, label="s(t)")
plt.xlabel("t [s]")
plt.ylabel("error")
plt.legend()
plt.title("Tracking and sliding variables")

plt.figure()
plt.plot(t, d_hat, label="d_hat(t)")
plt.xlabel("t [s]")
plt.ylabel("disturbance estimate")
plt.legend()
plt.title("Disturbance estimate")

plt.show()
