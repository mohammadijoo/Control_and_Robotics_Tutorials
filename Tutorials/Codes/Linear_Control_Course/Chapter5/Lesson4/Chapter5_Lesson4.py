import numpy as np
import matplotlib.pyplot as plt
from control import tf, step_response  # python-control library

def first_order_metrics(K, tau, tol=0.02):
    # delay time (50%), rise time (10-90%), settling time (tol)
    t_d = -tau * np.log(0.5)
    t_r = tau * np.log(9.0)  # 10-90%
    t_s = -tau * np.log(tol)
    return t_d, t_r, t_s

# Example: simple velocity loop of a robotic joint
K = 2.0
tau = 0.1  # seconds

# Define transfer function G(s) = K / (tau s + 1)
G = tf([K], [tau, 1.0])

# Step response
t = np.linspace(0, 1.0, 1000)
t_out, y_out = step_response(G, T=t)

t_d, t_r, t_s = first_order_metrics(K=K, tau=tau, tol=0.02)
print("Delay time t_d =", t_d)
print("Rise time t_r (10-90%) =", t_r)
print("Settling time t_s (2%) =", t_s)

plt.figure()
plt.plot(t_out, y_out, label="step response")
plt.axhline(K, linestyle="--", label="final value")
plt.axvline(t_d, linestyle=":", label="t_d")
plt.axvline(t_r, linestyle=":", label="t_r")
plt.axvline(t_s, linestyle=":", label="t_s (2%)")
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.legend()
plt.grid(True)
plt.show()
