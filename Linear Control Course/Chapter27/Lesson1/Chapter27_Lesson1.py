import numpy as np
import matplotlib.pyplot as plt
import control as ctl  # python-control library

# Plant and controller parameters
tau = 0.5
G = ctl.tf([1.0], [tau, 1.0])   # G(s) = 1 / (tau s + 1)
Kp = 4.0
Ki = 5.0
C = ctl.tf([Kp, Ki], [1.0, 0.0])  # Kp + Ki / s

L = C * G
S = 1 / (1 + L)
T = L / (1 + L)

# 1) Tracking: step in reference, no disturbance
t = np.linspace(0, 5, 500)
t1, y_track = ctl.step_response(T, T=t)  # y(t) for r(t) = 1

# 2) Regulation: step disturbance at plant output, r(t) = 0
# For output disturbance d(t), transfer from d to y is S(s).
t2, y_reg = ctl.step_response(S, T=t)

plt.figure()
plt.plot(t1, y_track, label="Tracking: y(t) for step r(t)")
plt.plot(t2, y_reg, label="Regulation: y(t) for step d(t)")
plt.axhline(1.0, linestyle="--", label="Reference level r(t) = 1")
plt.axhline(0.0, linestyle=":", label="Regulation target y(t) = 0")
plt.xlabel("Time [s]")
plt.ylabel("Output y(t)")
plt.legend()
plt.grid(True)
plt.title("Tracking vs Regulation on First-Order Plant")
plt.show()
