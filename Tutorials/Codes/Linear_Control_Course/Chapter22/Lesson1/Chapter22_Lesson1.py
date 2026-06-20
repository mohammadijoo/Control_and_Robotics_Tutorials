import numpy as np
import matplotlib.pyplot as plt
import control as ctrl  # python-control library

# Simple DC-motor-like joint model: P(s) = K / (J s + B)
J = 0.01    # kg m^2
B = 0.1     # N m s/rad
K = 1.0     # gain (N m per unit control)
P = ctrl.tf([K], [J, B])

# PI controller: C(s) = Kp + Ki/s
Kp = 5.0
Ki = 10.0
C = ctrl.tf([Kp, Ki], [1, 0])  # (Kp s + Ki) / s

L = C * P               # loop transfer
S = ctrl.feedback(1, L) # S(s) = 1 / (1 + L(s))
Gd = P * S              # disturbance -> output at plant input

# Frequency grid
w = np.logspace(-1, 3, 500)

magS, phaseS, wS = ctrl.bode(S, w, Plot=False)
magGd, phaseGd, wGd = ctrl.bode(Gd, w, Plot=False)

plt.figure()
plt.semilogx(wS, 20 * np.log10(magS))
plt.xlabel("Frequency [rad/s]")
plt.ylabel("|S(jw)| [dB]")

plt.figure()
plt.semilogx(wGd, 20 * np.log10(magGd))
plt.xlabel("Frequency [rad/s]")
plt.ylabel("|P(jw) S(jw)| [dB]  (disturbance -> output)")
plt.show()

# Time-domain disturbance step response (R(s) = 0, Du(s) = 1/s)
t = np.linspace(0, 5, 1000)
t_out, y_out = ctrl.step_response(Gd, T=t)
plt.figure()
plt.plot(t_out, y_out)
plt.xlabel("Time [s]")
plt.ylabel("Output y(t) due to unit disturbance")
plt.grid(True)
plt.show()
