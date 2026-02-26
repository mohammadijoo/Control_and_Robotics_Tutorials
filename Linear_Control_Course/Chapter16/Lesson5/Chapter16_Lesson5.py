import numpy as np
import matplotlib.pyplot as plt
import control as ctrl  # python-control

# Plant: typical lightly damped robotic joint axis (simplified)
s = ctrl.TransferFunction.s
G = 10 / (s * (s + 1) * (s + 5))

# Proportional controller
K = 40.0  # choose a trial gain
C = ctrl.TransferFunction([K], [1])
L = C * G

# 1) Bode plot with stability margins
omega = np.logspace(-2, 2, 500)
mag, phase, w = ctrl.bode_plot(L, omega, dB=True, Hz=False, deg=True, Plot=False)
# python-control 0.10+ can return a ControlPlot; here we use the older API style
ctrl.bode_plot(L, omega, dB=True, Hz=False, deg=True, Plot=True)
gm, pm, wg, wp = ctrl.margin(L)
print("Gain margin (abs):", gm, "at w =", wg)
print("Phase margin (deg):", pm, "at w =", wp)

# 2) Nyquist plot
plt.figure()
ctrl.nyquist_plot(L)

# 3) Nichols plot with chart grid (closed-loop M- and N-contours)
plt.figure()
ctrl.nichols_plot(L, omega, grid=True)

plt.show()
