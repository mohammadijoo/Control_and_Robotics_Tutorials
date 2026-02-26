import numpy as np
import matplotlib.pyplot as plt

# Core control library
import control as ctl

# Robot joint parameters (simplified single-axis)
J = 0.01   # kg m^2
B = 0.1    # N m s/rad

# PD gains (typical for a servo joint)
Kp = 50.0
Kd = 2.0

# Plant and controller transfer functions
s = ctl.TransferFunction.s
P = 1.0 / (J * s**2 + B * s)
C = Kp + Kd * s

L = C * P          # loop transfer
S = 1 / (1 + L)    # sensitivity
T = L / (1 + L)    # complementary sensitivity

# Bode magnitude plots of S and T
omega = np.logspace(-1, 3, 400)
magS, phaseS, wS = ctl.bode(S, omega, Plot=False)
magT, phaseT, wT = ctl.bode(T, omega, Plot=False)

plt.figure()
plt.loglog(wS, magS)
plt.loglog(wT, magT)
plt.xlabel("Frequency [rad/s]")
plt.ylabel("Magnitude")
plt.legend(["|S(jw)|", "|T(jw)|"])
plt.grid(True, which="both")
plt.title("Sensitivity and Complementary Sensitivity for a Robot Joint")
plt.show()
