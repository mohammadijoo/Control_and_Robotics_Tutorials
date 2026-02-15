import numpy as np
import matplotlib.pyplot as plt

# Control/robotics libraries
# pip install control roboticstoolbox-python
import control as ctl

# Simple robot joint model: J * q_ddot + b * q_dot = u
J = 0.01   # kg m^2
b = 0.05   # N m s/rad

# Plant P(s) = 1/(J s^2 + b s)
s = ctl.TransferFunction.s
P = 1 / (J * s**2 + b * s)

# Proportional-derivative-like controller (here just proportional for simplicity)
k = 5.0
C = k

L = C * P                  # Loop transfer
S = ctl.feedback(1, L)     # S(s) = 1 / (1 + L(s))
T = ctl.feedback(L, 1)     # T(s) = L(s) / (1 + L(s))

# Bode magnitude of S and T, and numeric approximation of the Bode integral
w = np.logspace(-1, 3, 1000)   # rad/s
magS, phaseS, wS = ctl.bode(S, w, Plot=False)
magT, phaseT, wT = ctl.bode(T, w, Plot=False)

# Approximate Bode sensitivity integral int_0^inf ln|S(jw)| dw via trapezoidal rule
lnS = np.log(magS)
integral_lnS = np.trapz(lnS, w)

print("Approximate Bode integral of ln|S(jw)| =", integral_lnS)

# Plot in dB to visualize the waterbed effect
plt.figure()
plt.semilogx(w, 20 * np.log10(magS), label="S(jw)")
plt.semilogx(w, 20 * np.log10(magT), label="T(jw)", linestyle="--")
plt.axhline(0.0, linestyle=":", linewidth=0.8)
plt.xlabel("Frequency (rad/s)")
plt.ylabel("Magnitude (dB)")
plt.title("Sensitivity and Complementary Sensitivity for Robot Joint Loop")
plt.legend()
plt.grid(True, which="both")
plt.show()
