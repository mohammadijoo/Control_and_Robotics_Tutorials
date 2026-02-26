import numpy as np
import control  # python-control library (classical control tools)
from control import matlab

# Approximate robotic joint: J = 0.01 kg m^2, B = 0.1 N m s/rad
J = 0.01
B = 0.1

# G(s) = 1 / (J s^2 + B s) = 100 / (s (s + 10))
numG = [100.0]
denG = [1.0, 10.0, 0.0]
G = matlab.tf(numG, denG)

# PD controller C(s) = Kd s + Kp
Kp = 20.0
Kd = 1.0
C = matlab.tf([Kd, Kp], [1.0])

# Loop transfer, sensitivity, complementary sensitivity
L = C * G
S = control.feedback(control.tf([1.0], [1.0]), L)  # 1 / (1 + L)
T = control.feedback(L, control.tf([1.0], [1.0]))  # L / (1 + L)

# Frequency grid for analysis
omega = np.logspace(-1, 3, 400)  # 0.1 to 100 rad/s

# Frequency response of S(jw)
magS, phaseS, omega_out = control.freqresp(S, omega)
magS = magS.flatten()  # python-control returns 3D arrays

Ms = float(np.max(magS))
Ms_dB = 20.0 * np.log10(Ms)
print("Peak sensitivity Ms = {:.3f} ({:.2f} dB)".format(Ms, Ms_dB))

# Nichols plot of L(jw)
control.nichols_plot(L, omega)
# In an interactive environment, you can overlay constant-S contours
# using control.nichols_grid() or by precomputing the curves.
