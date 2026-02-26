import numpy as np
import matplotlib.pyplot as plt

# python-control library (commonly used in robotics and mechatronics)
# pip install control
import control as ctrl

# Plant: G_m(s) = 1 / (s (s + 1))
s = ctrl.TransferFunction.s
Gm = 1 / (s * (s + 1))

# Lead compensator parameters from Sections 2-3
alpha = 1.0 / 3.0
T = np.sqrt(3.0) / 2.0
Kc = 2.58  # numerical approximation

Cle = Kc * (T * s + 1) / (alpha * T * s + 1)

# Loop and closed-loop transfer functions
L = Cle * Gm
Tcl = ctrl.feedback(L, 1)  # unity feedback

# Time-domain response (step in position reference)
t, y = ctrl.step_response(Tcl)

plt.figure()
plt.plot(t, y)
plt.xlabel("Time [s]")
plt.ylabel("Position response")
plt.title("Lead-compensated position servo (Python)")
plt.grid(True)

# Bode plot of loop transfer function
plt.figure()
mag, phase, omega = ctrl.bode(L, dB=True, Hz=False, omega_limits=(0.1, 100), omega_num=500)
plt.suptitle("Loop Bode plot with lead compensation")

plt.show()
