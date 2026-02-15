import numpy as np
import matplotlib.pyplot as plt

# Control toolbox for LTI systems
import control  # pip install control

# Robot joint parameters
J = 0.01   # kg*m^2 (effective inertia)
b = 0.1    # N*m*s/rad (viscous friction)

# Plant: torque -> angle
# P(s) = 1 / (J s^2 + b s) = 1 / (s (J s + b))
P = control.tf([1.0], [J, b, 0.0])

# PI controller
Kp = 5.0
Ki = 10.0
C = control.tf([Kp, Ki], [1.0, 0.0])  # Kp + Ki/s

# Loop transfer function L(s) = C(s) P(s)
L = control.series(C, P)

# Disturbance-to-output for plant-input disturbance:
# G_d(s) = P(s) / (1 + L(s))
Gd = control.minreal(P / (1 + L))

# Frequency grid
w = np.logspace(-1, 2, 400)  # rad/s

# Bode magnitude of Gd(jw)
mag, phase, wout = control.bode(Gd, w, Plot=False)
mag_db = 20 * np.log10(mag)

plt.figure()
plt.semilogx(wout, mag_db)
plt.xlabel("Frequency [rad/s]")
plt.ylabel("Magnitude of Gd(jw) [dB]")
plt.title("Disturbance-to-output magnitude for plant-input disturbance")
plt.grid(True, which="both")
plt.show()
