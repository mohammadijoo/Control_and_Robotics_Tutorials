import numpy as np
import control as ct
import matplotlib.pyplot as plt

# DC motor / joint parameters (simple illustrative values)
J = 0.01   # kg*m^2
b = 0.1    # N*m*s/rad
Kt = 0.01  # N*m/A
Kp = 5.0   # proportional controller gain

# Transfer functions: G(s) = Kt / (J s^2 + b s), C(s) = Kp
s = ct.TransferFunction.s
G = Kt / (J * s**2 + b * s)
C = Kp
L = C * G  # open-loop transfer function

# Frequency grid (rad/s)
w = np.logspace(-1, 3, 400)

# Frequency response
mag, phase, omega = ct.freqresp(L, w)  # mag, phase are complex arrays

# Convert to Nichols coordinates
mag = np.abs(mag.squeeze())
phase_rad = np.angle(phase.squeeze())  # some versions return separate phase
mag_db = 20.0 * np.log10(mag)
phase_deg = 180.0 / np.pi * phase_rad

# Optional phase unwrapping for visual continuity
phase_rad_unwrapped = np.unwrap(phase_rad)
phase_deg_unwrapped = 180.0 / np.pi * phase_rad_unwrapped

# Plot Nichols curve
plt.figure()
plt.plot(phase_deg_unwrapped, mag_db)
plt.xlabel("Phase (deg)")
plt.ylabel("Magnitude (dB)")
plt.title("Nichols Plot of Robotic Joint Open Loop")
plt.grid(True)
plt.gca().invert_xaxis()  # conventional: phase decreases to the right
plt.show()
