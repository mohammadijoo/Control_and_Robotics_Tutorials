import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Plant parameters (e.g. joint dynamics)
M, B, K = 1.0, 2.0, 20.0
G = ct.TransferFunction([1.0], [M, B, K])

# Baseline PD controller
Kp, Kd = 40.0, 8.0
C0 = ct.TransferFunction([Kd, Kp], [1.0, 0.0])  # Kd s + Kp

# First-order low-pass filter F(s) = wf / (s + wf)
wf = 50.0
F = ct.TransferFunction([wf], [1.0, wf])

# Case 1: measurement-path filter F_m(s)
# Loop: Lm(s) = G(s) * C0(s) * F(s) in the feedback branch
# We explicitly build the closed-loop with noise input
Lm = G * C0 * F
Tm_ref_meas = ct.feedback(G * C0, F)   # y/r with F in feedback
Tm_ref_ctrl = ct.feedback(G * C0 * F)  # y/r with F in controller path

# Frequency response comparison
w = np.logspace(-1, 3, 400)
mag_meas, phase_meas, _ = ct.bode(Tm_ref_meas, w, Plot=False)
mag_ctrl, phase_ctrl, _ = ct.bode(Tm_ref_ctrl, w, Plot=False)

plt.figure()
plt.loglog(w, mag_meas, label="y/r with measurement-path F")
plt.loglog(w, mag_ctrl, label="y/r with controller-path F", linestyle="--")
plt.xlabel("Frequency [rad/s]")
plt.ylabel("Magnitude")
plt.legend()
plt.grid(True, which="both")
plt.show()
