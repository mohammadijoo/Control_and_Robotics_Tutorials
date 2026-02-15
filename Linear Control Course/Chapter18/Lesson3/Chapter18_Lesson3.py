import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control

# Plant: second-order joint dynamics, J*s^2 + b*s + k (simplified)
J = 0.01   # inertia
b = 0.1    # damping
k = 0.0    # stiffness (assume gravity-compensated, small k)

numG = [1.0]
denG = [J, b, k]
G = ct.TransferFunction(numG, denG)

# PD controller (in practice used in robot joint controllers)
Kp = 20.0
Kd = 0.5
C = Kd * ct.TransferFunction([1, 0], [1]) + Kp

L = C * G  # open-loop

# Measurement low-pass filter: F(s) = wf / (s + wf)
wf = 50.0  # rad/s, choose relative to closed-loop bandwidth
F = ct.TransferFunction([wf], [1.0, wf])

# Closed-loop from noise at measurement to output:
# without filter: Gn0(s) = -L / (1 + L)
Gn0 = -L / (1 + L)

# with filter: GnF(s) = -L*F / (1 + L*F)
GnF = -(L * F) / (1 + L * F)

omega = np.logspace(0, 3, 400)  # 1 to 1000 rad/s

mag0, phase0, w0 = ct.bode(Gn0, omega, Plot=False)
magF, phaseF, wF = ct.bode(GnF, omega, Plot=False)

plt.figure()
plt.loglog(w0, mag0, label="no filter")
plt.loglog(wF, magF, linestyle="--", label="with measurement LPF")
plt.xlabel("omega [rad/s]")
plt.ylabel("|Y/N|")
plt.grid(True, which="both")
plt.legend()
plt.title("Noise-to-output magnitude with and without low-pass filtering")
plt.show()
