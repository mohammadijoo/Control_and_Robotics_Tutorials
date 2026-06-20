import numpy as np
import matplotlib.pyplot as plt

# python-control for LTI systems (pip install control)
import control as ctl

# ----- First-order pole and zero -----
omega_c = 10.0  # rad/s corner frequency for pole
omega_z = 5.0   # rad/s corner frequency for zero

G_p = ctl.TransferFunction([omega_c], [1.0, omega_c])  # 1 / (1 + s/omega_c)
G_z = ctl.TransferFunction([1.0, omega_z], [omega_z])  # 1 + s/omega_z

# ----- Second-order plant (e.g., DC motor speed loop) -----
omega_n = 20.0
zeta = 0.3
G_2 = ctl.TransferFunction([omega_n**2],
                           [1.0, 2*zeta*omega_n, omega_n**2])

# Total plant as product of first- and second-order factors
G_total = G_p * G_2

# Frequency range (logarithmic)
omega = np.logspace(-1, 3, 500)

# Bode plots
plt.figure()
mag_p, phase_p, w = ctl.bode(G_p, omega, Plot=False)
plt.subplot(2, 1, 1)
plt.semilogx(w, 20*np.log10(mag_p))
plt.ylabel("Magnitude (dB)")
plt.title("First-order pole 1 / (1 + s/omega_c)")

plt.subplot(2, 1, 2)
plt.semilogx(w, np.degrees(phase_p))
plt.ylabel("Phase (deg)")
plt.xlabel("Frequency (rad/s)")
plt.tight_layout()

plt.figure()
mag_2, phase_2, w2 = ctl.bode(G_2, omega, Plot=False)
plt.subplot(2, 1, 1)
plt.semilogx(w2, 20*np.log10(mag_2))
plt.ylabel("Magnitude (dB)")
plt.title("Second-order factor omega_n^2 / (s^2 + 2 zeta omega_n s + omega_n^2)")

plt.subplot(2, 1, 2)
plt.semilogx(w2, np.degrees(phase_2))
plt.ylabel("Phase (deg)")
plt.xlabel("Frequency (rad/s)")
plt.tight_layout()

plt.figure()
mag_tot, phase_tot, w3 = ctl.bode(G_total, omega, Plot=False)
plt.subplot(2, 1, 1)
plt.semilogx(w3, 20*np.log10(mag_tot))
plt.ylabel("Magnitude (dB)")
plt.title("Combined first- and second-order plant")

plt.subplot(2, 1, 2)
plt.semilogx(w3, np.degrees(phase_tot))
plt.ylabel("Phase (deg)")
plt.xlabel("Frequency (rad/s)")
plt.tight_layout()
plt.show()
