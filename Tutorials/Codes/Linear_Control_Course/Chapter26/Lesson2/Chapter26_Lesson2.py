import numpy as np
import control as ctrl  # python-control
import matplotlib.pyplot as plt

# Design parameters
tau = 0.05          # s, time constant
wc = 1.0 / tau      # rad/s
w0 = 50.0           # rad/s, center frequency
zeta = 0.05         # damping for band-pass
Q_notch = 10.0      # quality factor for notch

# Low-pass and high-pass (first-order)
LP = ctrl.TransferFunction([wc], [1.0, wc])        # wc / (s + wc)
HP = ctrl.TransferFunction([1.0, 0.0], [1.0, wc])  # s / (s + wc)

# Band-pass (second-order): 2 zeta w0 s / (s^2 + 2 zeta w0 s + w0^2)
BP = ctrl.TransferFunction([2.0 * zeta * w0, 0.0],
                           [1.0, 2.0 * zeta * w0, w0 ** 2])

# Notch: (s^2 + w0^2) / (s^2 + (w0/Q) s + w0^2)
Notch = ctrl.TransferFunction([1.0, 0.0, w0 ** 2],
                              [1.0, w0 / Q_notch, w0 ** 2])

w = np.logspace(-1, 3, 600)  # frequency grid

plt.figure()
for sys, label in [(LP, "LP"), (HP, "HP"), (BP, "BP"), (Notch, "Notch")]:
    mag, phase, w_out = ctrl.bode(sys, w, Plot=False)
    plt.loglog(w_out, mag, label=label)

plt.xlabel("omega [rad/s]")
plt.ylabel("|H(j omega)|")
plt.grid(True, which="both")
plt.legend()
plt.title("Magnitude responses of LP/HP/BP/Notch filters")
plt.show()
