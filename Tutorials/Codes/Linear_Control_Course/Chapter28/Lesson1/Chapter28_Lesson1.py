import numpy as np
import matplotlib.pyplot as plt
import control as ctrl  # pip install control

# Second-order plant parameters
zeta = 0.5
wn   = 4.0

num = [wn**2]
den = [1.0, 2.0*zeta*wn, wn**2]

G = ctrl.TransferFunction(num, den)

# Time response: step
t, y = ctrl.step_response(G)

plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("y(t)")
plt.title("Step response of second-order plant")
plt.grid(True)

# Frequency response using python-control
w = np.logspace(-1, 2, 400)
mag, phase, omega = ctrl.bode(G, w, Plot=False)

plt.figure()
plt.subplot(2, 1, 1)
plt.semilogx(omega, 20*np.log10(mag))
plt.ylabel("Magnitude [dB]")
plt.grid(True)

plt.subplot(2, 1, 2)
plt.semilogx(omega, phase * 180.0/np.pi)
plt.xlabel("Frequency [rad/s]")
plt.ylabel("Phase [deg]")
plt.grid(True)
plt.tight_layout()

# From-scratch frequency response evaluation
def freq_response(num, den, omega):
    """
    Evaluate G(j*omega) for a SISO transfer function defined by
    numerator and denominator coefficient lists.
    """
    s = 1j * omega
    num_val = np.polyval(num, s)
    den_val = np.polyval(den, s)
    return num_val / den_val

omega_test = np.logspace(-1, 2, 5)
G_vals = freq_response(num, den, omega_test)
print("Sampled G(j*omega):", G_vals)

plt.show()
