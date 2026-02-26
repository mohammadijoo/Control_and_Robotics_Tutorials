import numpy as np
import matplotlib.pyplot as plt
import control as ct  # python-control library

# Plant and controller parameters
Kp = 2.0           # plant DC gain
tau_p = 0.1        # plant time constant
Kc = 5.0           # proportional controller gain

# Define plant P(s) = Kp / (tau_p s + 1)
P = ct.TransferFunction([Kp], [tau_p, 1.0])

# Define controller C(s) = Kc
C = ct.TransferFunction([Kc], [1.0])

# Loop transfer L(s) and complementary sensitivity T(s)
L = C * P
T = ct.feedback(L, 1)  # feedback(L,1) = L / (1 + L)

# Bode magnitude of T(jw) to study noise shaping
w = np.logspace(0, 4, 400)  # 1 rad/s to 10^4 rad/s
mag_T, phase_T, omega = ct.bode_plot(T, w, Plot=False)

plt.figure()
plt.semilogx(omega, 20 * np.log10(mag_T))
plt.xlabel("Frequency (rad/s)")
plt.ylabel("|T(jw)| (dB)")
plt.title("Complementary Sensitivity Magnitude")
plt.grid(True)

# Approximate output noise variance for band-limited white noise
sigma_n2 = 1.0  # noise PSD level
Phi_n = sigma_n2 * np.ones_like(omega)
# Using a simple Riemann sum approximation:
sigma_y2 = (1.0 / (2.0 * np.pi)) * np.trapz((mag_T ** 2) * Phi_n, omega)
print("Approximate output noise variance due to sensor noise:", sigma_y2)

plt.show()
