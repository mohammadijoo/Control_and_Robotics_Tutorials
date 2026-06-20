import numpy as np
import matplotlib.pyplot as plt

# Robot joint parameters (example)
J = 0.01   # inertia
B = 0.1    # viscous damping
K = 1.0    # motor/gear gain
Kp = 5.0   # proportional controller gain

def L_of_s(s):
    return Kp * K / (J * s**2 + B * s)

# Frequency grid (rad/s)
w = np.logspace(-2, 3, 400)  # 0.01 to 1000 rad/s

# Evaluate L(j w)
Ljw = L_of_s(1j * w)

# Build Nyquist points (positive and negative frequencies)
L_pos = Ljw
L_neg = np.conj(Ljw[::-1])

nyquist_points = np.concatenate([L_pos, L_neg])

plt.figure()
plt.plot(nyquist_points.real, nyquist_points.imag)
plt.axhline(0.0, linestyle="--")
plt.axvline(-1.0, linestyle="--")  # reference for stability analysis
plt.xlabel("Re{L(j*omega)}")
plt.ylabel("Im{L(j*omega)}")
plt.title("Nyquist Curve for Robot Joint with P Control")
plt.grid(True)
plt.axis("equal")
plt.show()
