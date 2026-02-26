import numpy as np
import matplotlib.pyplot as plt

# Optional: python-control library, very common in robotics/control workflows
try:
    import control
except ImportError:
    control = None

# Open-loop plant: G(s) = 1 / (s (s + 2))
# Numerator and denominator of G(s)
num = np.array([1.0])             # N(s) = 1
den = np.array([1.0, 2.0, 0.0])   # D(s) = s^2 + 2 s + 0

# Pad numerator to same degree as denominator
deg = max(len(den), len(num))
den_padded = np.pad(den, (deg - len(den), 0), mode="constant")
num_padded = np.pad(num, (deg - len(num), 0), mode="constant")

K_values = np.linspace(0.0, 50.0, 200)
poles_real = []
poles_imag = []

for K in K_values:
    # Characteristic polynomial Delta(s;K) = D(s) + K N(s)
    char_poly = den_padded + K * num_padded
    roots = np.roots(char_poly)

    for r in roots:
        poles_real.append(np.real(r))
        poles_imag.append(np.imag(r))

plt.figure()
plt.scatter(poles_real, poles_imag, s=10)
plt.axhline(0.0, linewidth=0.5)
plt.axvline(0.0, linewidth=0.5)
plt.xlabel("Real(s)")
plt.ylabel("Imag(s)")
plt.title("Numerical root locus: G(s) = 1 / (s (s + 2))")

# If python-control is available, overlay its root locus
if control is not None:
    G = control.TransferFunction(num, den)
    control.root_locus(G, kvect=K_values, plot=True)

plt.show()
