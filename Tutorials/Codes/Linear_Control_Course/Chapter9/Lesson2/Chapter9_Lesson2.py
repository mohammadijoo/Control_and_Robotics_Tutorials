import numpy as np
import matplotlib.pyplot as plt

# Optional: use python-control if available (commonly used in robotics control stacks)
try:
    import control  # python-control
    USE_CONTROL_LIB = True
except ImportError:
    USE_CONTROL_LIB = False

# Open-loop transfer function G(s) = K (s+1) / [s (s+2) (s+4)]
num = np.array([1.0, 1.0])        # s + 1
den = np.polymul([1.0, 0.0],      # s
                 np.polymul([1.0, 2.0], [1.0, 4.0]))  # (s+2)(s+4)

if USE_CONTROL_LIB:
    # Using python-control
    G = control.tf(num, den)
    plt.figure()
    control.rlocus(G, kvect=np.linspace(0, 200, 400))
    plt.title("Root locus of G(s) = K (s+1) / [s (s+2) (s+4)]")
    plt.xlabel("Real axis")
    plt.ylabel("Imag axis")
    plt.grid(True)
    plt.show()
else:
    # From-scratch root locus computation (brute-force)
    kvect = np.linspace(0.0, 200.0, 400)
    poles_real = []
    poles_imag = []

    for K in kvect:
        # Characteristic polynomial: D(s) + K N(s) = 0
        # Here D(s) = den(s), N(s) = num(s)
        # So: den(s) + K * num(s) = 0
        # We form polynomial coefficients:
        # deg(den) = 3, deg(num) = 1, so we pad appropriately.
        den_pad = np.array(den, dtype=float)
        num_pad = np.pad(num, (len(den_pad) - len(num), 0))
        char_poly = den_pad + K * num_pad

        roots = np.roots(char_poly)
        poles_real.extend(np.real(roots))
        poles_imag.extend(np.imag(roots))

    plt.figure()
    plt.scatter(poles_real, poles_imag, s=5)
    plt.axhline(0.0, linewidth=1)
    plt.axvline(0.0, linewidth=1)
    plt.title("Root locus (brute-force) of G(s) = K (s+1) / [s (s+2) (s+4)]")
    plt.xlabel("Real axis")
    plt.ylabel("Imag axis")
    plt.grid(True)
    plt.show()
