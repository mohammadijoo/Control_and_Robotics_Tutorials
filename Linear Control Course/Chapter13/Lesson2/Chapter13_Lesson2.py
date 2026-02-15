import numpy as np

# First-order low-pass: G(s) = K / (1 + T s)
K = 2.0
T = 0.1

def G_of_s(s: complex) -> complex:
    """Evaluate G(s) = K / (1 + T s) at complex s."""
    return K / (1.0 + T * s)

# Frequency grid (rad/s)
omega = np.logspace(-1, 3, 200)  # 0.1 to 1000 rad/s
s = 1j * omega

Gjw = np.array([G_of_s(sk) for sk in s])
mag = np.abs(Gjw)
phase = np.angle(Gjw)  # radians

for w, m, p in zip(omega[::50], mag[::50], phase[::50]):
    print(f"w = {w:7.3f} rad/s, |G(jw)| = {m:7.3f}, phase = {p:7.3f} rad")
