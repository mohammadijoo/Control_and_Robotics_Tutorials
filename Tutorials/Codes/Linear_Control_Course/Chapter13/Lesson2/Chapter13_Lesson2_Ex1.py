import numpy as np
import control as ct  # python-control

# Joint model parameters (example)
J = 0.01   # inertia
B = 0.1    # damping
K = 0.5    # torque-to-position static gain (simplified)

# G(s) = K / (J s + B)
num = [K]
den = [J, B]
G = ct.TransferFunction(num, den)

# Frequency grid
omega = np.logspace(-1, 3, 300)  # rad/s

# Frequency response
# freqresp returns a complex array Gjw of shape (outputs, inputs, len(omega))
Gjw = ct.freqresp(G, omega)[0][0, 0, :]  # SISO: extract scalar response

mag = np.abs(Gjw)
phase = np.angle(Gjw)

# Example: print a few values
for w, m, p in zip(omega[::60], mag[::60], phase[::60]):
    print(f"w = {w:7.3f}, |G(jw)| = {m:7.3f}, phase = {p:7.3f} rad")
