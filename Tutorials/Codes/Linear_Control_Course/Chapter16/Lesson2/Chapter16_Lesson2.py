import numpy as np
import control as ct  # python-control toolbox

# Plant: G(s) = K / (s (0.1 s + 1))
K = 5.0
num = [K]
den = [0.1, 1.0, 0.0]
G = ct.tf(num, den)

# Unity feedback closed-loop transfer T(s) = G(s)/(1+G(s))
T = ct.feedback(G, 1)

# Frequency grid
w = np.logspace(-1, 2, 400)  # rad/s

# Open-loop frequency response L(jw)
L = ct.evalfr(G, 1j*w[0])  # just to check type
Lw = np.array([ct.evalfr(G, 1j*wi) for wi in w])

# Nichols coordinates for open-loop
phi = np.angle(Lw)          # rad
Lm = np.abs(Lw)
L_db = 20.0 * np.log10(Lm)

# Closed-loop via Hall-circle formula (from open-loop L)
Tw = Lw / (1.0 + Lw)
M = np.abs(Tw)
psi = np.angle(Tw)

# For comparison, closed-loop directly from T(s)
Tw_direct = np.array([ct.evalfr(T, 1j*wi) for wi in w])
M_direct = np.abs(Tw_direct)
psi_direct = np.angle(Tw_direct)

# Check maximum closed-loop peak Mr and its frequency
Mr = M.max()
idx_mr = np.argmax(M)
w_mr = w[idx_mr]
Mr_db = 20.0 * np.log10(Mr)

print(f"Closed-loop resonant peak Mr = {Mr:.3f} ({Mr_db:.2f} dB) at w = {w_mr:.3f} rad/s")

# Example: function mapping Nichols point to closed-loop magnitude/phase
def closed_loop_from_nichols(L_db_point, phi_point):
    L_lin = 10.0**(L_db_point / 20.0)
    # Form complex open-loop point on ray with phase phi_point
    L_point = L_lin * np.exp(1j * phi_point)
    T_point = L_point / (1.0 + L_point)
    return np.abs(T_point), np.angle(T_point)

# This function would be used if you had only Nichols coordinates, e.g. from log files.
