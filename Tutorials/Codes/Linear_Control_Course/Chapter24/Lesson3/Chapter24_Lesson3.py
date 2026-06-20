import numpy as np
import control
from control.matlab import tf, bode, nichols

# Optional robotics models (not strictly needed for this example)
# from roboticstoolbox import models as rtb_models

# Robot joint approximated as P(s) = K / (J s^2 + B s)
J = 0.02   # kg m^2 (effective inertia)
B = 0.1    # N m s/rad (viscous friction)
K = 1.0    # N m / V (torque constant * amplifier gain)

s = control.TransferFunction.s
P = K / (J * s**2 + B * s)

# PI + lead controller
kp = 50.0
Ti = 0.1
Tl = 0.02
alpha = 0.2

C = kp * (1 + 1 / (Ti * s)) * ((Tl * s + 1) / (alpha * Tl * s + 1))

L = C * P
S = 1 / (1 + L)
T = L / (1 + L)

# Frequency grid
w = np.logspace(-1, 3, 600)

# Compute sensitivity peak M_S
magS, phaseS, wS = control.bode(S, w, Plot=False)
MS = np.max(magS)
print(f"Maximum sensitivity M_S ≈ {MS:.2f}")

# Simple multiplicative output uncertainty weight
# Example: about 20% error above 50 rad/s
W2 = 0.2 * (s / 50 + 1) / (s / 500 + 1)

magWT, phaseWT, wWT = control.bode(W2 * T, w, Plot=False)
robust_index = np.max(magWT)
print(f"max |W2(jw) T(jw)| ≈ {robust_index:.2f}")

if robust_index < 1.0:
    print("Small-gain condition satisfied: robustly stable for the modeled output uncertainty.")
else:
    print("Robust stability not guaranteed for the chosen W2.")

# Plot Bode and Nichols for inspection
control.bode(L, w)
control.nichols(L, w)
