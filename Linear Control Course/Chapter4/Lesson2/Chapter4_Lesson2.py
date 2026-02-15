import numpy as np
import control  # python-control library
from math import sqrt

# Joint model parameters (example values)
J = 0.01   # kg m^2
B = 0.02   # N m s/rad
K = 1.0    # N m/rad
Kt = 0.5   # N m per unit input

# Transfer function G(s) = Theta(s)/U(s)
num = [Kt]
den = [J, B, K]
G = control.TransferFunction(num, den)

print("Transfer function G(s) = Theta(s)/U(s):")
print(G)

# Poles and zeros
poles = control.pole(G)
zeros = control.zero(G)

print("Poles:", poles)
print("Zeros:", zeros)

# Step response (e.g., position response to a step in torque command)
t, y = control.step_response(G)
# t, y can be plotted with matplotlib
