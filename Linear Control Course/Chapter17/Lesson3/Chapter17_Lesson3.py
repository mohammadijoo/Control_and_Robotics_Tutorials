import numpy as np
import control as ctl  # python-control: classical control toolbox

# Example: G(s) = 1/s, C(s) = K
K = 10.0
s = ctl.TransferFunction.s
G = 1 / s
C = K
L = C * G

# Compute classical margins
gm, pm, wg, wp = ctl.margin(L)

# Delay margin from phase margin and gain crossover frequency
Td_from_pm = np.deg2rad(pm) / wg

print("Phase margin (deg):", pm)
print("Gain crossover (rad/s):", wg)
print("Approximate delay margin (s):", Td_from_pm)
