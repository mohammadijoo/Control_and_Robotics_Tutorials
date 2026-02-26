import numpy as np
import control as ct

# Example plant: simplified joint dynamics (type-1)
s = ct.TransferFunction.s
G = 1 / (s * (s + 2))

# Lag compensator parameters
Kc = 5.0
omega_p = 0.2   # rad/s
omega_z = 1.0   # rad/s, omega_z > omega_p
Clag = Kc * (s/omega_z + 1) / (s/omega_p + 1)

# Open-loop and closed-loop
L = Clag * G
T = ct.feedback(L, 1)  # unity feedback

# Frequency response around crossover
omega = np.logspace(-2, 2, 400)
mag_L, phase_L, _ = ct.bode(L, omega, Plot=False)

# Steady-state constants (approximate)
# For type-1: Kv = lim_{s->0} s L(s) = slope of |L(jw)| at low w in log-log
Kv = ct.dcgain(ct.series(Clag, G * s))  # s*L(s) evaluated at s=0

print("Lag compensator:", Clag)
print("Approximate velocity constant Kv:", Kv)

# For robotics: linearized joint model via robotics toolbox (conceptual)
# import roboticstoolbox as rtb
# panda = rtb.models.DH.Panda()
# # Extract local linear model around a configuration and use similar lag design
