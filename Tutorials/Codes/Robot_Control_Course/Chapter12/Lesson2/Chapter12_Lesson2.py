
import numpy as np
from numpy.linalg import eig
from math import log, sqrt

# Discrete-time closed-loop matrix Ad (example for one joint)
Ad = np.array([[0.95, 0.01],
               [-0.2,  0.90]])

Ts = 0.005  # sampling period [s]

# Eigenvalues (discrete poles)
z, _ = eig(Ad)

print("Discrete poles:", z)

# Check stability: all |z| < 1
stable = np.all(np.abs(z) < 1.0)
print("Schur-stable?", stable)

# Approximate dominant pole and settling time
r = max(np.abs(z))
if r < 1.0:
    # approximate decay time constant tau by matching r = exp(-Ts/tau)
    tau = -Ts / log(r)
    Ts_settle = 4.0 * tau   # 2 percent criterion
    print("Approx continuous settling time [s]:", Ts_settle)
    print("Approx samples to settle:", Ts_settle / Ts)

# Using python-control to construct a discrete system
import control as ct

sysd = ct.ss(Ad, np.zeros((2,1)), np.eye(2), np.zeros((2,1)), Ts)
print("System is stable?", ct.isdtime(sysd) and ct.pole(sysd))
