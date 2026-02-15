
import numpy as np
from numpy.linalg import eigvals
from math import isfinite

# Nominal parameters
J_nom = 1.0     # nominal inertia
b = 0.2         # viscous friction
k_p = 25.0
k_d = 10.0

def A_matrix(J, b, k_p, k_d):
    """
    State matrix for single-joint PD control:
        e_dot   = v
        v_dot   = -(k_p/J) * e - ((b + k_d)/J) * v
    """
    return np.array([[0.0,        1.0],
                     [-k_p / J,  -(b + k_d) / J]])

# Uncertainty range on inertia: J in [0.5, 1.5]
J_grid = np.linspace(0.5, 1.5, 21)
robust_stable = True

for J in J_grid:
    A = A_matrix(J, b, k_p, k_d)
    lam = eigvals(A)
    max_real = np.max(np.real(lam))
    if max_real >= 0.0 or not isfinite(max_real):
        print("Potential instability at J =", J, "eigs =", lam)
        robust_stable = False
        break

print("Grid-based robustness check (eigenvalues real parts < 0):", robust_stable)

# Simple time-domain simulation for several J values
def simulate_joint(J, b, k_p, k_d, e0=0.5, edot0=0.0, dt=1e-3, T=2.0):
    n_steps = int(T / dt)
    e = e0
    edot = edot0
    traj = []
    for k in range(n_steps):
        # PD torque for regulation to e = 0
        tau = -k_p * e - k_d * edot
        # Dynamics: J * eddot + b * edot = tau
        eddot = (tau - b * edot) / J
        # Euler integration (small dt)
        edot = edot + dt * eddot
        e = e + dt * edot
        traj.append((k * dt, e, edot))
    return np.array(traj)

if robust_stable:
    for J in [0.5, 1.0, 1.5]:
        traj = simulate_joint(J, b, k_p, k_d)
        print("Final error for J =", J, "is", traj[-1, 1])
