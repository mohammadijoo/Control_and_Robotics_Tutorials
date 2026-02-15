import numpy as np

def cantilever_tip_deflection(F, L, E, I):
    # delta = F L^3 / (3 E I)
    return F * L**3 / (3.0 * E * I)

def continuum_tip_position(kappa, phi, L):
    # constant-curvature section
    if abs(kappa) < 1e-9:
        return np.array([0.0, 0.0, L])
    x = (1.0/kappa) * (1 - np.cos(kappa*L)) * np.cos(phi)
    y = (1.0/kappa) * (1 - np.cos(kappa*L)) * np.sin(phi)
    z = (1.0/kappa) * np.sin(kappa*L)
    return np.array([x, y, z])

F, L, E, I = 10.0, 0.5, 70e9, 2.0e-10  # SI units
print("Flexible-link tip deflection (m):", cantilever_tip_deflection(F, L, E, I))

kappa, phi = 4.0, np.pi/6
print("Continuum tip position (m):", continuum_tip_position(kappa, phi, L))
      