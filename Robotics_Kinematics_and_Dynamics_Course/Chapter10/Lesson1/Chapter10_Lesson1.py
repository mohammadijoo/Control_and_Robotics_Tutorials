import numpy as np
from dataclasses import dataclass

@dataclass
class TwoLinkParams:
    m1: float
    m2: float
    l1: float
    l2: float
    c1: float
    c2: float
    I1: float
    I2: float
    g: float = 9.81  # gravitational acceleration

def inertia_matrix(q, p: TwoLinkParams):
    """
    Joint-space inertia matrix M(q) for a planar 2R arm.
    q: array-like of shape (2,)
    """
    q1, q2 = q
    cos2 = np.cos(q2)

    M11 = (p.I1 + p.I2
           + p.m1 * p.c1**2
           + p.m2 * (p.l1**2 + p.c2**2 + 2.0 * p.l1 * p.c2 * cos2))
    M12 = p.I2 + p.m2 * (p.c2**2 + p.l1 * p.c2 * cos2)
    M22 = p.I2 + p.m2 * p.c2**2

    M = np.array([[M11, M12],
                  [M12, M22]], dtype=float)
    return M

def potential_energy(q, p: TwoLinkParams):
    """
    Gravitational potential V(q) for a planar 2R arm in a vertical plane.
    """
    q1, q2 = q
    V = (p.m1 * p.g * p.c1 * np.sin(q1)
         + p.m2 * p.g * (p.l1 * np.sin(q1) + p.c2 * np.sin(q1 + q2)))
    return V

def kinetic_energy(q, qd, p: TwoLinkParams):
    """
    Kinetic energy T(q, qdot) = 0.5 * qdot^T M(q) qdot
    """
    q = np.asarray(q, dtype=float).reshape(2)
    qd = np.asarray(qd, dtype=float).reshape(2)
    M = inertia_matrix(q, p)
    return 0.5 * float(qd.T @ M @ qd)

def total_energy(q, qd, p: TwoLinkParams):
    return kinetic_energy(q, qd, p) + potential_energy(q, p)

if __name__ == "__main__":
    # Example parameters (roughly human-arm sized)
    params = TwoLinkParams(
        m1=2.0, m2=1.5,
        l1=0.4, l2=0.3,
        c1=0.2, c2=0.15,
        I1=0.02, I2=0.01
    )
    q = np.array([0.5, -0.3])
    qd = np.array([0.4, 0.2])

    T = kinetic_energy(q, qd, params)
    V = potential_energy(q, params)
    E = T + V
    print("T =", T, "V =", V, "E =", E)
      
