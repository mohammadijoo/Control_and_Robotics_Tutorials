import numpy as np

def planar_2r_dynamics(q, qdot, params):
    """
    Compute M(q), C(q, qdot) qdot, g(q) for a planar 2R arm.

    q      : array-like length 2 [q1, q2]
    qdot   : array-like length 2 [dq1, dq2]
    params : dict with keys
             m1, m2, l1, lc1, lc2, I1, I2, g0
    """
    q1, q2 = q
    dq1, dq2 = qdot
    m1 = params["m1"]
    m2 = params["m2"]
    l1 = params["l1"]
    lc1 = params["lc1"]
    lc2 = params["lc2"]
    I1 = params["I1"]
    I2 = params["I2"]
    g0 = params["g0"]

    c2 = np.cos(q2)
    s2 = np.sin(q2)

    # Inertia matrix M(q)
    M11 = I1 + I2 + m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * c2)
    M12 = I2 + m2 * (lc2**2 + l1 * lc2 * c2)
    M22 = I2 + m2 * lc2**2
    M = np.array([[M11, M12],
                  [M12, M22]])

    # Coriolis/centrifugal term C(q, qdot) qdot (vector)
    h1 = -m2 * l1 * lc2 * s2 * (2 * dq1 * dq2 + dq2**2)
    h2 =  m2 * l1 * lc2 * s2 * dq1**2
    Cqdot = np.array([h1, h2])

    # Gravity vector g(q)
    g1 = (m1 * lc1 + m2 * l1) * g0 * np.cos(q1) + m2 * lc2 * g0 * np.cos(q1 + q2)
    g2 = m2 * lc2 * g0 * np.cos(q1 + q2)
    g = np.array([g1, g2])

    return M, Cqdot, g

# Example usage:
if __name__ == "__main__":
    params = dict(m1=1.0, m2=1.0, l1=1.0, lc1=0.5, lc2=0.5,
                  I1=0.1, I2=0.1, g0=9.81)
    q = np.array([0.5, 0.3])
    qdot = np.array([0.1, -0.2])
    M, Cqdot, g = planar_2r_dynamics(q, qdot, params)
    print("M(q) =\n", M)
    print("C(q, qdot) qdot =", Cqdot)
    print("g(q) =", g)
      
