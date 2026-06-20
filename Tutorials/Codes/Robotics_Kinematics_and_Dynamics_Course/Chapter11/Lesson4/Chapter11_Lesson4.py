import numpy as np

class TwoLinkParams:
    def __init__(self, l1, l2, c1, c2, m1, m2, I1, I2, g=9.81):
        self.l1 = l1
        self.l2 = l2
        self.c1 = c1
        self.c2 = c2
        self.m1 = m1
        self.m2 = m2
        self.I1 = I1
        self.I2 = I2
        self.g = g

def M_2R(q, params):
    q1, q2 = q
    l1 = params.l1
    c1 = params.c1
    c2 = params.c2
    m1 = params.m1
    m2 = params.m2
    I1 = params.I1
    I2 = params.I2

    cos2 = np.cos(q2)

    M11 = I1 + I2 + m1 * c1**2 + m2 * (l1**2 + c2**2 + 2.0 * l1 * c2 * cos2)
    M12 = I2 + m2 * (c2**2 + l1 * c2 * cos2)
    M22 = I2 + m2 * c2**2

    return np.array([[M11, M12],
                     [M12, M22]])

def C_2R(q, qdot, params):
    q1, q2 = q
    q1dot, q2dot = qdot
    l1 = params.l1
    c2 = params.c2
    m2 = params.m2

    sin2 = np.sin(q2)
    h = m2 * l1 * c2 * sin2

    C11 = -h * q2dot
    C12 = -h * (q1dot + q2dot)
    C21 = h * q1dot
    C22 = 0.0

    return np.array([[C11, C12],
                     [C21, C22]])

def g_2R(q, params):
    q1, q2 = q
    c1 = params.c1
    c2 = params.c2
    l1 = params.l1
    m1 = params.m1
    m2 = params.m2
    g = params.g

    g1 = (m1 * c1 + m2 * l1) * g * np.cos(q1) + m2 * c2 * g * np.cos(q1 + q2)
    g2 = m2 * c2 * g * np.cos(q1 + q2)

    return np.array([g1, g2])

def forward_dynamics_2R(q, qdot, tau, params):
    M = M_2R(q, params)
    C = C_2R(q, qdot, params)
    g_vec = g_2R(q, params)
    rhs = tau - C @ qdot - g_vec
    qddot = np.linalg.solve(M, rhs)
    return qddot

# Example usage
if __name__ == "__main__":
    params = TwoLinkParams(
        l1=1.0, l2=1.0,
        c1=0.5, c2=0.5,
        m1=1.0, m2=1.0,
        I1=0.1, I2=0.1,
        g=9.81
    )
    q = np.array([0.3, 0.4])
    qdot = np.array([0.2, -0.1])
    tau = np.array([0.5, 0.2])
    qddot = forward_dynamics_2R(q, qdot, tau, params)
    print("qddot =", qddot)
      
