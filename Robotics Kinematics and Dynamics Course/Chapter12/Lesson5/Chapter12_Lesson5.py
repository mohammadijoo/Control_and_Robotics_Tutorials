import numpy as np

class TwoLinkParams:
    def __init__(self, m1, m2, l1, c1, c2, I1, I2, g=9.81):
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.c1 = c1
        self.c2 = c2
        self.I1 = I1
        self.I2 = I2
        self.g  = g

def M_lagrange_2R(q, p):
    q1, q2 = q
    a = p.I1 + p.I2 + p.m1 * p.c1**2 + p.m2 * (p.l1**2 + p.c2**2)
    b = p.m2 * p.l1 * p.c2
    d = p.I2 + p.m2 * p.c2**2
    c2 = np.cos(q2)
    M11 = a + 2.0 * b * c2
    M12 = d + b * c2
    M22 = d
    return np.array([[M11, M12],
                     [M12, M22]], dtype=float)

def h_lagrange_2R(q, dq, p):
    q1, q2 = q
    dq1, dq2 = dq
    b = p.m2 * p.l1 * p.c2
    s2 = np.sin(q2)
    h1 = -b * s2 * (2.0 * dq1 * dq2 + dq2**2)
    h2 =  b * s2 * dq1**2
    return np.array([h1, h2], dtype=float)

def g_lagrange_2R(q, p):
    q1, q2 = q
    g1 = p.g * (p.c1 * p.m1 * np.cos(q1)
                + p.m2 * (p.l1 * np.cos(q1) + p.c2 * np.cos(q1 + q2)))
    g2 = p.g * (p.m2 * p.c2 * np.cos(q1 + q2))
    return np.array([g1, g2], dtype=float)

def tau_lagrange_2R(q, dq, ddq, p):
    M = M_lagrange_2R(q, p)
    h = h_lagrange_2R(q, dq, p)
    g = g_lagrange_2R(q, p)
    return M.dot(ddq) + h + g

# ----------------------------------------------------------------------
# Newton-Euler interface: you should plug in your own implementation
# from Chapter 12, Lesson 4. Here we declare the expected signature.
# ----------------------------------------------------------------------
def tau_newton_euler_2R(q, dq, ddq, p, g_base=None):
    """
    Placeholder: call your validated Newton-Euler implementation here.
    Arguments:
        q, dq, ddq: arrays of shape (2,)
        p: TwoLinkParams
        g_base: 3D gravity vector in base frame (e.g. [0, -p.g, 0])
    Returns:
        tau_NE: array of shape (2,)
    """
    raise NotImplementedError("Connect to your Newton-Euler implementation.")

# ----------------------------------------------------------------------
# Reconstruction of M_NE(q) and g_NE(q) using the strategy of Section 5
# ----------------------------------------------------------------------
def reconstruct_M_NE(q, p):
    n = 2
    M = np.zeros((n, n), dtype=float)
    dq = np.zeros(n)
    g_base = np.zeros(3)  # zero gravity
    for j in range(n):
        ddq = np.zeros(n)
        ddq[j] = 1.0
        tau = tau_newton_euler_2R(q, dq, ddq, p, g_base=g_base)
        M[:, j] = tau
    return M

def reconstruct_g_NE(q, p):
    n = 2
    ddq = np.zeros(n)
    dq = np.zeros(n)
    # gravity pointing along negative y-axis
    g_base = np.array([0.0, -p.g, 0.0])
    tau = tau_newton_euler_2R(q, dq, ddq, p, g_base=g_base)
    return tau

def monte_carlo_validation(num_samples=100):
    p = TwoLinkParams(m1=2.0, m2=1.0,
                      l1=1.0, c1=0.5, c2=0.5,
                      I1=0.2, I2=0.1)
    max_err = 0.0
    for k in range(num_samples):
        q  = np.array([np.random.uniform(-np.pi, np.pi),
                       np.random.uniform(-np.pi, np.pi)])
        dq = np.array([np.random.uniform(-1.0, 1.0),
                       np.random.uniform(-1.0, 1.0)])
        ddq = np.array([np.random.uniform(-2.0, 2.0),
                        np.random.uniform(-2.0, 2.0)])

        tau_L  = tau_lagrange_2R(q, dq, ddq, p)
        tau_NE = tau_newton_euler_2R(q, dq, ddq, p,
                                     g_base=np.array([0.0, -p.g, 0.0]))
        err = np.linalg.norm(tau_L - tau_NE, ord=np.inf)
        max_err = max(max_err, err)

    print("Max absolute torque error over", num_samples, "samples:", max_err)

    # Single configuration structural test
    q0 = np.array([0.3, -0.7])
    M_L = M_lagrange_2R(q0, p)
    g_L = g_lagrange_2R(q0, p)
    M_NE = reconstruct_M_NE(q0, p)
    g_NE = reconstruct_g_NE(q0, p)

    print("M_L(q0) =\n", M_L)
    print("M_NE(q0) =\n", M_NE)
    print("M_L - M_NE =\n", M_L - M_NE)
    print("g_L(q0) =", g_L)
    print("g_NE(q0) =", g_NE)
    print("g_L - g_NE =", g_L - g_NE)

if __name__ == "__main__":
    # Once tau_newton_euler_2R is connected, this will
    # provide a strong regression test of your implementation.
    monte_carlo_validation()
      
