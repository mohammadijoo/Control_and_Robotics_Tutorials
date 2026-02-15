import numpy as np

class Planar2R:
    def __init__(self, l1, l2, lc1, lc2, m1, m2, I1, I2, g=9.81):
        self.l1 = l1
        self.l2 = l2
        self.lc1 = lc1
        self.lc2 = lc2
        self.m1 = m1
        self.m2 = m2
        self.I1 = I1
        self.I2 = I2
        self.g = g

    def M(self, q):
        q1, q2 = q
        c2 = np.cos(q2)

        m11 = (self.I1 + self.I2
               + self.m1 * self.lc1**2
               + self.m2 * (self.l1**2 + self.lc2**2 + 2.0 * self.l1 * self.lc2 * c2))
        m12 = self.I2 + self.m2 * (self.lc2**2 + self.l1 * self.lc2 * c2)
        m22 = self.I2 + self.m2 * self.lc2**2

        return np.array([[m11, m12],
                         [m12, m22]])

    def C(self, q, qd):
        q1, q2 = q
        q1d, q2d = qd
        s2 = np.sin(q2)
        h = -self.m2 * self.l1 * self.lc2 * s2

        # One common choice of C matrix
        c11 = h * q2d
        c12 = h * (q1d + q2d)
        c21 = -h * q1d
        c22 = 0.0

        return np.array([[c11, c12],
                         [c21, c22]])

    def g_vec(self, q):
        q1, q2 = q
        g1 = ((self.m1 * self.lc1 + self.m2 * self.l1) * self.g * np.cos(q1)
              + self.m2 * self.lc2 * self.g * np.cos(q1 + q2))
        g2 = self.m2 * self.lc2 * self.g * np.cos(q1 + q2)
        return np.array([g1, g2])

    def dynamics(self, q, qd, u):
        """Compute qdd = M(q)^(-1) (u - C(q,qd) qd - g(q))."""
        Mq = self.M(q)
        Cq = self.C(q, qd)
        gq = self.g_vec(q)
        rhs = u - Cq.dot(qd) - gq
        qdd = np.linalg.solve(Mq, rhs)
        return qdd
      
