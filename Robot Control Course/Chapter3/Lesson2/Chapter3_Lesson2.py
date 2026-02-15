
import numpy as np

# Simple 2-link planar arm parameters
m1, m2 = 1.0, 1.0
l1, l2 = 1.0, 1.0
lc1, lc2 = 0.5, 0.5
I1, I2 = 0.05, 0.05
g_const = 9.81

def M_matrix(q):
    q1, q2 = q
    c2 = np.cos(q2)
    a = I1 + I2 + m2 * l1**2
    b = m2 * l1 * lc2
    d11 = a + 2.0 * b * c2
    d12 = I2 + b * c2
    d21 = d12
    d22 = I2
    return np.array([[d11, d12],
                     [d21, d22]])

def C_matrix(q, qdot):
    q1, q2 = q
    q1dot, q2dot = qdot
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    b = m2 * l1 * lc2
    h = -b * s2
    # 2x2 Coriolis matrix (one common convention)
    return np.array([[h * q2dot, h * (q1dot + q2dot)],
                     [-h * q1dot, 0.0]])

def g_vector(q):
    q1, q2 = q
    g1 = (m1 * lc1 + m2 * l1) * g_const * np.cos(q1) \
         + m2 * lc2 * g_const * np.cos(q1 + q2)
    g2 = m2 * lc2 * g_const * np.cos(q1 + q2)
    return np.array([g1, g2])

def feedback_linearization_tau(q, qdot,
                               qd, qd_dot, qd_ddot,
                               Kp, Kv):
    """
    q, qdot, qd, qd_dot, qd_ddot: 2-element numpy arrays
    Kp, Kv: 2x2 gain matrices (numpy arrays)
    """
    e = q - qd
    e_dot = qdot - qd_dot
    v = qd_ddot - Kv @ e_dot - Kp @ e
    M = M_matrix(q)
    C = C_matrix(q, qdot)
    g = g_vector(q)
    tau = M @ v + C @ qdot + g
    return tau

# Example usage at one time instant
q = np.array([0.1, 0.1])
qdot = np.array([0.0, 0.0])
qd = np.array([0.5, -0.3])
qd_dot = np.array([0.0, 0.0])
qd_ddot = np.array([0.0, 0.0])

Kp = np.diag([25.0, 25.0])
Kv = np.diag([10.0, 10.0])

tau = feedback_linearization_tau(q, qdot, qd, qd_dot, qd_ddot, Kp, Kv)
print("Tau command:", tau)
