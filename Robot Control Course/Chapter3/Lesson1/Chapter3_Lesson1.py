
import numpy as np

# Physical parameters (example values)
g = 9.81
m1, m2 = 1.0, 1.0
l1, l2 = 1.0, 1.0
lc1, lc2 = 0.5, 0.5
I1, I2 = 0.1, 0.1

def M_mat(q):
    q1, q2 = q
    c2 = np.cos(q2)
    M11 = I1 + I2 + m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*c2)
    M22 = I2 + m2*lc2**2
    M12 = I2 + m2*(lc2**2 + l1*lc2*c2)
    M21 = M12
    return np.array([[M11, M12],
                     [M21, M22]])

def C_mat(q, qd):
    q1, q2 = q
    q1d, q2d = qd
    s2 = np.sin(q2)
    h = -m2*l1*lc2*s2
    C11 = h*q2d
    C12 = h*(q1d + q2d)
    C21 = -h*q1d
    C22 = 0.0
    return np.array([[C11, C12],
                     [C21, C22]])

def g_vec(q):
    q1, q2 = q
    g1 = (m1*lc1 + m2*l1)*g*np.cos(q1) + m2*lc2*g*np.cos(q1 + q2)
    g2 = m2*lc2*g*np.cos(q1 + q2)
    return np.array([g1, g2])

def computed_torque(q, qd, q_d, qd_d, qdd_d, Kp, Kd):
    e = q - q_d
    ed = qd - qd_d
    v = qdd_d - Kd @ ed - Kp @ e
    M = M_mat(q)
    C = C_mat(q, qd)
    g_vec_q = g_vec(q)
    tau = M @ v + C @ qd + g_vec_q
    return tau

# Example of one control step
Kp = np.diag([50.0, 50.0])
Kd = np.diag([10.0, 10.0])

q    = np.array([0.1, 0.1])
qd   = np.array([0.0, 0.0])
q_d  = np.array([0.5, 0.3])
qd_d = np.array([0.0, 0.0])
qdd_d = np.array([0.0, 0.0])

tau = computed_torque(q, qd, q_d, qd_d, qdd_d, Kp, Kd)
print("tau =", tau)
