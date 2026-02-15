
import numpy as np
from numpy import cos, sin
from dataclasses import dataclass

@dataclass
class TwoLinkParams:
    l1: float
    l2: float
    m1: float
    m2: float
    I1: float
    I2: float
    g: float = 9.81

# True and nominal parameters
true_params = TwoLinkParams(l1=1.0, l2=1.0,
                            m1=3.0, m2=2.0,
                            I1=0.2, I2=0.1)
nom_params  = TwoLinkParams(l1=1.0, l2=1.0,
                            m1=2.5, m2=1.5,   # under-estimated masses
                            I1=0.2, I2=0.1)

def M_matrix(q, p: TwoLinkParams):
    q1, q2 = q
    c2 = cos(q2)
    m11 = p.I1 + p.I2 + p.m1*(p.l1**2)/4.0 + p.m2*(p.l1**2 + (p.l2**2)/4.0 + p.l1*p.l2*c2)
    m12 = p.I2 + p.m2*((p.l2**2)/4.0 + 0.5*p.l1*p.l2*c2)
    m22 = p.I2 + p.m2*(p.l2**2)/4.0
    return np.array([[m11, m12],
                     [m12, m22]])

def C_matrix(q, qd, p: TwoLinkParams):
    q1, q2 = q
    q1d, q2d = qd
    s2 = sin(q2)
    h = -p.m2*p.l1*p.l2*s2*(2*q1d*q2d + q2d**2)
    h1 = p.m2*p.l1*p.l2*s2*(q1d**2)
    # A simple skew-symmetric-like structure
    c11 = -p.m2*p.l1*p.l2*s2*q2d
    c12 = -p.m2*p.l1*p.l2*s2*(q1d + q2d)
    c21 =  p.m2*p.l1*p.l2*s2*q1d
    c22 = 0.0
    return np.array([[c11, c12],
                     [c21, c22]])

def g_vector(q, p: TwoLinkParams):
    q1, q2 = q
    g1 = (p.m1*p.l1/2.0 + p.m2*p.l1)*p.g*cos(q1) + p.m2*p.l2/2.0*p.g*cos(q1 + q2)
    g2 = p.m2*p.l2/2.0*p.g*cos(q1 + q2)
    return np.array([g1, g2])

# Desired trajectory: simple sinusoid
def qd_des(t):
    qd  = np.array([0.5*np.sin(0.5*t), 0.3*np.sin(0.5*t)])
    qd1 = np.array([0.5*0.5*np.cos(0.5*t), 0.3*0.5*np.cos(0.5*t)])
    qd2 = np.array([-0.5*(0.5**2)*np.sin(0.5*t), -0.3*(0.5**2)*np.sin(0.5*t)])
    return qd, qd1, qd2

Kp = np.diag([25.0, 16.0])
Kd = np.diag([10.0, 8.0])

def dynamics(t, x):
    q  = x[0:2]
    qd = x[2:4]
    qd_d, qd1_d, qd2_d = qd_des(t)

    e   = q  - qd_d
    ed  = qd - qd1_d

    # Computed-torque using nominal model
    Mn = M_matrix(q, nom_params)
    Cn = C_matrix(q, qd, nom_params)
    gn = g_vector(q, nom_params)
    v  = qd2_d - Kd @ ed - Kp @ e
    tau = Mn @ v + Cn @ qd + gn

    # True robot dynamics
    Mr = M_matrix(q, true_params)
    Cr = C_matrix(q, qd, true_params)
    gr = g_vector(q, true_params)
    qdd = np.linalg.solve(Mr, tau - Cr @ qd - gr)

    return np.hstack((qd, qdd))

# Integrate with solve_ivp
if __name__ == "__main__":
    from scipy.integrate import solve_ivp
    T = 10.0
    x0 = np.zeros(4)  # start at rest
    sol = solve_ivp(dynamics, [0.0, T], x0, max_step=0.01, rtol=1e-6, atol=1e-8)
    t = sol.t
    q  = sol.y[0:2, :]
    qd = np.array([qd_des(tt)[0] for tt in t]).T
    e  = q - qd

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(t, e[0, :], label="e1")
    plt.plot(t, e[1, :], label="e2")
    plt.xlabel("time [s]")
    plt.ylabel("tracking error [rad]")
    plt.legend()
    plt.grid(True)
    plt.show()
