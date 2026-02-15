
import numpy as np

# --------------------------
# Robot parameters (2-link planar)
# --------------------------
l1, l2 = 1.0, 1.0      # link lengths
m1, m2 = 1.0, 1.0      # link masses
lc1, lc2 = 0.5, 0.5    # CoM distances
I1, I2 = 0.05, 0.05    # link inertias
g = 9.81

def forward_kinematics(q):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])

def jacobian(q):
    q1, q2 = q
    s1 = np.sin(q1); c1 = np.cos(q1)
    s12 = np.sin(q1 + q2); c12 = np.cos(q1 + q2)
    j11 = -l1 * s1 - l2 * s12
    j12 = -l2 * s12
    j21 =  l1 * c1 + l2 * c12
    j22 =  l2 * c12
    return np.array([[j11, j12],
                     [j21, j22]])

def jacobian_dot(q, qdot):
    # simple numerical approximation for lab purposes
    eps = 1e-6
    J0 = jacobian(q)
    Jdot = np.zeros((2, 2))
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps
        J_plus = jacobian(q + dq)
        J_minus = jacobian(q - dq)
        # approximate dJ/dq_i * qdot_i
        Jdot += (J_plus - J_minus) / (2.0 * eps) * qdot[i]
    return Jdot

def M_matrix(q):
    q1, q2 = q
    c2 = np.cos(q2)
    a = I1 + I2 + m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * c2)
    b = I2 + m2 * (lc2**2 + l1 * lc2 * c2)
    d = I2 + m2 * lc2**2
    return np.array([[a, b],
                     [b, d]])

def C_vector(q, qdot):
    q1, q2 = q
    q1dot, q2dot = qdot
    s2 = np.sin(q2)
    h = -m2 * l1 * lc2 * s2
    c1 = h * (2.0 * q1dot * q2dot + q2dot**2)
    c2 = h * q1dot**2
    return np.array([c1, c2])

def g_vector(q):
    q1, q2 = q
    g1 = (m1 * lc1 + m2 * l1) * g * np.cos(q1) + m2 * lc2 * g * np.cos(q1 + q2)
    g2 = m2 * lc2 * g * np.cos(q1 + q2)
    return np.array([g1, g2])

# --------------------------
# Quintic time scaling
# --------------------------
def s_quintic(t, T):
    tau = t / T
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5

def s_quintic_dot(t, T):
    tau = t / T
    return (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T

def s_quintic_ddot(t, T):
    tau = t / T
    return (60 * tau - 180 * tau**2 + 120 * tau**3) / (T**2)

# --------------------------
# Desired Cartesian motion: line from x0 to xf
# --------------------------
x0 = np.array([0.8, 0.4])
xf = np.array([1.2, 0.6])
T = 4.0    # duration [s]

def x_des(t):
    s = s_quintic(t, T)
    return x0 + s * (xf - x0)

def xdot_des(t):
    sd = s_quintic_dot(t, T)
    return sd * (xf - x0)

def xddot_des(t):
    sdd = s_quintic_ddot(t, T)
    return sdd * (xf - x0)

# --------------------------
# Control gains and simulation setup
# --------------------------
Kp = np.diag([100.0, 100.0])
Kd = np.diag([20.0, 20.0])

dt = 0.001
t_final = 5.0
N = int(t_final / dt)

q = np.array([0.0, 0.0])
qdot = np.array([0.0, 0.0])

xs_log = []
xd_log = []
err_log = []

for k in range(N):
    t = k * dt

    # forward kinematics
    x = forward_kinematics(q)
    J = jacobian(q)
    xdot = J @ qdot
    Jdot = jacobian_dot(q, qdot)

    # desired trajectory
    xd = x_des(t)
    xdd = xddot_des(t)
    xd_dot = xdot_des(t)

    # errors
    e = xd - x
    e_dot = xd_dot - xdot

    # task-space PD law
    a_x = xdd + Kd @ e_dot + Kp @ e

    # joint accelerations via kinematic relation
    J_inv = np.linalg.inv(J)
    qdd_des = J_inv @ (a_x - Jdot @ qdot)

    # inverse dynamics (computed torque)
    M = M_matrix(q)
    C = C_vector(q, qdot)
    g_vec = g_vector(q)
    tau = M @ qdd_des + C + g_vec

    # plant: here we assume ideal dynamics = inverse of the same model
    qdd = np.linalg.solve(M, tau - C - g_vec)

    # integrate
    qdot = qdot + dt * qdd
    q = q + dt * qdot

    xs_log.append(x.copy())
    xd_log.append(xd.copy())
    err_log.append(e.copy())

xs_log = np.array(xs_log)
xd_log = np.array(xd_log)
err_log = np.array(err_log)

print("Final end-effector error:", err_log[-1])
