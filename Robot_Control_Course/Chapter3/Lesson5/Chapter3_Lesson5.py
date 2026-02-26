
import numpy as np

# 2-DOF planar arm parameters (simple model)
m1, m2 = 1.0, 1.0      # link masses
l1, l2 = 1.0, 1.0      # link lengths
lc1, lc2 = 0.5, 0.5    # COM distances
I1, I2 = 0.1, 0.1      # link inertias about COM
g = 9.81

def M_matrix(q):
    q1, q2 = q
    c2 = np.cos(q2)
    m11 = I1 + I2 + m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2*l1*lc2*c2)
    m12 = I2 + m2 * (lc2**2 + l1*lc2*c2)
    m22 = I2 + m2 * lc2**2
    M = np.array([[m11, m12],
                  [m12, m22]])
    return M

def C_matrix(q, qdot):
    q1, q2 = q
    q1dot, q2dot = qdot
    s2 = np.sin(q2)
    h = -m2 * l1 * lc2 * s2
    C = np.array([[h*q2dot, h*(q1dot + q2dot)],
                  [-h*q1dot, 0.0]])
    return C

def g_vector(q):
    q1, q2 = q
    g1 = (m1*lc1 + m2*l1) * g * np.cos(q1) + m2*lc2*g*np.cos(q1 + q2)
    g2 = m2*lc2*g*np.cos(q1 + q2)
    return np.array([g1, g2])

# Reference trajectory: simple joint sine motion
def qd(t):
    return np.array([0.5*np.sin(0.5*t), 0.5*np.cos(0.5*t)])

def qd_dot(t):
    return np.array([0.5*0.5*np.cos(0.5*t), -0.5*0.5*np.sin(0.5*t)])

def qd_ddot(t):
    return np.array([-0.5*0.5*0.5*np.sin(0.5*t), -0.5*0.5*0.5*np.cos(0.5*t)])

Kp = np.diag([100.0, 80.0])
Kd = np.diag([20.0, 16.0])

def controller_PD(q, qdot, t):
    e = qd(t) - q
    edot = qd_dot(t) - qdot
    tau = Kp.dot(e) + Kd.dot(edot)
    return tau

def controller_CT(q, qdot, t):
    e = qd(t) - q
    edot = qd_dot(t) - qdot
    v = qd_ddot(t) + Kd.dot(edot) + Kp.dot(e)
    M = M_matrix(q)
    C = C_matrix(q, qdot)
    g_vec = g_vector(q)
    tau = M.dot(v) + C.dot(qdot) + g_vec
    return tau

def simulate(controller, T=10.0, dt=0.001):
    steps = int(T / dt)
    q = np.array([0.0, 0.0])
    qdot = np.array([0.0, 0.0])

    Je = 0.0
    Jtau = 0.0
    e_max = 0.0

    qs = []
    qds = []
    ts = []

    for k in range(steps):
        t = k*dt
        e = qd(t) - q
        edot = qd_dot(t) - qdot
        tau = controller(q, qdot, t)

        M = M_matrix(q)
        C = C_matrix(q, qdot)
        g_vec = g_vector(q)

        qddot = np.linalg.solve(M, tau - C.dot(qdot) - g_vec)

        # simple explicit Euler (for lab usage; higher-order integrators are better)
        qdot = qdot + dt*qddot
        q = q + dt*qdot

        # metrics
        Je += e.T.dot(e) * dt
        Jtau += tau.T.dot(tau) * dt
        e_norm = np.linalg.norm(e)
        if e_norm > e_max:
            e_max = e_norm

        qs.append(q.copy())
        qds.append(qd(t))
        ts.append(t)

    return {
        "t": np.array(ts),
        "q": np.array(qs),
        "qd": np.array(qds),
        "Je": Je,
        "Jtau": Jtau,
        "e_max": e_max
    }

if __name__ == "__main__":
    res_PD = simulate(controller_PD)
    res_CT = simulate(controller_CT)

    print("PD:   J_e = {:.3f}, J_tau = {:.3f}, e_max = {:.3f}".format(
        res_PD["Je"], res_PD["Jtau"], res_PD["e_max"]))
    print("CT:   J_e = {:.3f}, J_tau = {:.3f}, e_max = {:.3f}".format(
        res_CT["Je"], res_CT["Jtau"], res_CT["e_max"]))

    # Example: plot first joint tracking (requires matplotlib)
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(res_PD["t"], res_PD["qd"][:,0], label="q1_d")
    plt.plot(res_PD["t"], res_PD["q"][:,0], label="q1_PD")
    plt.plot(res_CT["t"], res_CT["q"][:,0], label="q1_CT")
    plt.xlabel("t [s]")
    plt.ylabel("joint 1 [rad]")
    plt.legend()
    plt.show()
