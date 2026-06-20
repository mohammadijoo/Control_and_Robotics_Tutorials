import numpy as np

# Four-bar geometric parameters
a, b, c, d = 0.2, 0.4, 0.3, 0.5  # [m]

# Link inertias about joint axes (simple approx: I = m * l^2 / 12)
m2, m3, m4 = 1.0, 1.0, 1.0
I2 = m2 * a**2 / 12.0
I3 = m3 * b**2 / 12.0
I4 = m4 * c**2 / 12.0
g  = 9.81

def constraints(q):
    """Loop-closure constraints phi(q) in R^2."""
    th2, th3, th4 = q
    phi1 = a * np.cos(th2) + b * np.cos(th3) - d - c * np.cos(th4)
    phi2 = a * np.sin(th2) + b * np.sin(th3) - c * np.sin(th4)
    return np.array([phi1, phi2])

def Jc(q):
    """Constraint Jacobian J_c(q) in R^{2x3}."""
    th2, th3, th4 = q
    return np.array([
        [-a * np.sin(th2), -b * np.sin(th3),  c * np.sin(th4)],
        [ a * np.cos(th2),  b * np.cos(th3), -c * np.cos(th4)]
    ])

def M(q):
    """Inertia matrix M(q) for planar 4-bar (very simplified)."""
    # Here we neglect coupling between joints for illustration and
    # use a diagonal inertia matrix.
    return np.diag([I2, I3, I4])

def h(q, qdot):
    """Coriolis + gravity vector h(q, qdot)."""
    th2, th3, th4 = q
    # Simple gravity acting at midpoints of links, projected to joint torques.
    # This is not exact but suffices to illustrate constraint handling.
    tau2_g = -m2 * g * (a / 2.0) * np.cos(th2)
    tau3_g = -m3 * g * (b / 2.0) * np.cos(th3)
    tau4_g = -m4 * g * (c / 2.0) * np.cos(th4)
    return np.array([tau2_g, tau3_g, tau4_g])

def Jc_dot(q, qdot):
    """Time derivative of J_c(q)."""
    th2, th3, th4 = q
    th2d, th3d, th4d = qdot
    # Differentiate Jc analytically
    J = Jc(q)
    Jdot = np.zeros_like(J)
    # d/dt [-a sin(th2)] = -a cos(th2) * th2d, etc.
    Jdot[0, 0] = -a * np.cos(th2) * th2d
    Jdot[0, 1] = -b * np.cos(th3) * th3d
    Jdot[0, 2] =  c * np.cos(th4) * th4d
    Jdot[1, 0] = -a * np.sin(th2) * th2d
    Jdot[1, 1] = -b * np.sin(th3) * th3d
    Jdot[1, 2] = -c * np.sin(th4) * th4d
    return Jdot

def constrained_dynamics_step(q, qdot, tau, dt):
    """
    Single time step of index-1 DAE integration:
    solve block system for qdd, lambda, then integrate.
    """
    Mc = M(q)
    hc = h(q, qdot)
    J = Jc(q)
    Jdot = Jc_dot(q, qdot)

    # Build KKT system
    Z = np.zeros((2, 2))
    K_top = np.hstack((Mc, J.T))
    K_bot = np.hstack((J,  Z))
    K = np.vstack((K_top, K_bot))

    rhs_top = tau - hc
    rhs_bot = -Jdot @ qdot
    rhs = np.concatenate((rhs_top, rhs_bot))

    sol = np.linalg.solve(K, rhs)
    qdd = sol[:3]
    lam = sol[3:]

    # Explicit Euler step (for simplicity)
    qdot_new = qdot + dt * qdd
    q_new = q + dt * qdot_new

    # Optional: project back to constraint manifold (simple Newton step)
    for _ in range(2):
        phi = constraints(q_new)
        Jn = Jc(q_new)
        dq = np.linalg.lstsq(Jn.T @ Jn, -Jn.T @ phi, rcond=None)[0]
        q_new = q_new + dq

    return q_new, qdot_new, qdd, lam

if __name__ == "__main__":
    # Initial consistent configuration (crank at 10 degrees)
    q = np.array([np.deg2rad(10.0), np.deg2rad(60.0), np.deg2rad(30.0)])
    qdot = np.zeros(3)
    tau = np.array([1.0, 0.0, 0.0])  # drive only the crank joint
    dt = 1e-3

    for k in range(1000):
        q, qdot, qdd, lam = constrained_dynamics_step(q, qdot, tau, dt)
        if k % 100 == 0:
            print(f"step {k}: th2 = {np.rad2deg(q[0]):.2f} deg, lambda = {lam}")
      
