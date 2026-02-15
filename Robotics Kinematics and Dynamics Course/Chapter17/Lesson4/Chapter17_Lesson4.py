import numpy as np

def solve_contact_dynamics(M, h, Jc, tau, f_ext, qd, ac_des=None):
    """
    Solve
        [ M  -Jc.T ] [ qddot ] = [ S.T tau + f_ext - h ]
        [ Jc   0   ] [ lambda ]   [ ac_des - Jc_dot qd ]
    for qddot and lambda, given M, h, Jc at the current state.

    Parameters
    ----------
    M : (n,n) array_like
        Joint-space inertia matrix M(q).
    h : (n,) array_like
        Bias term h(q, qd).
    Jc : (m,n) array_like
        Contact Jacobian for active constraints.
    tau : (n_act,) or (n,) array_like
        Generalized effort; here we assume full actuation for simplicity.
    f_ext : (n,) array_like
        Generalized non-contact external forces.
    qd : (n,) array_like
        Generalized velocities.
    ac_des : (m,) array_like or None
        Desired constraint acceleration; if None, zero is used (rigid contact).

    Returns
    -------
    qddot : (n,) ndarray
        Joint accelerations satisfying the constraints.
    lam : (m,) ndarray
        Contact forces in constraint space.
    """
    M = np.asarray(M)
    h = np.asarray(h).reshape(-1)
    Jc = np.asarray(Jc)
    qd = np.asarray(qd).reshape(-1)
    f_ext = np.asarray(f_ext).reshape(-1)
    n = M.shape[0]
    m = Jc.shape[0]

    if ac_des is None:
        ac_des = np.zeros(m)
    else:
        ac_des = np.asarray(ac_des).reshape(-1)

    # For teaching purposes, we approximate Jc_dot qd numerically using a small finite difference.
    # In practice, robotics libraries (e.g., Pinocchio) can compute Jc_dot exactly.
    eps = 1e-6
    # Dummy approximation: assume Jc_dot qd = 0 here (quasi-static / low-speed assumption)
    Jc_dot_qd = np.zeros(m)

    # Assemble KKT matrix and right-hand side
    KKT = np.block([
        [M, -Jc.T],
        [Jc, np.zeros((m, m))]
    ])

    rhs = np.concatenate([
        tau + f_ext - h,
        ac_des - Jc_dot_qd
    ])

    sol = np.linalg.solve(KKT, rhs)
    qddot = sol[:n]
    lam = sol[n:]

    return qddot, lam

# Example: simple 2-DOF planar model with one holonomic contact constraint
if __name__ == "__main__":
    n = 2
    M = np.array([[2.0, 0.1],
                  [0.1, 1.0]])
    h = np.array([0.0, 0.0])
    Jc = np.array([[1.0, 1.0]])  # scalar constraint: q1 + q2 = constant
    tau = np.array([0.0, 0.0])
    f_ext = np.array([0.0, 0.0])
    qd = np.array([0.0, 0.0])

    qddot, lam = solve_contact_dynamics(M, h, Jc, tau, f_ext, qd)
    print("qddot =", qddot)
    print("lambda =", lam)
      
