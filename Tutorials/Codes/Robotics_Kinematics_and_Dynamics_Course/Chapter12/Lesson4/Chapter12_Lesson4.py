import numpy as np

def newton_euler(
    R, p, r_c, m, I,
    q, qd, qdd,
    g0=np.array([0.0, 0.0, -9.81]),
    joint_type=None
):
    """
    R[i] : 3x3 rotation from frame i-1 to i
    p[i] : 3-vector from frame i-1 to i (expressed in frame i-1)
    r_c[i] : 3-vector CoM offset in frame i
    m[i] : mass of link i
    I[i] : 3x3 inertia matrix of link i about frame i origin
    q, qd, qdd : joint states (length n)
    g0 : gravity in base frame
    joint_type[i] : 'R' or 'P' (default 'R')
    """

    n = len(q)
    if joint_type is None:
        joint_type = ['R'] * n

    z_axis = np.array([0.0, 0.0, 1.0])

    # Forward recursion
    omega = [np.zeros(3) for _ in range(n + 1)]
    omegadot = [np.zeros(3) for _ in range(n + 1)]
    a = [np.zeros(3) for _ in range(n + 1)]
    a_c = [np.zeros(3) for _ in range(n)]

    # Base: fixed base (can be generalized)
    omega[0] = np.zeros(3)
    omegadot[0] = np.zeros(3)
    a[0] = -np.array(g0, dtype=float)

    for i in range(1, n + 1):
        Ri = R[i]         # R_i^{i-1}
        pi = p[i]         # p_i^{i-1} expressed in frame i-1
        jt = joint_type[i-1]
        if jt == 'R':
            # revolute joint
            omega[i] = Ri @ omega[i-1] + z_axis * qd[i-1]
            omegadot[i] = (
                Ri @ omegadot[i-1]
                + z_axis * qdd[i-1]
                + np.cross(omega[i], z_axis * qd[i-1])
            )
            a[i] = (
                Ri @ (
                    a[i-1]
                    + np.cross(omegadot[i-1], pi)
                    + np.cross(omega[i-1], np.cross(omega[i-1], pi))
                )
            )
        else:
            # prismatic joint
            omega[i] = Ri @ omega[i-1]
            omegadot[i] = Ri @ omegadot[i-1]
            a[i] = (
                Ri @ (
                    a[i-1]
                    + np.cross(omegadot[i-1], pi)
                    + np.cross(omega[i-1], np.cross(omega[i-1], pi))
                )
                + 2.0 * np.cross(omega[i], z_axis * qd[i-1])
                + z_axis * qdd[i-1]
            )

        rc = r_c[i-1]
        a_c[i-1] = (
            a[i]
            + np.cross(omegadot[i], rc)
            + np.cross(omega[i], np.cross(omega[i], rc))
        )

    # Backward recursion
    f = [np.zeros(3) for _ in range(n + 2)]
    nvec = [np.zeros(3) for _ in range(n + 2)]
    tau = np.zeros(n)

    for i in range(n, 0, -1):
        # link index i-1 for data arrays
        mi = m[i-1]
        Ii = I[i-1]
        rc = r_c[i-1]
        # transform child wrench
        Ri_next = R[i+1].T if (i < n) else np.eye(3)
        # For simplicity: p_{i+1}^i zero if i == n
        pi_next = np.zeros(3) if i == n else p[i+1]

        f[i] = mi * a_c[i-1] + Ri_next @ f[i+1]
        nvec[i] = (
            Ii @ omegadot[i]
            + np.cross(omega[i], Ii @ omega[i])
            + np.cross(rc, mi * a_c[i-1])
            + Ri_next @ nvec[i+1]
            + np.cross(pi_next + rc, Ri_next @ f[i+1])
        )

        if joint_type[i-1] == 'R':
            tau[i-1] = nvec[i].dot(z_axis)
        else:
            tau[i-1] = f[i].dot(z_axis)

    return tau
      
