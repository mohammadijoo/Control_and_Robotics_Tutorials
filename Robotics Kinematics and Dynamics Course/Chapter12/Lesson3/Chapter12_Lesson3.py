import numpy as np

Z_AXIS = np.array([0.0, 0.0, 1.0])

class Link:
    def __init__(self, mass, inertia, r_com, joint_type="R"):
        """
        mass    : scalar
        inertia : 3x3 inertia matrix about link frame origin
        r_com   : 3D vector from link frame origin to center of mass
        joint_type: "R" for revolute, "P" for prismatic
        """
        self.m = mass
        self.I = inertia
        self.r_com = r_com
        self.joint_type = joint_type


def inverse_dynamics_newton_euler(q, qd, qdd, g, R_list, p_list, links):
    """
    q, qd, qdd : arrays of shape (n,)
    g          : 3D gravity vector (e.g. [0, 0, -9.81])
    R_list[i]  : 3x3 rotation from frame (i-1) to i
    p_list[i]  : 3D vector from frame (i-1) origin to frame i origin
    links[i]   : Link object for joint i
    returns    : tau of shape (n,)
    """
    n = len(links)

    # Forward recursion: velocities and accelerations
    w = [np.zeros(3) for _ in range(n + 1)]
    wd = [np.zeros(3) for _ in range(n + 1)]
    a = [np.zeros(3) for _ in range(n + 1)]

    # Base acceleration includes gravity
    a[0] = -np.asarray(g, dtype=float)

    for i in range(1, n + 1):
        R_i = R_list[i - 1]
        p_i = p_list[i - 1]
        link = links[i - 1]

        if link.joint_type == "R":
            # revolute joint
            w[i] = R_i @ w[i - 1] + Z_AXIS * qd[i - 1]
            wd[i] = (R_i @ wd[i - 1]
                     + Z_AXIS * qdd[i - 1]
                     + np.cross(w[i], Z_AXIS * qd[i - 1]))
            a[i] = (R_i @ (a[i - 1]
                           + np.cross(wd[i - 1], p_i)
                           + np.cross(w[i - 1],
                                      np.cross(w[i - 1], p_i))))
        else:
            # prismatic joint (simplified)
            w[i] = R_i @ w[i - 1]
            wd[i] = R_i @ wd[i - 1]
            a[i] = (R_i @ (a[i - 1]
                           + np.cross(wd[i - 1], p_i)
                           + np.cross(w[i - 1],
                                      np.cross(w[i - 1], p_i)))
                    + Z_AXIS * qdd[i - 1]
                    + 2.0 * np.cross(w[i], Z_AXIS * qd[i - 1]))

    # Center-of-mass accelerations, forces and moments
    F = [np.zeros(3) for _ in range(n)]
    N = [np.zeros(3) for _ in range(n)]
    for i in range(n):
        link = links[i]
        r_c = link.r_com
        a_c = (a[i + 1]
               + np.cross(wd[i + 1], r_c)
               + np.cross(w[i + 1], np.cross(w[i + 1], r_c)))
        F[i] = link.m * a_c
        N[i] = (link.I @ wd[i + 1]
                + np.cross(w[i + 1], link.I @ w[i + 1]))

    # Backward recursion: forces, moments, and joint efforts
    f_next = np.zeros(3)
    n_next = np.zeros(3)
    tau = np.zeros(n)

    for i in reversed(range(n)):
        R_ip1 = R_list[i] if i < n - 1 else np.eye(3)
        p_ip1 = p_list[i] if i < n - 1 else np.zeros(3)
        r_c = links[i].r_com

        f_i = R_ip1 @ f_next + F[i]
        n_i = (N[i]
               + R_ip1 @ n_next
               + np.cross(r_c, F[i])
               + np.cross(p_ip1, R_ip1 @ f_next))

        if links[i].joint_type == "R":
            tau[i] = n_i.dot(Z_AXIS)
        else:
            tau[i] = f_i.dot(Z_AXIS)

        f_next, n_next = f_i, n_i

    return tau
      
