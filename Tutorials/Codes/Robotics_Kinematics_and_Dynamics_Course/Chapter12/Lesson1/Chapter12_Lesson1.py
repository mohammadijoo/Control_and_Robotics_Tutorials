import numpy as np

# Each link is represented as a dict with:
#   "R": 3x3 rotation matrix R_i^{i-1}
#   "p": 3x1 vector p_i (origin i-1 to origin i, in frame i-1)
#   "rc": 3x1 vector r_ci (origin i to COM_i, in frame i)
#   "joint_type": "R" for revolute, "P" for prismatic
#
# The function returns arrays of shape (n, 3)
#   omega[i], alpha[i], a[i], ac[i]

def forward_recursion(links, q, qd, qdd, g=np.array([0.0, 0.0, 9.81])):
    n = len(links)
    z = np.array([0.0, 0.0, 1.0])

    omega = np.zeros((n, 3))
    alpha = np.zeros((n, 3))
    a = np.zeros((n, 3))
    ac = np.zeros((n, 3))

    # Base initialization (fixed base)
    omega_prev = np.zeros(3)
    alpha_prev = np.zeros(3)
    a_prev = -g  # incorporate gravity

    for i, link in enumerate(links):
        R = link["R"]
        p = link["p"]
        rc = link["rc"]
        jt = link["joint_type"]

        qi = q[i]
        qdi = qd[i]
        qddi = qdd[i]

        if jt == "R":
            # Revolute joint
            omega_i = R @ omega_prev + z * qdi
            alpha_i = R @ alpha_prev + z * qddi + np.cross(omega_i, z * qdi)
            a_base = (a_prev
                      + np.cross(alpha_prev, p)
                      + np.cross(omega_prev, np.cross(omega_prev, p)))
            a_i = R @ a_base
        elif jt == "P":
            # Prismatic joint
            omega_i = R @ omega_prev
            alpha_i = R @ alpha_prev
            a_base = (a_prev
                      + np.cross(alpha_prev, p)
                      + np.cross(omega_prev, np.cross(omega_prev, p)))
            a_i = (R @ a_base
                   + z * qddi
                   + np.cross(2.0 * omega_i, z * qdi))
        else:
            raise ValueError("joint_type must be 'R' or 'P'")

        ac_i = (a_i
                + np.cross(alpha_i, rc)
                + np.cross(omega_i, np.cross(omega_i, rc)))

        omega[i, :] = omega_i
        alpha[i, :] = alpha_i
        a[i, :] = a_i
        ac[i, :] = ac_i

        omega_prev = omega_i
        alpha_prev = alpha_i
        a_prev = a_i

    return omega, alpha, a, ac


# Example: tiny 2R planar arm in the x-y plane (z axis out of plane)
def make_planar_2R_example():
    l1, l2 = 1.0, 1.0
    # Simple planar rotations about z with links along x
    def Rz(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]])

    # Here we assume identity R_i^{i-1} for simplicity and
    # store link lengths in p vectors in frame i-1
    links = []
    links.append({
        "R": np.eye(3),
        "p": np.array([0.0, 0.0, 0.0]),  # base to joint 1
        "rc": np.array([l1 / 2.0, 0.0, 0.0]),
        "joint_type": "R",
    })
    links.append({
        "R": np.eye(3),
        "p": np.array([l1, 0.0, 0.0]),   # joint 1 to joint 2
        "rc": np.array([l2 / 2.0, 0.0, 0.0]),
        "joint_type": "R",
    })
    return links

if __name__ == "__main__":
    links = make_planar_2R_example()
    q = np.array([0.5, 0.3])
    qd = np.array([0.2, 0.1])
    qdd = np.array([0.0, 0.05])

    omega, alpha, a, ac = forward_recursion(links, q, qd, qdd)
    print("omega:\n", omega)
    print("alpha:\n", alpha)
    print("a:\n", a)
    print("ac:\n", ac)
      
