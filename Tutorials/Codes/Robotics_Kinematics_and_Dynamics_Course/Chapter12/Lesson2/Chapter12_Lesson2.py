import numpy as np

def newton_euler_backward(F_list, N_list, R_ip1_i_list, p_i_list,
                          z_list, joint_type_list,
                          f_tip=None, n_tip=None):
    """
    Backward recursion for a serial chain.

    Parameters
    ----------
    F_list : list of (3,) arrays
        Inertial forces F_i at link CoM in frame {i}.
    N_list : list of (3,) arrays
        Inertial moments N_i about frame origin {i}.
    R_ip1_i_list : list of (3,3) arrays
        Rotation matrices ^i R_{i+1} (from frame {i+1} to {i}).
        Length n, but R_ip1_i_list[-1] is used only if you model a tip link.
    p_i_list : list of (3,) arrays
        Vectors p_i from origin of frame {i} to origin of frame {i+1}, in frame {i}.
    z_list : list of (3,) arrays
        Joint axes z_i expressed in frame {i}.
    joint_type_list : list of str
        "R" for revolute, "P" for prismatic.
    f_tip, n_tip : (3,) arrays or None
        External wrench at the terminal body (expressed in last frame).

    Returns
    -------
    f_list, n_list, tau_list : lists of arrays/scalars
        Internal forces, moments, and generalized efforts at each joint.
    """
    n = len(F_list)
    assert len(N_list) == n
    assert len(R_ip1_i_list) == n
    assert len(p_i_list) == n
    assert len(z_list) == n
    assert len(joint_type_list) == n

    if f_tip is None:
        f_next = np.zeros(3)
    else:
        f_next = np.asarray(f_tip).reshape(3)
    if n_tip is None:
        n_next = np.zeros(3)
    else:
        n_next = np.asarray(n_tip).reshape(3)

    f_list = [np.zeros(3) for _ in range(n)]
    n_list = [np.zeros(3) for _ in range(n)]
    tau_list = [0.0 for _ in range(n)]

    # Backward sweep: i = n-1, ..., 0
    for i in range(n - 1, -1, -1):
        R_ip1_i = R_ip1_i_list[i]
        p_i = p_i_list[i]
        F_i = F_list[i]
        N_i = N_list[i]
        z_i = z_list[i]
        jt = joint_type_list[i]

        # Propagate child wrench into frame {i}
        f_child_i = R_ip1_i @ f_next
        n_child_i = R_ip1_i @ n_next + np.cross(p_i, f_child_i)

        # Internal wrench at joint i
        f_i = F_i + f_child_i
        # r_i_c is already accounted for in N_i via N_i + r_i_c x F_i,
        # or you can add that explicitly here depending on how you define N_i.
        n_i = N_i + n_child_i

        f_list[i] = f_i
        n_list[i] = n_i

        if jt == "R":
            tau_i = float(z_i.dot(n_i))
        elif jt == "P":
            tau_i = float(z_i.dot(f_i))
        else:
            raise ValueError("joint_type_list must contain only 'R' or 'P'")

        tau_list[i] = tau_i

        # Wrench at this joint becomes "child" for previous joint
        f_next = f_i
        n_next = n_i

    return f_list, n_list, tau_list

# Example usage: 2-link planar manipulator (simplified)
if __name__ == "__main__":
    n = 2
    F_list = [np.array([1.0, 0.0, 0.0]),
              np.array([0.5, 0.0, 0.0])]
    N_list = [np.array([0.0, 0.0, 0.2]),
              np.array([0.0, 0.0, 0.1])]
    R_ip1_i_list = [np.eye(3), np.eye(3)]
    p_i_list = [np.array([0.5, 0.0, 0.0]),
                np.array([0.3, 0.0, 0.0])]
    z_list = [np.array([0.0, 0.0, 1.0]),
              np.array([0.0, 0.0, 1.0])]
    joint_type_list = ["R", "R"]

    f_list, n_list, tau_list = newton_euler_backward(
        F_list, N_list, R_ip1_i_list, p_i_list,
        z_list, joint_type_list,
        f_tip=np.zeros(3), n_tip=np.zeros(3)
    )

    print("Joint torques:", tau_list)
      
