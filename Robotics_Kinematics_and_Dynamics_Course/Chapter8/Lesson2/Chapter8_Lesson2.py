import numpy as np

def yoshikawa_manipulability(J, tol=1e-6):
    """
    Compute Yoshikawa's velocity manipulability index
    w(q) = product of nonzero singular values of J(q).
    Parameters
    ----------
    J : (m, n) ndarray
        Jacobian matrix at configuration q.
    tol : float
        Threshold below which singular values are treated as zero.
    """
    # Singular value decomposition
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    # Keep only significant singular values
    s_nonzero = s[s > tol]
    if s_nonzero.size == 0:
        return 0.0
    return float(np.prod(s_nonzero))


def jacobian_2r(theta1, theta2, l1, l2):
    """
    Planar 2R Jacobian for end-effector position (x, y).
    """
    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    s12 = np.sin(theta1 + theta2)
    c12 = np.cos(theta1 + theta2)

    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ], dtype=float)
    return J


if __name__ == "__main__":
    l1 = 1.0
    l2 = 0.7
    theta1 = 0.5
    theta2 = 1.0

    J = jacobian_2r(theta1, theta2, l1, l2)
    w = yoshikawa_manipulability(J)
    print("J =\n", J)
    print("Yoshikawa manipulability w =", w)

    # If using Robotics Toolbox for Python (Peter Corke)
    # from roboticstoolbox import DHRobot, RevoluteDH
    # Define robot, compute J0 = robot.jacob0(q) and pass to yoshikawa_manipulability
      
