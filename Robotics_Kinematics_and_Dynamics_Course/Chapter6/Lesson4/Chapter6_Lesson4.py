import numpy as np

def fk_2r(q, l1, l2):
    """
    Forward kinematics for 2R planar arm.
    q: array-like of size 2 [q1, q2]
    returns: np.array([x, y])
    """
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y])


def jacobian_2r(q, l1, l2):
    """
    Analytical Jacobian of 2R planar arm.
    J is 2x2.
    """
    q1, q2 = q
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)

    J = np.zeros((2, 2))
    J[0, 0] = -l1 * s1 - l2 * s12
    J[0, 1] = -l2 * s12
    J[1, 0] =  l1 * c1 + l2 * c12
    J[1, 1] =  l2 * c12
    return J


def ik_newton_2r(x_d, l1, l2, q0, tol=1e-6, max_iter=50, alpha=1.0):
    """
    Newton (square) IK for 2R planar arm.
    x_d: desired [x_d, y_d]
    q0: initial guess [q1, q2]
    Returns (q, success, iters)
    """
    q = np.array(q0, dtype=float)
    x_d = np.array(x_d, dtype=float)

    for k in range(max_iter):
        x = fk_2r(q, l1, l2)
        r = x - x_d
        err = np.linalg.norm(r)
        if err < tol:
            return q, True, k

        J = jacobian_2r(q, l1, l2)
        # Solve J * dq = -r
        try:
            dq = np.linalg.solve(J, -r)
        except np.linalg.LinAlgError:
            # singular Jacobian
            return q, False, k

        q = q + alpha * dq

    return q, False, max_iter


def ik_dls_2r(x_d, l1, l2, q0, tol=1e-6, max_iter=50, alpha=1.0, lam=1e-2):
    """
    Damped least-squares IK for 2R planar arm.
    Uses step dq = -(J^T J + lam^2 I)^(-1) J^T r
    """
    q = np.array(q0, dtype=float)
    x_d = np.array(x_d, dtype=float)

    for k in range(max_iter):
        x = fk_2r(q, l1, l2)
        r = x - x_d
        err = np.linalg.norm(r)
        if err < tol:
            return q, True, k

        J = jacobian_2r(q, l1, l2)
        JTJ = J.T @ J
        n = JTJ.shape[0]
        A = JTJ + (lam ** 2) * np.eye(n)
        g = J.T @ r
        dq = -np.linalg.solve(A, g)

        q = q + alpha * dq

    return q, False, max_iter


if __name__ == "__main__":
    l1, l2 = 1.0, 0.8
    x_d = np.array([1.2, 0.5])
    q0 = np.array([0.0, 0.0])

    q_newton, ok_n, it_n = ik_newton_2r(x_d, l1, l2, q0)
    print("Newton IK:", q_newton, "success:", ok_n, "iters:", it_n)

    q_dls, ok_d, it_d = ik_dls_2r(x_d, l1, l2, q0, lam=0.05)
    print("DLS IK:", q_dls, "success:", ok_d, "iters:", it_d)
      
