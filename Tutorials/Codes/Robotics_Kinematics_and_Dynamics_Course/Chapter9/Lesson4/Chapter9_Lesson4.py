import numpy as np

def planar_2r_jacobian(q1, q2, l1, l2):
    """
    Return the 2x2 translational Jacobian J(q) for a planar 2R arm.
    """
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)

    j11 = -l1 * s1 - l2 * s12
    j12 = -l2 * s12
    j21 =  l1 * c1 + l2 * c12
    j22 =  l2 * c12
    J = np.array([[j11, j12],
                  [j21, j22]], dtype=float)
    return J

def directional_capacity_box(J, f_dir, tau_max, eps=1e-9):
    """
    Directional load capacity under box torque limits.

    Parameters
    ----------
    J : (2, 2) ndarray
        Planar Jacobian at configuration q.
    f_dir : (2,) ndarray
        Desired force direction (need not be unit length).
    tau_max : (2,) ndarray
        Positive torque limits [tau1_max, tau2_max].
    eps : float
        Tolerance for treating small Jacobian projections as zero.

    Returns
    -------
    lam_star : float
        Maximum scalar lambda such that lambda * f_hat is feasible.
    f_hat : (2,) ndarray
        Normalized direction vector.
    active_joints : list of int
        Indices of joints that are limiting at lam_star.
    """
    f_dir = np.asarray(f_dir, dtype=float)
    tau_max = np.asarray(tau_max, dtype=float)
    norm = np.linalg.norm(f_dir)
    if norm < eps:
        raise ValueError("Direction vector f_dir must be nonzero.")
    f_hat = f_dir / norm

    v = J.T @ f_hat  # mapping from unit force to joint torques
    lambdas = []
    active_indices = []
    for i, v_i in enumerate(v):
        if abs(v_i) < eps:
            # This joint does not constrain this direction
            continue
        lam_i = tau_max[i] / abs(v_i)
        lambdas.append(lam_i)
        active_indices.append(i)

    if not lambdas:
        # In this (rare) case, the direction is unobservable in joint torques
        # and is not constrained by the box model.
        return np.inf, f_hat, []

    lam_star = min(lambdas)
    # Joints achieving the minimum are limiting
    limiting = [active_indices[i]
                for i, lam_i in enumerate(lambdas)
                if abs(lam_i - lam_star) < 1e-6]

    return lam_star, f_hat, limiting

if __name__ == "__main__":
    # Example: l1 = l2 = 1, q1 = 0, q2 = 0, symmetric torque limits
    l1 = l2 = 1.0
    q1 = 0.0
    q2 = 0.0
    J = planar_2r_jacobian(q1, q2, l1, l2)
    tau_max = np.array([10.0, 10.0])  # arbitrary units
    f_dir = np.array([0.0, -1.0])     # downward

    lam_star, f_hat, limiting = directional_capacity_box(J, f_dir, tau_max)
    print("J(q) =\n", J)
    print("Unit direction f_hat:", f_hat)
    print("Directional capacity lambda*:", lam_star)
    print("Limiting joints:", limiting)
      
