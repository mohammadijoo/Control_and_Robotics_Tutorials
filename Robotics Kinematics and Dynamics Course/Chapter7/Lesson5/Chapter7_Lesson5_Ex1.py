import modern_robotics as mr

def fk_se3(q, M, Slist):
    """
    SE(3) forward kinematics using product of exponentials.
    M     : 4x4 home configuration
    Slist : 6xn screw-axis matrix
    """
    return mr.FKinSpace(M, Slist, q)

def numeric_body_jacobian(fk_se3_fun, q, h=1e-6):
    """
    Numeric body Jacobian via SE(3) finite differences.
    fk_se3_fun : callable, fk_se3_fun(q) -> 4x4 SE(3) matrix T
    q          : 1D array (n,)
    h          : stepsize
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    n = q.size
    T0 = fk_se3_fun(q)
    Jb = np.zeros((6, n))

    for i in range(n):
        dq = np.zeros_like(q)
        dq[i] = h
        Tp = fk_se3_fun(q + dq)

        # Body twist corresponding to T0^{-1} Tp
        Delta = np.linalg.inv(T0) @ Tp
        xi_hat = mr.MatrixLog6(Delta)
        xi = mr.se3ToVec(xi_hat)  # 6D vector
        Jb[:, i] = xi / h

    return Jb

def check_jacobian(fk_fun, jac_fun, q, h=1e-6, atol=1e-8):
    """
    Compare numeric and analytic Jacobians, return relative Frobenius error.
    fk_fun  : pose function for numeric Jacobian (vector or SE(3))
    jac_fun : analytic Jacobian implementation, jac_fun(q) -> m x n
    """
    J_ana = np.asarray(jac_fun(q), dtype=float)
    # Choose appropriate numeric routine
    if J_ana.shape[0] == 6 and J_ana.shape[1] == q.size:
        J_num = numeric_body_jacobian(fk_fun, q, h)
    else:
        J_num = numeric_jacobian_vec(fk_fun, q, h)

    diff = J_num - J_ana
    err = np.linalg.norm(diff, ord="fro")
    ref = np.linalg.norm(J_ana, ord="fro") + atol
    rel_err = err / ref
    return rel_err, J_num, J_ana
      
