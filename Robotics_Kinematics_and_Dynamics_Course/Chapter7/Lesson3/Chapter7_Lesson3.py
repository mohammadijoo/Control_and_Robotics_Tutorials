import numpy as np

def skew(v):
    """Return [v]_x for v in R^3."""
    vx, vy, vz = v
    return np.array([[0.0, -vz,  vy],
                     [vz,  0.0, -vx],
                     [-vy, vx,  0.0]])

def se3_hat(S):
    """
    Map a twist S = [omega; v] in R^6 to se(3) hat matrix.
    S: shape (6,)
    """
    omega = S[0:3]
    v = S[3:6]
    mat = np.zeros((4, 4))
    mat[0:3, 0:3] = skew(omega)
    mat[0:3, 3] = v
    return mat

def se3_exp(S, theta, tol=1e-9):
    """
    Exponential map exp( [S]^ ^ theta ) for twist S, scalar theta.
    Uses the Rodrigues formula for SO(3) and standard SE(3) formula.
    """
    S_hat = se3_hat(S)
    omega = S[0:3]
    v = S[3:6]
    omega_norm = np.linalg.norm(omega)
    if omega_norm < tol:
        # Pure translation case
        T = np.eye(4)
        T[0:3, 3] = v * theta
        return T
    # Rotation part
    w = omega / omega_norm
    w_hat = skew(w)
    theta_w = omega_norm * theta
    R = (np.eye(3)
         + np.sin(theta_w) * w_hat
         + (1.0 - np.cos(theta_w)) * (w_hat @ w_hat))
    # Translation part
    G = (np.eye(3) * theta_w
         + (1.0 - np.cos(theta_w)) * w_hat
         + (theta_w - np.sin(theta_w)) * (w_hat @ w_hat))
    p = G @ (v / omega_norm)
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def adjoint(T):
    """
    Ad_T for T in SE(3) given as 4x4 matrix.
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3:6, 3:6] = R
    Ad[3:6, 0:3] = skew(p) @ R
    return Ad

def fk_poe_space(S_list, M, q):
    """
    Forward kinematics using space PoE.
    S_list: list of twists S_i in space frame, each shape (6,)
    M: 4x4 home configuration
    q: array of joint angles, shape (n,)
    """
    T = np.eye(4)
    for S, theta in zip(S_list, q):
        T = T @ se3_exp(S, theta)
    T = T @ M
    return T

def jacobian_space(S_list, q):
    """
    Spatial Jacobian J_s(q) for list of space screws S_i and configuration q.
    Returns 6 x n matrix.
    """
    n = len(S_list)
    J = np.zeros((6, n))
    T = np.eye(4)
    for i in range(n):
        if i == 0:
            J[:, 0] = S_list[0].reshape(6,)
        else:
            Ad_T = adjoint(T)
            J[:, i] = Ad_T @ S_list[i]
        # update T for next column
        T = T @ se3_exp(S_list[i], q[i])
    return J

def jacobian_body(S_list, M, q):
    """
    Body Jacobian J_b(q) computed via J_b = Ad_{T(q)^{-1}} * J_s(q).
    """
    T = fk_poe_space(S_list, M, q)
    J_s = jacobian_space(S_list, q)
    Ad_inv = adjoint(np.linalg.inv(T))
    return Ad_inv @ J_s

# Example usage for planar 2R with l1, l2
if __name__ == "__main__":
    l1, l2 = 1.0, 0.8
    S1 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    S2 = np.array([0.0, 0.0, 1.0, 0.0, -l1, 0.0])
    S_list = [S1, S2]
    M = np.eye(4)
    M[0, 3] = l1 + l2
    q = np.array([0.5, -0.4])
    Tq = fk_poe_space(S_list, M, q)
    Js = jacobian_space(S_list, q)
    Jb = jacobian_body(S_list, M, q)
    print("T(q) =\n", Tq)
    print("J_s(q) =\n", Js)
    print("J_b(q) =\n", Jb)
      
