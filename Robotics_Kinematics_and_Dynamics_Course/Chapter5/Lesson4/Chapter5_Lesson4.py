import numpy as np

def skew3(omega):
    """Return the 3x3 skew-symmetric matrix [omega]_x given omega in R^3."""
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def twist_hat(S):
    """Lift a 6-vector twist S = [omega; v] to a 4x4 se(3) matrix [S]."""
    omega = S[0:3]
    v = S[3:6]
    se3 = np.zeros((4, 4))
    se3[0:3, 0:3] = skew3(omega)
    se3[0:3, 3] = v
    return se3

def matrix_exp6(se3mat, theta):
    """
    Compute exp(se3mat * theta) using the Rodrigues formula.
    se3mat is a 4x4 matrix with rotational part in the top-left block.
    """
    Omega = se3mat[0:3, 0:3]
    v = se3mat[0:3, 3]
    # Extract axis-angle parameters from Omega (assume ||omega|| = 1 or 0)
    omega_hat = Omega
    omega = np.array([Omega[2,1] - Omega[1,2],
                      Omega[0,2] - Omega[2,0],
                      Omega[1,0] - Omega[0,1]]) * 0.5
    omega_norm = np.linalg.norm(omega)

    R = np.eye(3)
    p = np.zeros(3)

    if omega_norm > 1e-9:
        # Normalize
        omega = omega / omega_norm
        omega_hat = skew3(omega)
        # Rodrigues for rotation
        R = (np.eye(3)
             + np.sin(theta) * omega_hat
             + (1.0 - np.cos(theta)) * (omega_hat @ omega_hat))
        # Compute the matrix V(theta) used in screw motion
        V = (np.eye(3) * theta
             + (1.0 - np.cos(theta)) * omega_hat
             + (theta - np.sin(theta)) * (omega_hat @ omega_hat))
        p = V @ v
    else:
        # Pure translation
        R = np.eye(3)
        p = v * theta

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def fk_poe(S_list, M, q):
    """
    Forward kinematics via PoE for arbitrary n-DOF arm.
    S_list: (6, n) array of screw axes (space frame).
    M: 4x4 home configuration.
    q: (n,) vector of joint coordinates.
    """
    n = S_list.shape[1]
    T = M.copy()
    # Multiply exponentials from joint n down to 1 (space frame)
    for i in reversed(range(n)):
        S = S_list[:, i]
        se3mat = twist_hat(S)
        T = matrix_exp6(se3mat, q[i]) @ T
    return T

# Example usage: 7-DOF "toy" redundant arm (just for illustration)
# Here we choose simple screw axes; in practice you would compute them from the geometry.
S_list = np.array([
    [0, 0, 1, 0, 0, 0],      # Joint 1: rotation about z-axis
    [0, 1, 0, -0.3, 0, 0],   # Joint 2: rotation about y-axis
    [0, 1, 0, -0.6, 0, 0],   # Joint 3
    [0, 1, 0, -0.9, 0, 0],   # Joint 4
    [1, 0, 0, 0, -0.9, 0],   # Joint 5
    [0, 1, 0, -1.2, 0, 0],   # Joint 6
    [1, 0, 0, 0, -1.2, 0]    # Joint 7
], dtype=float).T  # shape (6, 7)

M = np.eye(4)
M[0:3, 3] = np.array([1.2, 0.0, 0.2])  # tool tip offset in home pose

q_example = np.deg2rad(np.array([0, 20, -30, 40, -10, 15, 5]))
T_example = fk_poe(S_list, M, q_example)
print("End-effector pose T(q) for redundant 7-DOF arm:\n", T_example)
      
