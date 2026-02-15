import numpy as np

def skew3(omega):
    """Return [omega]_x for omega in R^3."""
    wx, wy, wz = omega
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]])

def se3_hat(S):
    """From 6-vector S = [omega; v] to 4x4 matrix in se(3)."""
    omega = S[0:3]
    v = S[3:6]
    mat = np.zeros((4, 4))
    mat[0:3, 0:3] = skew3(omega)
    mat[0:3, 3] = v
    return mat

def matrix_exp3(omega, theta):
    """SO(3) exponential using Rodrigues formula."""
    # omega is assumed to be unit length when used for a revolute joint
    wx = skew3(omega)
    theta2 = theta * theta
    return (np.eye(3)
            + np.sin(theta) * wx
            + (1.0 - np.cos(theta)) * (wx @ wx))

def matrix_exp6(S, theta):
    """
    SE(3) exponential of twist S and joint displacement theta
    using closed-form formulas for revolute and prismatic joints.
    """
    omega = S[0:3]
    v = S[3:6]
    norm_w = np.linalg.norm(omega)

    if norm_w > 1e-8:
        # revolute case
        omega_unit = omega / norm_w
        theta_scaled = norm_w * theta
        R = matrix_exp3(omega_unit, theta_scaled)
        wx = skew3(omega_unit)
        G = (np.eye(3) * theta_scaled
             + (1.0 - np.cos(theta_scaled)) * wx
             + (theta_scaled - np.sin(theta_scaled)) * (wx @ wx))
        p = G @ (v / norm_w)
    else:
        # prismatic case
        R = np.eye(3)
        p = v * theta

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def fkine_space(M, S_list, theta_list):
    """
    Forward kinematics via PoE in the space frame.

    M: 4x4 home configuration (SE(3))
    S_list: array-like of shape (n, 6) with screw axes S_i
    theta_list: length-n iterable of joint variables
    """
    T = np.eye(4)
    for S, theta in zip(S_list, theta_list):
        T = T @ matrix_exp6(np.asarray(S, dtype=float), float(theta))
    return T @ M

if __name__ == "__main__":
    # Example: planar 2R manipulator in 3D with z-axis rotations
    # Joint 1 axis: z through origin, Joint 2 axis: z through point (L1, 0, 0)
    L1 = 1.0
    L2 = 1.0
    S1 = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])        # omega = (0,0,1), q = (0,0,0)
    S2 = np.array([0.0, 0.0, 1.0, 0.0, -L1, 0.0])        # omega = (0,0,1), q = (L1,0,0)
    S_list = np.vstack((S1, S2))

    # Home configuration: arm stretched along x-axis
    M = np.eye(4)
    M[0, 3] = L1 + L2   # end-effector at (L1+L2,0,0) at theta=(0,0)

    theta = [np.deg2rad(30.0), np.deg2rad(45.0)]
    T_theta = fkine_space(M, S_list, theta)
    print("T(theta) =")
    print(T_theta)
      
