import numpy as np

def det_jacobian_2r(theta1, theta2, l1=1.0, l2=1.0):
    """
    Determinant of the 2x2 Jacobian for a planar 2R arm.
    """
    return l1 * l2 * np.sin(theta2)

def is_kinematic_singular(theta1, theta2, l1=1.0, l2=1.0, tol=1e-6):
    detJ = det_jacobian_2r(theta1, theta2, l1, l2)
    return abs(detJ) < tol

def jacobian_2r(theta1, theta2, l1=1.0, l2=1.0):
    s1, c1 = np.sin(theta1), np.cos(theta1)
    s12, c12 = np.sin(theta1 + theta2), np.cos(theta1 + theta2)
    J = np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [ l1 * c1 + l2 * c12,  l2 * c12]
    ])
    return J

def damped_pseudoinverse_step(theta1, theta2, dx,
                              l1=1.0, l2=1.0,
                              lam=1e-2):
    """
    One damped least-squares IK step in joint space:
        dq = J^T (J J^T + lam^2 I)^{-1} dx
    for a 2x2 Jacobian J.
    """
    J = jacobian_2r(theta1, theta2, l1, l2)
    JJt = J @ J.T
    lam2I = (lam ** 2) * np.eye(2)
    dq = J.T @ np.linalg.solve(JJt + lam2I, dx)
    return dq

# Example usage
theta1, theta2 = 0.0, 0.0   # fully stretched, singular
print("detJ =", det_jacobian_2r(theta1, theta2))
print("is singular? ", is_kinematic_singular(theta1, theta2))
      
