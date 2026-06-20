import numpy as np

def rot2(theta):
    """2D rotation matrix."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])

def fk_planar_2R(q, L):
    """
    Open-chain forward kinematics for a planar 2R arm.
    q: array-like [theta1, theta2]
    L: array-like [L1, L2] link lengths
    Returns end-effector position in R^2.
    """
    theta1, theta2 = q
    L1, L2 = L

    p1 = rot2(theta1) @ np.array([L1, 0.0])
    p2 = p1 + rot2(theta1 + theta2) @ np.array([L2, 0.0])
    return p2

def fourbar_constraint(theta1, theta3, L0, L1, L2, L3):
    """
    Loop-closure constraint Phi(theta1, theta3) for the four-bar example.
    Returns scalar Phi; feasible configurations satisfy Phi = 0.
    """
    pB = np.array([L1 * np.cos(theta1),
                   L1 * np.sin(theta1)])
    pC = np.array([L0 + L3 * np.cos(theta3),
                   0.0 + L3 * np.sin(theta3)])
    Phi = np.dot(pB - pC, pB - pC) - L2**2
    return Phi

def solve_fourbar_theta3(theta1, L0, L1, L2, L3,
                         theta3_init=0.0,
                         tol=1e-10,
                         max_iter=50):
    """
    Solve Phi(theta1, theta3) = 0 for theta3 using Newton iteration.
    For robustness in practice, bracketed solvers or multiple initial
    guesses are recommended.
    """
    theta3 = float(theta3_init)
    for k in range(max_iter):
        Phi = fourbar_constraint(theta1, theta3, L0, L1, L2, L3)

        # Numerical derivative dPhi/dtheta3 via symmetric difference
        h = 1e-6
        Phi_p = fourbar_constraint(theta1, theta3 + h, L0, L1, L2, L3)
        Phi_m = fourbar_constraint(theta1, theta3 - h, L0, L1, L2, L3)
        dPhi = (Phi_p - Phi_m) / (2.0 * h)

        if abs(dPhi) < 1e-12:
            break  # avoid division by zero

        step = Phi / dPhi
        theta3 -= step

        if abs(step) < tol:
            break
    return theta3

if __name__ == "__main__":
    # Open chain: 2R arm
    L = [0.5, 0.4]
    q = [0.5, -0.3]
    p_ee = fk_planar_2R(q, L)
    print("Open-chain end-effector position:", p_ee)

    # Closed chain: four-bar linkage
    L0, L1, L2, L3 = 0.8, 0.5, 0.7, 0.4
    theta1 = 0.4
    theta3 = solve_fourbar_theta3(theta1, L0, L1, L2, L3, theta3_init=0.2)
    print("Closed-chain solution theta3:", theta3)
    print("Constraint residual:",
          fourbar_constraint(theta1, theta3, L0, L1, L2, L3))
      
