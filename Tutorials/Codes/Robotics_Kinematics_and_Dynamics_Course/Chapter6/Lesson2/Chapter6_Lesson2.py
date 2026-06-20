import numpy as np

def ik_2r(x, y, l1, l2, tol=1e-9):
    """
    Analytical IK for a planar 2R arm.
    Returns a list of solution tuples (theta1, theta2).
    Angles are in radians.
    """
    r2 = x**2 + y**2
    # Reachability check
    if r2 > (l1 + l2)**2 + tol or r2 < (l1 - l2)**2 - tol:
        return []  # no real solutions

    # cos(theta2)
    c2 = (r2 - l1**2 - l2**2) / (2.0 * l1 * l2)
    # Numerical clipping for robustness
    c2 = np.clip(c2, -1.0, 1.0)
    # two possible s2 signs
    s2_candidates = [np.sqrt(1.0 - c2**2), -np.sqrt(1.0 - c2**2)]

    phi = np.arctan2(y, x)
    solutions = []
    for s2 in s2_candidates:
        theta2 = np.arctan2(s2, c2)
        # atan2 term for projection of l2
        k1 = l1 + l2 * c2
        k2 = l2 * s2
        psi = np.arctan2(k2, k1)
        theta1 = phi - psi
        solutions.append((theta1, theta2))
    return solutions

if __name__ == "__main__":
    l1, l2 = 0.5, 0.4
    x_d, y_d = 0.6, 0.3
    sols = ik_2r(x_d, y_d, l1, l2)
    for i, (t1, t2) in enumerate(sols):
        print(f"Solution {i}: theta1 = {t1:.4f}, theta2 = {t2:.4f}")
      
