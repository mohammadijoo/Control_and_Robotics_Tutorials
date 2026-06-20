import math
from typing import List, Tuple

def ik_2r(l1: float, l2: float,
          x_d: float, y_d: float,
          tol: float = 1e-9) -> List[Tuple[float, float]]:
    """
    Geometric IK for a planar 2R manipulator.

    Args:
        l1, l2 : link lengths (> 0)
        x_d, y_d : desired end-effector position in base frame
        tol : numerical tolerance on workspace checks

    Returns:
        List of (q1, q2) solutions in radians.
    Raises:
        ValueError if the target is unreachable.
    """
    d_sq = x_d * x_d + y_d * y_d
    d = math.sqrt(d_sq)

    # Workspace feasibility via triangle inequality
    if d > l1 + l2 + tol or d < abs(l1 - l2) - tol:
        raise ValueError("Target is outside the reachable workspace")

    # Clamp cosine to [-1, 1] to avoid numerical errors
    c2 = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c2 = max(-1.0, min(1.0, c2))

    # Two possible signs for s2 = sin(q2)
    disc = max(0.0, 1.0 - c2 * c2)
    s2_candidates = [math.sqrt(disc), -math.sqrt(disc)]

    solutions: List[Tuple[float, float]] = []

    for s2 in s2_candidates:
        q2 = math.atan2(s2, c2)

        k1 = l1 + l2 * c2
        k2 = l2 * s2

        # atan2 takes (y, x)
        phi = math.atan2(y_d, x_d)
        psi = math.atan2(k2, k1)
        q1 = phi - psi

        solutions.append((q1, q2))

    return solutions

if __name__ == "__main__":
    l1, l2 = 1.0, 0.6
    x_d, y_d = 1.0, 0.4

    sols = ik_2r(l1, l2, x_d, y_d)
    for i, (q1, q2) in enumerate(sols):
        print(f"Solution {i}: q1 = {q1:.3f} rad, q2 = {q2:.3f} rad")
      
