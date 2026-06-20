"""
Chapter22_Lesson4.py
Requirements for Implementing State Feedback

This script checks the minimum implementation requirements for a continuous-time
state-feedback law u(t) = -K x(t) + r(t).

Libraries used:
  - NumPy for linear algebra
No pole-placement algorithm is used here; the gain K is assumed to be supplied
by a later design step.
"""

import numpy as np


def controllability_matrix(A, B):
    """Return Wc = [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def observability_matrix(A, C):
    """Return Wo = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def rank(M, tol=1e-9):
    """Numerical matrix rank wrapper."""
    return int(np.linalg.matrix_rank(M, tol=tol))


def stabilizability_pbh(A, B, tol=1e-8):
    """
    PBH stabilizability test for continuous-time systems.

    The pair (A,B) is stabilizable iff every eigenvalue lambda of A with
    Re(lambda) >= 0 satisfies rank([lambda I - A, B]) = n.
    """
    n = A.shape[0]
    failed = []
    for lam in np.linalg.eigvals(A):
        if lam.real >= -tol:
            pbh = np.hstack((lam * np.eye(n) - A, B))
            if rank(pbh, tol) < n:
                failed.append(lam)
    return len(failed) == 0, failed


def full_state_is_directly_measured(C, n, tol=1e-9):
    """
    A necessary condition for direct implementation without observer is that
    the measurement map y = Cx contains enough independent information to
    reconstruct x algebraically.
    """
    return rank(C, tol) == n


def simulate_closed_loop(A, B, K, x0, dt=0.01, tf=5.0):
    """
    From-scratch forward-Euler simulation of x_dot = (A - BK)x.
    This is intentionally simple; production simulation should use solve_ivp
    or a validated real-time integration/discretization method.
    """
    Acl = A - B @ K
    steps = int(tf / dt)
    xs = np.zeros((steps + 1, len(x0)))
    ts = np.linspace(0.0, tf, steps + 1)
    xs[0, :] = x0
    for k in range(steps):
        xs[k + 1, :] = xs[k, :] + dt * (Acl @ xs[k, :])
    return ts, xs


def main():
    # Example plant: second-order stable open-loop system.
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])

    # Position-only measurement is not enough for direct full-state feedback.
    C_position_only = np.array([[1.0, 0.0]])

    # Full-state measurement means both states are measured or reconstructed.
    C_full = np.eye(2)

    # K is assumed to come from a design step; this lesson checks implementation readiness.
    K = np.array([[4.0, 2.0]])

    n = A.shape[0]
    print("=== Requirement checks for u = -Kx + r ===")
    print("State dimension n:", n)
    print("Input dimension m:", B.shape[1])

    Wc = controllability_matrix(A, B)
    print("\nControllability matrix Wc:\n", Wc)
    print("rank(Wc) =", rank(Wc), "required =", n)

    ok_stab, bad_modes = stabilizability_pbh(A, B)
    print("PBH stabilizable?", ok_stab)
    if bad_modes:
        print("Unstabilizable nondecaying modes:", bad_modes)

    print("\nFull state measured with C_position_only?",
          full_state_is_directly_measured(C_position_only, n))
    print("Full state measured with C_full?",
          full_state_is_directly_measured(C_full, n))

    Acl = A - B @ K
    print("\nClosed-loop matrix Acl = A - BK:\n", Acl)
    print("closed-loop eigenvalues:", np.linalg.eigvals(Acl))

    cond_Wc = np.linalg.cond(Wc)
    print("condition number of Wc:", cond_Wc)
    if cond_Wc > 1e6:
        print("Warning: controllability calculation is ill-conditioned.")

    x0 = np.array([1.0, 0.0])
    t, x = simulate_closed_loop(A, B, K, x0)
    print("\nFinal simulated state at t =", t[-1], "is", x[-1, :])


if __name__ == "__main__":
    main()
