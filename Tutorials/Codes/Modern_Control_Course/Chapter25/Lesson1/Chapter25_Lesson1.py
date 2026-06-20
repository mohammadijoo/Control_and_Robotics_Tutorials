"""
Chapter25_Lesson1.py
Uncontrollable Modes and Unassignable Poles

Requires:
    numpy
    scipy
Optional:
    python-control (not required here)

This script demonstrates:
1. Kalman controllability rank test
2. PBH test
3. State-feedback pole assignment limitation
4. Invariance of uncontrollable eigenvalues under A_cl = A - B K
"""

import numpy as np
from numpy.linalg import matrix_rank, eigvals
from scipy.signal import place_poles


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    return np.hstack(blocks)


def pbh_controllability_report(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> None:
    """Print PBH ranks rank([lambda I - A, B]) for each eigenvalue."""
    n = A.shape[0]
    print("PBH test:")
    for lam in eigvals(A):
        M = np.hstack((lam * np.eye(n) - A, B))
        r = matrix_rank(M, tol=tol)
        status = "controllable at this eigenvalue" if r == n else "UNCONTROLLABLE mode"
        print(f"  lambda = {lam: .6g}, rank = {r}/{n}: {status}")


def kalman_decomposition_by_reachable_basis(A: np.ndarray, B: np.ndarray):
    """
    Construct a simple reachable-basis transformation using QR.
    The first r columns of T span the reachable subspace.
    """
    C = controllability_matrix(A, B)
    Q, R = np.linalg.qr(C)
    r = matrix_rank(C)

    # Build a full orthonormal basis by completing Q[:, :r]
    Qr = Q[:, :r]
    random_block = np.eye(A.shape[0])
    U, _ = np.linalg.qr(np.hstack([Qr, random_block]))
    T = U[:, :A.shape[0]]

    Abar = T.T @ A @ T
    Bbar = T.T @ B
    return T, Abar, Bbar, r


def main() -> None:
    # Example: the second state is not actuated and produces an unassignable pole at +2.
    A = np.array([[0.0, 0.0],
                  [0.0, 2.0]])
    B = np.array([[1.0],
                  [0.0]])

    print("A =\n", A)
    print("B =\n", B)

    C = controllability_matrix(A, B)
    print("\nKalman controllability matrix C = [B, AB] =\n", C)
    print("rank(C) =", matrix_rank(C), "out of n =", A.shape[0])
    pbh_controllability_report(A, B)

    # Try state feedback K = [k1, k2].  Only the first closed-loop pole can move.
    print("\nClosed-loop poles for several gains K = [k1, k2]:")
    for K in [np.array([[0.0, 0.0]]),
              np.array([[3.0, 0.0]]),
              np.array([[8.0, 100.0]]),
              np.array([[-1.0, -50.0]])]:
        Acl = A - B @ K
        print(f"  K = {K.tolist()} -> eig(A-BK) = {np.sort_complex(eigvals(Acl))}")

    print("\nAttempting full pole placement at {-4, -5}:")
    try:
        result = place_poles(A, B, [-4.0, -5.0])
        print("K =", result.gain_matrix)
        print("eig(A-BK) =", eigvals(A - B @ result.gain_matrix))
    except Exception as exc:
        print("Pole placement failed because the pair (A,B) is not controllable.")
        print("Reason:", exc)

    print("\nReachable-basis decomposition:")
    T, Abar, Bbar, r = kalman_decomposition_by_reachable_basis(A, B)
    print("reachable dimension r =", r)
    print("T =\n", T)
    print("Abar = T^T A T =\n", Abar)
    print("Bbar = T^T B =\n", Bbar)
    print("The uncontrollable block contains the unassignable eigenvalue +2.")


if __name__ == "__main__":
    main()
