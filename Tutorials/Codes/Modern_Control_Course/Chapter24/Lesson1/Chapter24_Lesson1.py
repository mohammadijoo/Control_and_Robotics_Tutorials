"""
Chapter24_Lesson1.py
Existence conditions for MIMO pole assignment.

Requires:
    numpy
Optional:
    scipy (for scipy.signal.place_poles)

The script verifies Kalman controllability, PBH controllability, stabilizability,
and then performs a MIMO pole assignment for a controllable pair.
"""

import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Apow = np.eye(n)
    for _ in range(n):
        blocks.append(Apow @ B)
        Apow = A @ Apow
    return np.hstack(blocks)


def matrix_rank(M: np.ndarray, tol: float = 1e-10) -> int:
    """Numerical rank by singular values."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def is_controllable(A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> bool:
    """Kalman rank test."""
    return matrix_rank(controllability_matrix(A, B), tol) == A.shape[0]


def pbh_rank(A: np.ndarray, B: np.ndarray, lam: complex, tol: float = 1e-10) -> int:
    """Return rank([lam*I - A, B]) for a possibly complex lambda."""
    n = A.shape[0]
    M = np.hstack((lam * np.eye(n, dtype=complex) - A.astype(complex), B.astype(complex)))
    return matrix_rank(M, tol)


def pbh_controllable(A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> bool:
    """PBH controllability test at eigenvalues of A."""
    n = A.shape[0]
    eigs = np.linalg.eigvals(A)
    return all(pbh_rank(A, B, lam, tol) == n for lam in eigs)


def is_stabilizable(A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> bool:
    """Continuous-time stabilizability: PBH rank condition for Re(lambda) >= 0."""
    n = A.shape[0]
    eigs = np.linalg.eigvals(A)
    for lam in eigs:
        if np.real(lam) >= -tol and pbh_rank(A, B, lam, tol) < n:
            return False
    return True


def main() -> None:
    A = np.array([
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [-1.0, -5.0, -6.0],
    ])
    B = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
    ])

    print("Controllability matrix rank:", matrix_rank(controllability_matrix(A, B)))
    print("Kalman controllable:", is_controllable(A, B))
    print("PBH controllable:", pbh_controllable(A, B))
    print("Stabilizable:", is_stabilizable(A, B))

    desired_poles = np.array([-2.0, -3.0, -4.0])

    try:
        from scipy.signal import place_poles
        result = place_poles(A, B, desired_poles, method="YT")
        K = result.gain_matrix
        print("K from scipy.signal.place_poles:")
        print(K)
    except Exception as exc:
        print("SciPy place_poles unavailable; using a known valid K for this example.")
        print("Reason:", exc)
        K = np.array([
            [8.0, 6.0, 1.0],
            [-1.0, -5.0, -3.0],
        ])

    Acl = A - B @ K
    print("Closed-loop eigenvalues:", np.linalg.eigvals(Acl))

    # Example of a non-controllable pair: the first state is not actuated and is unstable.
    A_bad = np.diag([1.0, -2.0, -3.0])
    B_bad = np.array([
        [0.0],
        [1.0],
        [1.0],
    ])
    print("\nNon-controllable example rank:", matrix_rank(controllability_matrix(A_bad, B_bad)))
    print("Non-controllable example stabilizable:", is_stabilizable(A_bad, B_bad))
    print("The unstable uncontrollable pole at +1 cannot be moved by state feedback.")


if __name__ == "__main__":
    main()
