"""
Chapter11_Lesson3.py

Controllability in companion (controllable canonical) form.

Convention:
    p(s) = s^n + a_{n-1}s^{n-1} + ... + a_1 s + a_0

    A_c = [[0, 1, 0, ..., 0],
           [0, 0, 1, ..., 0],
           ...
           [0, 0, 0, ..., 1],
           [-a_0, -a_1, ..., -a_{n-1}]]

    B_c = [0, 0, ..., 1]^T
"""

from __future__ import annotations

import numpy as np


def companion_pair(coefficients: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """Build the SISO companion controllable canonical pair (A_c, B_c).

    coefficients = [a0, a1, ..., a_{n-1}]
    """
    n = len(coefficients)
    if n == 0:
        raise ValueError("At least one coefficient is required.")

    A = np.zeros((n, n), dtype=float)
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -np.asarray(coefficients, dtype=float)

    B = np.zeros((n, 1), dtype=float)
    B[-1, 0] = 1.0
    return A, B


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return Q = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def kalman_rank(A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> int:
    """Numerical rank of the Kalman controllability matrix."""
    Q = controllability_matrix(A, B)
    return int(np.linalg.matrix_rank(Q, tol=tol))


def pbh_test(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> bool:
    """PBH controllability test: rank([lambda I - A, B]) = n for every eigenvalue."""
    n = A.shape[0]
    eigvals = np.linalg.eigvals(A)
    for lam in eigvals:
        M = np.hstack((lam * np.eye(n) - A, B))
        if np.linalg.matrix_rank(M, tol=tol) < n:
            return False
    return True


def controllable_to_companion_transform(A: np.ndarray, B: np.ndarray, coefficients: list[float]) -> np.ndarray:
    """Return T such that x = T z, A_c = T^{-1} A T, B_c = T^{-1} B.

    For a controllable SISO pair with the same characteristic polynomial
    coefficients as the companion pair:
        T = Q(A,B) Q(A_c,B_c)^{-1}
    """
    Ac, Bc = companion_pair(coefficients)
    Q = controllability_matrix(A, B)
    Qc = controllability_matrix(Ac, Bc)
    if np.linalg.matrix_rank(Q) < A.shape[0]:
        raise ValueError("The supplied pair (A,B) is not controllable.")
    return Q @ np.linalg.inv(Qc)


def main() -> None:
    coefficients = [6.0, 11.0, 6.0]  # p(s)=s^3+6s^2+11s+6
    Ac, Bc = companion_pair(coefficients)
    Qc = controllability_matrix(Ac, Bc)

    print("A_c =\n", Ac)
    print("B_c =\n", Bc)
    print("Q_c = [B, AB, A^2B] =\n", Qc)
    print("det(Q_c) =", np.linalg.det(Qc))
    print("rank(Q_c) =", kalman_rank(Ac, Bc))
    print("PBH controllable?", pbh_test(Ac, Bc))

    # Example: a similarity transform preserves controllability.
    S = np.array([[1.0, 1.0, 0.0], [0.0, 1.0, 1.0], [1.0, 0.0, 1.0]])
    A = S @ Ac @ np.linalg.inv(S)
    B = S @ Bc
    print("\nTransformed realization rank =", kalman_rank(A, B))

    T = controllable_to_companion_transform(A, B, coefficients)
    recovered_Ac = np.linalg.inv(T) @ A @ T
    recovered_Bc = np.linalg.inv(T) @ B
    print("\nRecovered A_c =\n", np.round(recovered_Ac, 10))
    print("Recovered B_c =\n", np.round(recovered_Bc, 10))


if __name__ == "__main__":
    main()
