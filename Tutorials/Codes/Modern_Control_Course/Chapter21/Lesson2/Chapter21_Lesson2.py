"""
Chapter21_Lesson2.py
Computation of transmission / invariant zeros from a continuous-time
state-space realization (A, B, C, D).

Requires:
    numpy
    scipy

Main idea:
    For a square system (same number of inputs and outputs), the finite
    invariant zeros are generalized eigenvalues of the Rosenbrock pencil

        s E - M = [[sI - A, -B], [C, D]],
        E = [[I, 0], [0, 0]],  M = [[A, B], [-C, -D]].

    For nonsingular D, the zeros can also be computed as eigenvalues of
        A - B D^{-1} C.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import eig, svdvals


def as_matrix(x: np.ndarray) -> np.ndarray:
    """Convert an input array to a 2D float matrix."""
    arr = np.asarray(x, dtype=float)
    if arr.ndim != 2:
        raise ValueError("Input must be a two-dimensional matrix.")
    return arr


def rosenbrock_pencil(A, B, C, D):
    """Return the matrices (M, E) for the Rosenbrock generalized eigenproblem."""
    A, B, C, D = map(as_matrix, (A, B, C, D))
    n = A.shape[0]
    p, nC = C.shape
    nB, m = B.shape
    if A.shape != (n, n) or nB != n or nC != n or D.shape != (p, m):
        raise ValueError("Inconsistent dimensions among A, B, C, D.")

    E = np.block([
        [np.eye(n), np.zeros((n, m))],
        [np.zeros((p, n)), np.zeros((p, m))],
    ])
    M = np.block([
        [A, B],
        [-C, -D],
    ])
    return M, E


def normal_rank_at(A, B, C, D, s: complex, tol: float = 1e-9) -> int:
    """Numerically estimate rank of the Rosenbrock matrix at s."""
    A, B, C, D = map(as_matrix, (A, B, C, D))
    n = A.shape[0]
    R = np.block([
        [s * np.eye(n) - A, -B],
        [C, D],
    ]).astype(complex)
    sigma = svdvals(R)
    return int(np.sum(sigma > tol * max(R.shape) * sigma[0]))


def finite_invariant_zeros_square(A, B, C, D, tol: float = 1e-9):
    """
    Compute finite invariant zeros for a square continuous-time system.

    This routine assumes p == m. Rectangular MIMO systems require staircase or
    Kronecker-form algorithms to separate finite zeros from structural indices.
    """
    A, B, C, D = map(as_matrix, (A, B, C, D))
    p, m = D.shape
    if p != m:
        raise ValueError("This generalized-eigenvalue implementation requires p == m.")

    M, E = rosenbrock_pencil(A, B, C, D)
    alpha, beta = eig(M, E, homogeneous_eigvals=True)
    zeros = []
    for a, b in zip(alpha, beta):
        if abs(b) > tol:
            z = a / b
            if np.isfinite(z.real) and np.isfinite(z.imag):
                zeros.append(z)
    return np.array(zeros, dtype=complex)


def zeros_when_D_is_invertible(A, B, C, D):
    """Compute zeros as eigenvalues of A - B D^{-1} C when D is nonsingular."""
    A, B, C, D = map(as_matrix, (A, B, C, D))
    p, m = D.shape
    if p != m:
        raise ValueError("D must be square.")
    if np.linalg.matrix_rank(D) < m:
        raise ValueError("D is singular; use the Rosenbrock pencil instead.")
    Az = A - B @ np.linalg.solve(D, C)
    return np.linalg.eigvals(Az)


if __name__ == "__main__":
    # Minimal SISO example with D = 0.
    # G(s) = (s + 4)/(s^2 + 3s + 2), so the finite zero is -4.
    A = np.array([[0.0, 1.0], [-2.0, -3.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[4.0, 1.0]])
    D = np.array([[0.0]])

    z = finite_invariant_zeros_square(A, B, C, D)
    print("Finite invariant zeros from Rosenbrock pencil:", z)

    for candidate in [-4.0, -2.0, -1.0, 0.0]:
        print(f"rank R({candidate}) =", normal_rank_at(A, B, C, D, candidate))

    # Example with nonsingular D.
    D2 = np.array([[1.0]])
    print("Zeros when D is invertible:", zeros_when_D_is_invertible(A, B, C, D2))
