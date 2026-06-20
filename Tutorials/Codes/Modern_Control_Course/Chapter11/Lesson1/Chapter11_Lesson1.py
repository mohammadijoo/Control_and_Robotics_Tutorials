# Chapter11_Lesson1.py
# Kalman controllability matrix and rank condition for continuous-time LTI systems.
# Requirements: numpy. Optional: scipy is not required.

import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C_K = [B, AB, A^2 B, ..., A^(n-1)B]."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be an n by n square matrix.")
    if B.ndim != 2 or B.shape[0] != A.shape[0]:
        raise ValueError("B must be an n by m matrix with the same row count as A.")

    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(blocks)


def numerical_rank(M: np.ndarray, rtol: float = None) -> tuple[int, np.ndarray, float]:
    """Compute SVD-based numerical rank and return rank, singular values, tolerance."""
    s = np.linalg.svd(M, compute_uv=False)
    if rtol is None:
        rtol = max(M.shape) * np.finfo(float).eps
    tol = rtol * (s[0] if s.size else 0.0)
    rank = int(np.sum(s > tol))
    return rank, s, tol


def is_controllable(A: np.ndarray, B: np.ndarray, rtol: float = None) -> bool:
    Ck = controllability_matrix(A, B)
    rank, _, _ = numerical_rank(Ck, rtol=rtol)
    return rank == A.shape[0]


def reachable_subspace_basis(A: np.ndarray, B: np.ndarray, rtol: float = None) -> np.ndarray:
    """Return an orthonormal basis for the reachable subspace using SVD."""
    Ck = controllability_matrix(A, B)
    U, s, _ = np.linalg.svd(Ck, full_matrices=False)
    if rtol is None:
        rtol = max(Ck.shape) * np.finfo(float).eps
    tol = rtol * (s[0] if s.size else 0.0)
    r = int(np.sum(s > tol))
    return U[:, :r]


def demo() -> None:
    # Example 1: third-order companion-like system with one input.
    A1 = np.array([[0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [-6.0, -11.0, -6.0]])
    B1 = np.array([[0.0],
                   [0.0],
                   [1.0]])

    C1 = controllability_matrix(A1, B1)
    rank1, s1, tol1 = numerical_rank(C1)
    print("Example 1: controllable SISO system")
    print("C_K =\n", C1)
    print("singular values =", s1)
    print("rank =", rank1, "tol =", tol1)
    print("controllable =", rank1 == A1.shape[0])
    print()

    # Example 2: a system with an unactuated first state.
    A2 = np.array([[2.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, -4.0, -1.0]])
    B2 = np.array([[0.0],
                   [0.0],
                   [1.0]])

    C2 = controllability_matrix(A2, B2)
    rank2, s2, tol2 = numerical_rank(C2)
    print("Example 2: uncontrollable system")
    print("C_K =\n", C2)
    print("singular values =", s2)
    print("rank =", rank2, "tol =", tol2)
    print("controllable =", rank2 == A2.shape[0])
    print("reachable basis =\n", reachable_subspace_basis(A2, B2))


if __name__ == "__main__":
    demo()
