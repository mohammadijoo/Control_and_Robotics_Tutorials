# Chapter20_Lesson4.py
"""
Basic algorithms for exact realization reduction.

This educational script demonstrates the two-stage reduction:
1. restrict the realization to the reachable subspace;
2. quotient out the unobservable subspace of the reachable realization.

Required packages:
    pip install numpy scipy
For production control work, compare the result with python-control:
    pip install control
"""

from __future__ import annotations
import numpy as np
from numpy.linalg import svd


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C_R = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Apow = np.eye(n)
    for _ in range(n):
        blocks.append(Apow @ B)
        Apow = Apow @ A
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Apow = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Apow)
        Apow = Apow @ A
    return np.vstack(blocks)


def numerical_rank(M: np.ndarray, tol: float | None = 1e-10) -> int:
    s = svd(M, compute_uv=False)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (s[0] if s.size else 1.0)
    return int(np.sum(s > tol))


def column_space_basis(M: np.ndarray, tol: float | None = 1e-10) -> np.ndarray:
    """Orthonormal basis for range(M), returned as columns."""
    U, s, _ = svd(M, full_matrices=True)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (s[0] if s.size else 1.0)
    r = int(np.sum(s > tol))
    return U[:, :r]


def null_space_basis(M: np.ndarray, tol: float | None = 1e-10) -> np.ndarray:
    """Orthonormal basis for null(M), returned as columns."""
    _, s, Vt = svd(M, full_matrices=True)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (s[0] if s.size else 1.0)
    r = int(np.sum(s > tol))
    return Vt.T[:, r:]


def exact_minimal_reduction(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
    """
    Return a minimal realization externally equivalent to (A,B,C,D),
    assuming exact rank decisions are reliable for the given data.
    """
    n = A.shape[0]

    # Stage 1: keep only the reachable subspace.
    R = controllability_matrix(A, B)
    Qr = column_space_basis(R)
    A1 = Qr.T @ A @ Qr
    B1 = Qr.T @ B
    C1 = C @ Qr

    # Stage 2: remove the unobservable part of the reachable realization.
    O1 = observability_matrix(A1, C1)
    Q_unobs = null_space_basis(O1)
    q = Q_unobs.shape[1]
    if q == 0:
        return A1, B1, C1, D, Qr, np.eye(A1.shape[0])

    # Orthogonal complement completes the coordinate basis.
    Q_obs = null_space_basis(Q_unobs.T)
    T = np.hstack([Q_unobs, Q_obs])

    Ahat = T.T @ A1 @ T
    Bhat = T.T @ B1
    Chat = C1 @ T

    Amin = Ahat[q:, q:]
    Bmin = Bhat[q:, :]
    Cmin = Chat[:, q:]
    return Amin, Bmin, Cmin, D, Qr, T


def transfer_value(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, s: complex) -> np.ndarray:
    """Evaluate G(s)=C(sI-A)^(-1)B+D."""
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def main() -> None:
    # Nonminimal realization: states 2 and 3 do not affect the input-output map.
    A = np.diag([-1.0, -2.0, -3.0])
    B = np.array([[1.0], [0.0], [1.0]])
    C = np.array([[1.0, 0.0, 0.0]])
    D = np.array([[0.0]])

    R = controllability_matrix(A, B)
    O = observability_matrix(A, C)
    print("rank controllability matrix:", numerical_rank(R), "of", A.shape[0])
    print("rank observability matrix:", numerical_rank(O), "of", A.shape[0])

    Amin, Bmin, Cmin, Dmin, _, _ = exact_minimal_reduction(A, B, C, D)
    print("Amin =\n", Amin)
    print("Bmin =\n", Bmin)
    print("Cmin =\n", Cmin)
    print("Dmin =\n", Dmin)

    for s in [0.2, 1.0, 2.5]:
        g_old = transfer_value(A, B, C, D, s)
        g_new = transfer_value(Amin, Bmin, Cmin, Dmin, s)
        print(f"s={s:3.1f}: original {g_old.ravel()[0]: .8f}, reduced {g_new.ravel()[0]: .8f}")


if __name__ == "__main__":
    main()
