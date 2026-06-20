"""
Chapter20_Lesson2.py

Exact minimal-realization reduction for continuous-time LTI systems.

The script demonstrates the two exact reductions discussed in
Chapter 20, Lesson 2:
1. remove unreachable states;
2. remove unobservable states from the reachable realization.

Dependencies:
    numpy
    scipy

Run:
    python Chapter20_Lesson2.py
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import null_space, orth
from scipy.signal import ss2tf


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return Wc = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return Wo = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = [C]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(C @ Ak)
    return np.vstack(blocks)


def matrix_rank(M: np.ndarray, tol: float = 1e-10) -> int:
    """Numerical rank via singular values."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def complete_basis(Q: np.ndarray, n: int, tol: float = 1e-10) -> np.ndarray:
    """
    Complete an orthonormal basis Q to an n-by-n nonsingular matrix.
    Q is n-by-r with orthonormal columns.
    """
    if Q.size == 0:
        Q = np.zeros((n, 0))
    candidates = [Q]
    current = Q.copy()
    for j in range(n):
        e = np.zeros((n, 1))
        e[j, 0] = 1.0
        trial = np.hstack([current, e])
        if matrix_rank(trial, tol) > matrix_rank(current, tol):
            candidates.append(e)
            current = trial
        if current.shape[1] == n:
            break
    return np.hstack(candidates)


def reachable_reduction(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, tol: float = 1e-10):
    """
    Remove unreachable states by using a basis for Im(Wc).

    Returns (Ar, Br, Cr, D, T, r) where r is reachable dimension.
    """
    n = A.shape[0]
    Wc = controllability_matrix(A, B)
    Qr = orth(Wc, rcond=tol)
    r = Qr.shape[1]
    T = complete_basis(Qr, n, tol)
    Ti = np.linalg.inv(T)

    Abar = Ti @ A @ T
    Bbar = Ti @ B
    Cbar = C @ T

    Ar = Abar[:r, :r]
    Br = Bbar[:r, :]
    Cr = Cbar[:, :r]
    return Ar, Br, Cr, D.copy(), T, r


def observable_reduction(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, tol: float = 1e-10):
    """
    Remove unobservable states.

    The unobservable subspace is Ker(Wo). We put a complement first and
    Ker(Wo) last, then keep the observable block.
    """
    n = A.shape[0]
    Wo = observability_matrix(A, C)
    N = null_space(Wo, rcond=tol)          # n-by-k
    k = N.shape[1]
    q = n - k                              # observable dimension

    # Complement of Ker(Wo) is the row space of Wo, i.e., column space of Wo.T.
    Qo = orth(Wo.T, rcond=tol)             # n-by-q
    T = np.hstack([Qo, N]) if k > 0 else Qo
    Ti = np.linalg.inv(T)

    Abar = Ti @ A @ T
    Bbar = Ti @ B
    Cbar = C @ T

    Ao = Abar[:q, :q]
    Bo = Bbar[:q, :]
    Co = Cbar[:, :q]
    return Ao, Bo, Co, D.copy(), T, q


def minimal_realization(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, tol: float = 1e-10):
    """Perform reachable reduction followed by observable reduction."""
    Ar, Br, Cr, Dr, Tr, r = reachable_reduction(A, B, C, D, tol)
    Am, Bm, Cm, Dm, To, q = observable_reduction(Ar, Br, Cr, Dr, tol)
    return Am, Bm, Cm, Dm, r, q


def print_transfer(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, label: str):
    """Print numerator/denominator of each SISO channel when the system is SISO."""
    num, den = ss2tf(A, B, C, D)
    print(f"\n{label}")
    print("numerator:", np.array2string(num, precision=6, suppress_small=True))
    print("denominator:", np.array2string(den, precision=6, suppress_small=True))


def main() -> None:
    # Nonminimal example:
    # x1 is reachable and observable: contributes 1/(s+1)
    # x2 is unreachable but visible from initial condition: no zero-state transfer contribution
    # x3 is reachable but unobservable: no output contribution
    A = np.diag([-1.0, -2.0, -3.0])
    B = np.array([[1.0], [0.0], [1.0]])
    C = np.array([[1.0, 1.0, 0.0]])
    D = np.array([[0.0]])

    Wc = controllability_matrix(A, B)
    Wo = observability_matrix(A, C)

    print("rank(Wc) =", matrix_rank(Wc), "out of n =", A.shape[0])
    print("rank(Wo) =", matrix_rank(Wo), "out of n =", A.shape[0])
    print_transfer(A, B, C, D, "Original nonminimal realization")

    Am, Bm, Cm, Dm, r, q = minimal_realization(A, B, C, D)

    print("\nReachable dimension after first reduction:", r)
    print("Minimal dimension after second reduction:", q)
    print("\nAm =\n", Am)
    print("Bm =\n", Bm)
    print("Cm =\n", Cm)
    print("Dm =\n", Dm)

    print_transfer(Am, Bm, Cm, Dm, "Minimal realization")

    # Frequency-domain check at representative complex values.
    for s in [0.0, 1.0, 2.0 + 1.0j]:
        G_full = C @ np.linalg.inv(s * np.eye(3) - A) @ B + D
        G_min = Cm @ np.linalg.inv(s * np.eye(Am.shape[0]) - Am) @ Bm + Dm
        print(f"s={s:>6}: G_full={G_full[0,0]: .6g}, G_min={G_min[0,0]: .6g}")


if __name__ == "__main__":
    main()
