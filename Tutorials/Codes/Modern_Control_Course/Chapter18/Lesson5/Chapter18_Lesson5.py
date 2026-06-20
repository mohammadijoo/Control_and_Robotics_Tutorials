"""
Chapter18_Lesson5.py
Interpretation of Jordan form in control applications.

Required packages:
    numpy
    scipy

This script demonstrates:
1. matrix exponentials of Jordan blocks,
2. polynomial-times-exponential response terms,
3. controllability/observability rank tests in Jordan coordinates,
4. how input/output placement along a Jordan chain affects control authority and sensing.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.linalg import expm


def jordan_block(lam: float, size: int) -> np.ndarray:
    """Return one Jordan block J = lam*I + N with ones on the superdiagonal."""
    if size <= 0:
        raise ValueError("size must be positive")
    J = lam * np.eye(size)
    for i in range(size - 1):
        J[i, i + 1] = 1.0
    return J


def jordan_block_exponential(lam: float, size: int, t: float) -> np.ndarray:
    """Closed-form exp(J t) for one Jordan block."""
    E = np.zeros((size, size))
    scale = math.exp(lam * t)
    for i in range(size):
        for j in range(i, size):
            power = j - i
            E[i, j] = scale * (t ** power) / math.factorial(power)
    return E


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Construct Ctrb = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Construct Obsv = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def numerical_rank(M: np.ndarray, tol: float = 1e-10) -> int:
    """Numerical rank using singular values."""
    return int(np.linalg.matrix_rank(M, tol=tol))


def print_modal_response(A: np.ndarray, x0: Iterable[float], times: Iterable[float]) -> None:
    x0_vec = np.asarray(list(x0), dtype=float).reshape((-1, 1))
    print("\nFree response x(t) = exp(A t) x0")
    for t in times:
        xt = expm(A * t) @ x0_vec
        print(f"t={t:4.1f}: x(t)^T = {xt.ravel()}")


def main() -> None:
    # One defective eigenvalue: a length-3 Jordan chain at lambda = -1.
    lam = -1.0
    A = jordan_block(lam, 3)
    print("A = Jordan block J_3(-1):")
    print(A)

    t = 2.0
    closed = jordan_block_exponential(lam, 3, t)
    numeric = expm(A * t)
    print("\nClosed-form exp(A t) at t=2:")
    print(closed)
    print("\nSciPy expm(A t) at t=2:")
    print(numeric)
    print("\nMax absolute difference:", np.max(np.abs(closed - numeric)))

    # Control authority depends on where the input enters the chain.
    B_good = np.array([[0.0], [0.0], [1.0]])  # acts on tail of chain, reaches all generalized directions
    B_bad = np.array([[1.0], [0.0], [0.0]])   # acts only on head direction

    for name, B in [("B_good", B_good), ("B_bad", B_bad)]:
        Wc = controllability_matrix(A, B)
        print(f"\n{name} =\n{B}")
        print("Controllability matrix =")
        print(Wc)
        print("rank =", numerical_rank(Wc), "of", A.shape[0])

    # Sensing also depends on output placement along the chain.
    C_good = np.array([[1.0, 0.0, 0.0]])  # measures head; chain derivatives reveal all components
    C_bad = np.array([[0.0, 0.0, 1.0]])   # measures tail only

    for name, C in [("C_good", C_good), ("C_bad", C_bad)]:
        Wo = observability_matrix(A, C)
        print(f"\n{name} = {C}")
        print("Observability matrix =")
        print(Wo)
        print("rank =", numerical_rank(Wo), "of", A.shape[0])

    print_modal_response(A, x0=[0.0, 0.0, 1.0], times=[0, 0.5, 1, 2, 4, 6])


if __name__ == "__main__":
    main()
