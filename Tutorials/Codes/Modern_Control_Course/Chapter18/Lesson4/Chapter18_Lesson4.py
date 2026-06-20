"""
Chapter18_Lesson4.py
Repeated eigenvalues and non-diagonalizable systems.

This script studies a 3 x 3 Jordan block
    J = lambda I + N,
where N has ones on the superdiagonal and N^3 = 0.
It compares the closed-form Jordan exponential with scipy.linalg.expm
when SciPy is available.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def nilpotent_shift(size: int) -> np.ndarray:
    """Return the nilpotent shift matrix with ones on the superdiagonal."""
    if size <= 0:
        raise ValueError("size must be positive")
    n_mat = np.zeros((size, size), dtype=float)
    for i in range(size - 1):
        n_mat[i, i + 1] = 1.0
    return n_mat


def jordan_block(lambda_value: float, size: int) -> np.ndarray:
    """Return J_size(lambda) = lambda I + N."""
    return lambda_value * np.eye(size) + nilpotent_shift(size)


def exp_jordan_block(lambda_value: float, size: int, t: float) -> np.ndarray:
    """
    Compute exp(J t) exactly for a Jordan block.

    exp((lambda I + N)t) = exp(lambda t) sum_{k=0}^{m-1} (t^k/k!) N^k.
    """
    n_mat = nilpotent_shift(size)
    result = np.eye(size)
    power = np.eye(size)
    for k in range(1, size):
        power = power @ n_mat
        result = result + (t ** k / math.factorial(k)) * power
    return math.exp(lambda_value * t) * result


def geometric_multiplicity(a_mat: np.ndarray, lambda_value: float, tol: float = 1e-10) -> int:
    """Estimate dim ker(A - lambda I) from singular values."""
    shifted = a_mat - lambda_value * np.eye(a_mat.shape[0])
    s_vals = np.linalg.svd(shifted, compute_uv=False)
    rank = int(np.sum(s_vals > tol))
    return a_mat.shape[0] - rank


def simulate(lambda_value: float = -0.4, size: int = 3, x0: Iterable[float] = (1.0, -2.0, 1.5)) -> None:
    """Print a response table and a diagnostic comparison with scipy.linalg.expm."""
    a_mat = jordan_block(lambda_value, size)
    x0_vec = np.asarray(list(x0), dtype=float)

    print("A =")
    print(a_mat)
    print(f"algebraic multiplicity of lambda={lambda_value}: {size}")
    print(f"estimated geometric multiplicity: {geometric_multiplicity(a_mat, lambda_value)}")
    print()

    try:
        from scipy.linalg import expm
    except Exception:
        expm = None

    print("t        x1(t)        x2(t)        x3(t)        max_abs_error_vs_expm")
    for t in np.linspace(0.0, 8.0, 9):
        phi_closed = exp_jordan_block(lambda_value, size, float(t))
        x_closed = phi_closed @ x0_vec

        if expm is not None:
            x_scipy = expm(a_mat * t) @ x0_vec
            err = np.max(np.abs(x_closed - x_scipy))
        else:
            err = float("nan")

        print(f"{t:4.1f}  {x_closed[0]:11.6f} {x_closed[1]:11.6f} {x_closed[2]:11.6f}   {err:12.3e}")


if __name__ == "__main__":
    simulate()
