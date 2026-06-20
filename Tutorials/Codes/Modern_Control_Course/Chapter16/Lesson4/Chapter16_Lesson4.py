#!/usr/bin/env python3
"""
Chapter16_Lesson4.py

Example systems in controllable canonical form (CCF) for continuous-time SISO
state-space systems.

The convention used here is

    d(s) = s^n + a_{n-1}s^{n-1} + ... + a_1 s + a_0
    n(s) = b_{n-1}s^{n-1} + ... + b_1 s + b_0

with

    A = [[0, 1, 0, ..., 0],
         [0, 0, 1, ..., 0],
         ...
         [-a0, -a1, ..., -a_{n-1}]]

    B = [0, 0, ..., 1]^T
    C = [b0, b1, ..., b_{n-1}]
    D = 0

so that G(s) = C(sI-A)^(-1)B = n(s)/d(s).
"""

from __future__ import annotations

import numpy as np


def controllable_canonical_form(den_desc: list[float], num_desc: list[float]):
    """Return A, B, C, D for a strictly proper SISO transfer function."""
    den = np.asarray(den_desc, dtype=float)
    num = np.asarray(num_desc, dtype=float)

    if den.ndim != 1 or num.ndim != 1:
        raise ValueError("den_desc and num_desc must be one-dimensional lists.")
    if abs(den[0]) < 1e-14:
        raise ValueError("Leading denominator coefficient must be nonzero.")

    num = num / den[0]
    den = den / den[0]
    n = len(den) - 1

    if len(num) > n:
        raise ValueError("This script handles strictly proper transfer functions only.")

    num_padded_desc = np.pad(num, (n - len(num), 0), mode="constant")

    A = np.zeros((n, n))
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -den[1:][::-1]

    B = np.zeros((n, 1))
    B[-1, 0] = 1.0

    C = num_padded_desc[::-1].reshape(1, n)
    D = np.array([[0.0]])
    return A, B, C, D


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Compute [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    cols = []
    Ak = np.eye(n)
    for _ in range(n):
        cols.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(cols)


def eval_transfer(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray, s: complex):
    """Evaluate G(s) = C(sI-A)^(-1)B + D at a scalar complex value s."""
    n = A.shape[0]
    return (C @ np.linalg.solve(s * np.eye(n) - A, B) + D)[0, 0]


def print_system(name: str, den_desc: list[float], num_desc: list[float]):
    A, B, C, D = controllable_canonical_form(den_desc, num_desc)
    Wc = controllability_matrix(A, B)

    print("\n" + "=" * 72)
    print(name)
    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("D =\n", D)
    print("Controllability matrix Wc =\n", Wc)
    print("rank(Wc) =", np.linalg.matrix_rank(Wc))
    print("det(Wc)  =", np.linalg.det(Wc))

    test_points = [0.0, -0.5, 1.0 + 0.2j]
    for z in test_points:
        g_ss = eval_transfer(A, B, C, D, z)
        den_val = np.polyval(den_desc, z)
        num_val = np.polyval(num_desc, z)
        g_tf = num_val / den_val
        print(f"s={z!s:>10}: state-space={g_ss: .8f}, transfer-function={g_tf: .8f}")


if __name__ == "__main__":
    print_system("Example 1: G1(s) = 2/(s^2 + 3s + 2)",
                 den_desc=[1, 3, 2],
                 num_desc=[2])

    print_system("Example 2: G2(s) = (s + 2)/(s^3 + 6s^2 + 11s + 6)",
                 den_desc=[1, 6, 11, 6],
                 num_desc=[1, 2])

    print_system("Example 3: G3(s) = (0.5s^2 + 1.5s + 1)/(s^3 + 4s^2 + 5s + 2)",
                 den_desc=[1, 4, 5, 2],
                 num_desc=[0.5, 1.5, 1.0])
