# Chapter16_Lesson1.py
"""
Companion Matrix and Controllable Canonical Form (CCF)

Convention:
    denominator: p(s) = s^n + a_{n-1}s^{n-1} + ... + a_1 s + a_0
    pass a = [a0, a1, ..., a_{n-1}]

    strictly proper numerator:
    q(s) = b_0 + b_1 s + ... + b_{n-1}s^{n-1}
    pass b = [b0, b1, ..., b_{n-1}]

CCF:
    A = [[0, 1, 0, ..., 0],
         ...
         [0, 0, 0, ..., 1],
         [-a0, -a1, ..., -a_{n-1}]]
    B = [0, ..., 0, 1]^T
    C = [b0, b1, ..., b_{n-1}]
    D = direct feedthrough
"""

import numpy as np


def controllable_canonical_form(a, b, d=0.0):
    """Return A, B, C, D for controller companion / CCF realization."""
    a = np.asarray(a, dtype=float).reshape(-1)
    b = np.asarray(b, dtype=float).reshape(-1)
    n = len(a)

    if len(b) != n:
        raise ValueError("For this strictly proper CCF example, b must have length n.")

    A = np.zeros((n, n), dtype=float)
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -a

    B = np.zeros((n, 1), dtype=float)
    B[-1, 0] = 1.0

    C = b.reshape(1, n)
    D = np.array([[float(d)]])
    return A, B, C, D


def controllability_matrix(A, B):
    """Q_c = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    cols = []
    v = B.copy()
    for _ in range(n):
        cols.append(v)
        v = A @ v
    return np.hstack(cols)


def evaluate_transfer(A, B, C, D, s):
    """Evaluate G(s)=C(sI-A)^(-1)B+D at one complex value s."""
    n = A.shape[0]
    return (C @ np.linalg.solve(s * np.eye(n) - A, B) + D)[0, 0]


def polynomial_value_ascending(coeffs, s):
    """Evaluate coeffs[0] + coeffs[1]s + ... in ascending-power order."""
    value = 0.0 + 0.0j
    power = 1.0 + 0.0j
    for c in coeffs:
        value += c * power
        power *= s
    return value


if __name__ == "__main__":
    # Example: G(s) = (5 + 6s + 7s^2) / (s^3 + 4s^2 + 3s + 2)
    a = [2.0, 3.0, 4.0]  # a0, a1, a2
    b = [5.0, 6.0, 7.0]  # b0, b1, b2

    A, B, C, D = controllable_canonical_form(a, b)
    Qc = controllability_matrix(A, B)

    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("D =\n", D)
    print("Q_c =\n", Qc)
    print("rank(Q_c) =", np.linalg.matrix_rank(Qc))
    print("det(Q_c)  =", np.linalg.det(Qc))

    s = 1.0 + 2.0j
    G_state = evaluate_transfer(A, B, C, D, s)
    denominator = s ** len(a) + polynomial_value_ascending(a, s)
    numerator = polynomial_value_ascending(b, s)
    G_poly = numerator / denominator

    print("G(s) from state-space =", G_state)
    print("G(s) from polynomials =", G_poly)
    print("absolute error =", abs(G_state - G_poly))
