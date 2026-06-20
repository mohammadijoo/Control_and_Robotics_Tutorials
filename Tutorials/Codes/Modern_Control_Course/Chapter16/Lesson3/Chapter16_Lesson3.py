"""
Chapter16_Lesson3.py

Properties of Controllable Canonical Form (CCF) for analysis and design.

This script constructs a SISO controllable canonical form realization

    D(s) = s^n + a[n-1] s^(n-1) + ... + a[1] s + a[0]
    N(s) = b[n-1] s^(n-1) + ... + b[1] s + b[0]

and demonstrates:
1. companion/CCF matrix construction,
2. controllability matrix determinant and rank,
3. direct pole placement by coefficient matching.

Required library:
    numpy

Optional libraries for larger work:
    scipy.signal, python-control, slycot
"""

import numpy as np


def ccf_matrices(a, b):
    """Return A, B, C, D=0 for CCF using ascending coefficients.

    Parameters
    ----------
    a : array_like, shape (n,)
        Denominator coefficients [a0, a1, ..., a_{n-1}].
    b : array_like, shape (n,)
        Strictly proper numerator coefficients [b0, b1, ..., b_{n-1}].
        Pad with zeros if the numerator degree is lower than n-1.
    """
    a = np.asarray(a, dtype=float).ravel()
    b = np.asarray(b, dtype=float).ravel()
    n = len(a)
    if len(b) != n:
        raise ValueError("b must have the same length as a; pad numerator with zeros.")

    A = np.zeros((n, n), dtype=float)
    A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -a

    B = np.zeros((n, 1), dtype=float)
    B[-1, 0] = 1.0

    C = b.reshape(1, -1)
    D = np.array([[0.0]])
    return A, B, C, D


def controllability_matrix(A, B):
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    cols = []
    v = B.copy()
    for _ in range(n):
        cols.append(v)
        v = A @ v
    return np.hstack(cols)


def pole_placement_gain_ccf(a, desired_alpha):
    """Coefficient-matching feedback K for u = -K x + r.

    If D_des(s) = s^n + alpha[n-1]s^(n-1)+...+alpha[1]s+alpha[0],
    then K = [alpha0-a0, alpha1-a1, ..., alpha[n-1]-a[n-1]].
    """
    a = np.asarray(a, dtype=float).ravel()
    desired_alpha = np.asarray(desired_alpha, dtype=float).ravel()
    if desired_alpha.shape != a.shape:
        raise ValueError("desired_alpha must have length n.")
    return desired_alpha - a


def polynomial_from_roots_ascending(roots):
    """Return [alpha0, alpha1, ..., alpha_{n-1}] for monic polynomial."""
    descending = np.poly(np.asarray(roots, dtype=float))  # [1, alpha[n-1], ..., alpha0]
    return descending[:0:-1]  # [alpha0, ..., alpha[n-1]]


def main():
    # Example:
    # D(s) = s^4 + 6 s^3 + 11 s^2 + 6 s + 2
    # N(s) = 3 s^2 + 2 s + 1
    a = np.array([2.0, 6.0, 11.0, 6.0])
    b = np.array([1.0, 2.0, 3.0, 0.0])

    A, B, C, D = ccf_matrices(a, b)
    Wc = controllability_matrix(A, B)

    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("Controllability matrix =\n", Wc)
    print("rank(Wc) =", np.linalg.matrix_rank(Wc))
    print("det(Wc) =", round(np.linalg.det(Wc)))

    desired_roots = np.array([-2.0, -3.0, -4.0, -5.0])
    desired_alpha = polynomial_from_roots_ascending(desired_roots)
    K = pole_placement_gain_ccf(a, desired_alpha)

    Acl = A - B @ K.reshape(1, -1)
    print("desired_alpha [alpha0,...,alpha_{n-1}] =", desired_alpha)
    print("K for u = -Kx + r =", K)
    print("closed-loop eigenvalues =", np.sort_complex(np.linalg.eigvals(Acl)))


if __name__ == "__main__":
    main()
