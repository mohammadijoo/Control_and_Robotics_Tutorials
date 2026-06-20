"""
Chapter17_Lesson1.py

Observable Canonical Form (OCF) construction for a SISO continuous-time
transfer function

        G(s) = N(s) / D(s)

with monic denominator

        D(s) = s^n + a1 s^(n-1) + ... + an.

This script constructs the observer/observable companion realization

        x_dot = A_o x + B_o u
        y     = C_o x + D_o u

using the convention

        A_o = A_c^T,   C_o = [0 ... 0 1],

where A_c is the controllable companion matrix.
"""

from __future__ import annotations

import numpy as np


def trim_leading_zeros(coeffs, tol=1e-12):
    """Remove leading coefficients that are numerically zero."""
    coeffs = list(map(float, coeffs))
    while len(coeffs) > 1 and abs(coeffs[0]) < tol:
        coeffs.pop(0)
    return np.array(coeffs, dtype=float)


def observable_canonical_form(den, num):
    """
    Build OCF from denominator and numerator coefficients in descending powers.

    Parameters
    ----------
    den : sequence of float
        [1, a1, a2, ..., an] after normalization; non-monic input is allowed.
    num : sequence of float
        Numerator coefficients in descending powers. It may be strictly proper
        or proper. If degree(num) == degree(den), the direct term D_o is
        separated by polynomial division.

    Returns
    -------
    A_o, B_o, C_o, D_o : numpy arrays
        Observable canonical realization.
    """
    den = trim_leading_zeros(den)
    num = trim_leading_zeros(num)

    if den[0] == 0:
        raise ValueError("The denominator leading coefficient must be nonzero.")

    # Normalize denominator to be monic.
    num = num / den[0]
    den = den / den[0]

    n = len(den) - 1
    if n <= 0:
        raise ValueError("The denominator must have positive degree.")

    if len(num) > n + 1:
        raise ValueError("This implementation expects a proper transfer function.")

    # Pad numerator to length n+1, aligned with powers s^n, ..., s, 1.
    num_pad = np.zeros(n + 1)
    num_pad[-len(num):] = num

    # Separate direct feedthrough if numerator degree is n.
    D_o = float(num_pad[0])
    remainder = num_pad - D_o * den
    # Now remainder[0] is zero, and remainder[1:] corresponds to
    # coefficients of s^(n-1), ..., s, 1.
    numerator_without_direct = remainder[1:]

    a = den[1:]  # [a1, ..., an]

    A_o = np.zeros((n, n))
    for i in range(1, n):
        A_o[i, i - 1] = 1.0
    A_o[:, -1] = -a[::-1]  # [-an, ..., -a1]^T

    B_o = numerator_without_direct[::-1].reshape((n, 1))  # [constant, ..., s^(n-1)]^T
    C_o = np.zeros((1, n))
    C_o[0, -1] = 1.0

    return A_o, B_o, C_o, np.array([[D_o]])


def observability_matrix(A, C):
    """Construct O = [C; C A; ...; C A^(n-1)]."""
    A = np.array(A, dtype=float)
    C = np.array(C, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def transfer_value(A, B, C, D, s):
    """Evaluate G(s) = C (sI - A)^(-1) B + D at a scalar s."""
    A = np.array(A, dtype=float)
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def similarity_to_ocf(A, B, C):
    """
    For an observable SISO realization, compute the similarity map P satisfying

        A_ocf = P^(-1) A P, B_ocf = P^(-1) B, C_ocf = C P.

    The formula uses observability matrices:

        O_ocf = O_original P  =>  P = O_original^(-1) O_ocf.
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float).reshape((-1, 1))
    C = np.array(C, dtype=float).reshape((1, -1))

    # Get characteristic polynomial and use a zero numerator only to build the
    # target OCF A and C. B is not needed for the transformation matrix.
    den = np.poly(A)  # [1, a1, ..., an]
    A_target, _, C_target, _ = observable_canonical_form(den, [0.0])

    O_original = observability_matrix(A, C)
    O_target = observability_matrix(A_target, C_target)

    if np.linalg.matrix_rank(O_original) < A.shape[0]:
        raise ValueError("The original realization is not observable.")

    P = np.linalg.solve(O_original, O_target)
    A_ocf = np.linalg.solve(P, A @ P)
    B_ocf = np.linalg.solve(P, B)
    C_ocf = C @ P
    return P, A_ocf, B_ocf, C_ocf


def main():
    # Example:
    # G(s) = (2 s^2 + 5 s + 3) / (s^3 + 4 s^2 + 6 s + 8)
    den = [1.0, 4.0, 6.0, 8.0]
    num = [2.0, 5.0, 3.0]

    A_o, B_o, C_o, D_o = observable_canonical_form(den, num)

    print("A_o =\n", A_o)
    print("B_o =\n", B_o)
    print("C_o =\n", C_o)
    print("D_o =\n", D_o)

    # Check at selected complex frequencies.
    for s in [1.0, 2.0, 1.0 + 1.0j]:
        g_state = transfer_value(A_o, B_o, C_o, D_o, s)[0, 0]
        g_poly = np.polyval(num, s) / np.polyval(den, s)
        print(f"s={s:>8}: state={g_state}, polynomial={g_poly}, error={abs(g_state-g_poly):.2e}")

    O_o = observability_matrix(A_o, C_o)
    print("rank(O_o) =", np.linalg.matrix_rank(O_o))


if __name__ == "__main__":
    main()
