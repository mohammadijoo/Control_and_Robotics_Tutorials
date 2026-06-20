"""
Chapter16_Lesson2.py

Construction of Controllable Canonical Form (CCF) from SISO transfer-function data.

Convention:
    G(s) = N(s) / D(s)
    D(s) = s^n + a1 s^(n-1) + ... + an

If deg(N) < n:
    A = companion matrix with superdiagonal ones and last row [-an, ..., -a1]
    B = e_n
    C = [r_n, r_(n-1), ..., r_1]
    Dfeed = 0

If deg(N) = n:
    Dfeed = leading numerator coefficient after denominator is monic
    R(s) = N(s) - Dfeed * D(s)
    C is built from the strictly proper remainder R(s).
"""

from __future__ import annotations

import numpy as np


def trim_leading_zeros(coeffs, tol=1e-12):
    coeffs = np.asarray(coeffs, dtype=float).flatten()
    if coeffs.size == 0:
        raise ValueError("Coefficient vector cannot be empty.")
    idx = 0
    while idx < coeffs.size - 1 and abs(coeffs[idx]) < tol:
        idx += 1
    return coeffs[idx:]


def controllable_canonical_form(num, den):
    """
    Build a controllable-canonical-form realization.

    Parameters
    ----------
    num : array_like
        Numerator coefficients in descending powers of s.
    den : array_like
        Denominator coefficients in descending powers of s.

    Returns
    -------
    A, B, C, Dfeed : numpy arrays
    """
    num = trim_leading_zeros(num)
    den = trim_leading_zeros(den)

    if den[0] == 0:
        raise ValueError("Leading denominator coefficient must be nonzero.")

    # Normalize denominator to monic form.
    leading = den[0]
    den = den / leading
    num = num / leading

    n = len(den) - 1
    if n < 1:
        raise ValueError("Denominator degree must be at least one.")
    if len(num) > n + 1:
        raise ValueError("Improper transfer function: numerator degree exceeds denominator degree.")

    # Pad numerator to length n + 1 so it aligns with denominator coefficients.
    num_full = np.zeros(n + 1)
    num_full[-len(num):] = num

    # Direct term and strictly proper remainder.
    Dfeed = float(num_full[0])
    rem = num_full.copy()
    rem -= Dfeed * den
    # rem[0] should be zero; remaining coefficients are for s^(n-1), ..., s^0
    rem_strict = rem[1:]

    A = np.zeros((n, n))
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -den[:0:-1]  # [-an, ..., -a1]

    B = np.zeros((n, 1))
    B[-1, 0] = 1.0

    # C = [constant coefficient, coefficient of s, ..., coefficient of s^(n-1)]
    C = rem_strict[::-1].reshape(1, n)

    return A, B, C, np.array([[Dfeed]])


def transfer_from_state_space(A, B, C, Dfeed):
    """
    Numerically reconstruct Markov-style polynomial coefficients by sampling.
    This helper is mainly for demonstration and validation, not symbolic proof.
    """
    import scipy.signal as sig

    num, den = sig.ss2tf(A, B, C, Dfeed)
    return num[0], den


def main():
    # Example:
    # G(s) = (2 s^2 + 5 s + 3)/(s^3 + 4 s^2 + 6 s + 8)
    num = [2, 5, 3]
    den = [1, 4, 6, 8]

    A, B, C, Dfeed = controllable_canonical_form(num, den)

    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("D =\n", Dfeed)

    # Optional validation when SciPy is installed.
    try:
        num_back, den_back = transfer_from_state_space(A, B, C, Dfeed)
        print("Reconstructed numerator coefficients:", num_back)
        print("Reconstructed denominator coefficients:", den_back)
    except Exception as exc:
        print("SciPy validation skipped:", exc)


if __name__ == "__main__":
    main()
