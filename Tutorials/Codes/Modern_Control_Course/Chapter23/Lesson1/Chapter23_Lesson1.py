# Chapter23_Lesson1.py
"""
Formulating the SISO pole-placement problem for a continuous-time LTI system.

System:
    x_dot = A x + b u
    u     = -K x
    Acl   = A - b K

This file focuses on the formulation, controllability test, desired polynomial,
and coefficient matching for a second-order companion-form example.
"""

import numpy as np


def controllability_matrix(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return C = [b, A b, ..., A^(n-1) b]."""
    n = A.shape[0]
    cols = []
    v = b.copy()
    for _ in range(n):
        cols.append(v)
        v = A @ v
    return np.hstack(cols)


def is_controllable(A: np.ndarray, b: np.ndarray, tol: float = 1e-9) -> bool:
    """Check Kalman's rank condition for a SISO pair (A,b)."""
    Ctrb = controllability_matrix(A, b)
    return np.linalg.matrix_rank(Ctrb, tol=tol) == A.shape[0]


def companion_feedback_from_coefficients(open_coeffs_ascending, desired_coeffs_ascending):
    """
    For controllable companion form with
        A bottom row = [-a0, -a1, ..., -a_{n-1}], b = e_n,
    and feedback K = [k0, k1, ..., k_{n-1}], the closed-loop polynomial is
        s^n + (a_{n-1}+k_{n-1})s^{n-1} + ... + (a0+k0).

    Therefore k_i = alpha_i - a_i in ascending coefficient order.
    """
    open_coeffs_ascending = np.asarray(open_coeffs_ascending, dtype=float)
    desired_coeffs_ascending = np.asarray(desired_coeffs_ascending, dtype=float)
    if open_coeffs_ascending.shape != desired_coeffs_ascending.shape:
        raise ValueError("Coefficient arrays must have the same length.")
    return (desired_coeffs_ascending - open_coeffs_ascending).reshape(1, -1)


def main():
    # Second-order phase-variable system:
    # x1_dot = x2
    # x2_dot = -2 x1 - 3 x2 + u
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    b = np.array([[0.0],
                  [1.0]])

    print("A =\n", A)
    print("b =\n", b)

    Ctrb = controllability_matrix(A, b)
    print("Controllability matrix =\n", Ctrb)
    print("rank(Ctrb) =", np.linalg.matrix_rank(Ctrb))
    print("controllable =", is_controllable(A, b))

    desired_poles = np.array([-4.0, -5.0])
    desired_poly_desc = np.poly(desired_poles)  # [1, alpha1, alpha0]
    open_poly_desc = np.poly(A)                 # [1, a1, a0]

    print("open-loop polynomial coefficients [1, a1, a0] =", open_poly_desc)
    print("desired polynomial coefficients [1, alpha1, alpha0] =", desired_poly_desc)

    # Convert [1, a1, a0] to ascending non-leading coefficients [a0, a1]
    open_coeffs_ascending = open_poly_desc[:0:-1]
    desired_coeffs_ascending = desired_poly_desc[:0:-1]

    K = companion_feedback_from_coefficients(open_coeffs_ascending, desired_coeffs_ascending)
    Acl = A - b @ K

    print("K =", K)
    print("A_cl = A - b K =\n", Acl)
    print("closed-loop eigenvalues =", np.linalg.eigvals(Acl))
    print("closed-loop polynomial coefficients =", np.poly(Acl))

    # Residual check: p_cl(s) must match the desired polynomial.
    residual = np.linalg.norm(np.poly(Acl) - desired_poly_desc)
    print("polynomial matching residual =", residual)


if __name__ == "__main__":
    main()