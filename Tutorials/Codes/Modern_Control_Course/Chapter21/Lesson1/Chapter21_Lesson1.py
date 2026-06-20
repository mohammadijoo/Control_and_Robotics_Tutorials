# Chapter21_Lesson1.py
# Finite transmission zeros from the Rosenbrock system matrix.
# Required packages: numpy, scipy, sympy
#
# Model:
#   x_dot = A x + B u
#   y     = C x + D u
#
# A complex number z is a finite transmission zero if the Rosenbrock matrix
#   R(z) = [[zI - A, -B],
#           [C,       D]]
# loses rank relative to its normal rank.

import numpy as np
import scipy.linalg as la
import sympy as sp


def rosenbrock_matrix(A, B, C, D, s):
    """Return the Rosenbrock system matrix R(s)."""
    A = np.asarray(A, dtype=complex)
    B = np.asarray(B, dtype=complex)
    C = np.asarray(C, dtype=complex)
    D = np.asarray(D, dtype=complex)

    n = A.shape[0]
    top = np.hstack((s * np.eye(n, dtype=complex) - A, -B))
    bottom = np.hstack((C, D))
    return np.vstack((top, bottom))


def numerical_rank(M, tol=1e-9):
    """SVD-based numerical rank."""
    singular_values = la.svd(M, compute_uv=False)
    return int(np.sum(singular_values > tol))


def siso_zeros_by_rosenbrock_determinant(A, B, C, D):
    """
    For a square SISO Rosenbrock matrix, det(R(s)) gives the finite invariant
    zero polynomial up to a nonzero scalar factor when the realization is regular.
    """
    s = sp.symbols("s")
    A_sp = sp.Matrix(A)
    B_sp = sp.Matrix(B)
    C_sp = sp.Matrix(C)
    D_sp = sp.Matrix(D)
    n = A_sp.rows

    R = A_sp.zeros(n + C_sp.rows, n + B_sp.cols)
    R[:n, :n] = s * sp.eye(n) - A_sp
    R[:n, n:] = -B_sp
    R[n:, :n] = C_sp
    R[n:, n:] = D_sp

    zero_polynomial = sp.factor(R.det())
    roots = sp.nroots(zero_polynomial)
    return zero_polynomial, roots, R


def main():
    # Minimal SISO example with transfer function G(s) = (s + 4)/(s^2 + 3s + 2).
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[4.0, 1.0]])
    D = np.array([[0.0]])

    zero_polynomial, zeros, R_symbolic = siso_zeros_by_rosenbrock_determinant(A, B, C, D)
    print("Symbolic Rosenbrock matrix R(s):")
    print(R_symbolic)
    print("\ndet(R(s)) =", zero_polynomial)
    print("Transmission zeros =", zeros)

    z = complex(-4.0, 0.0)
    Rz = rosenbrock_matrix(A, B, C, D, z)
    print("\nR(-4) =")
    print(np.real_if_close(Rz))
    print("rank R(-4) =", numerical_rank(Rz))
    print("normal rank for this SISO square example =", A.shape[0] + B.shape[1])

    # Verify the zero-output exponential trajectory:
    # x(t) = exp(z t) x0, u(t) = exp(z t) u0.
    x0 = np.array([[1.0], [-4.0]])
    u0 = np.array([[6.0]])
    residual_state = (z * x0) - A @ x0 - B @ u0
    residual_output = C @ x0 + D @ u0
    print("\nState residual z*x0 - A*x0 - B*u0 =")
    print(np.real_if_close(residual_state))
    print("Output residual C*x0 + D*u0 =")
    print(np.real_if_close(residual_output))


if __name__ == "__main__":
    main()
