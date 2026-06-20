"""
Chapter23_Lesson2.py
Pole Assignment via Controllable Canonical Form (CCF)

Convention:
    A_c = [[0, 1, 0, ..., 0],
           ...
           [0, 0, 0, ..., 1],
           [-a0, -a1, ..., -a_{n-1}]]
    b_c = [0, 0, ..., 1]^T

For u = -K_c z + r and desired polynomial
    s^n + alpha_{n-1}s^{n-1} + ... + alpha_1 s + alpha_0,
the CCF gain is
    K_c = [alpha_0-a_0, alpha_1-a_1, ..., alpha_{n-1}-a_{n-1}].
"""

import numpy as np


def companion_pair_from_coefficients(a_ascending):
    """Return (A_c, b_c) for p(s)=s^n+a_{n-1}s^{n-1}+...+a_0.

    Parameters
    ----------
    a_ascending : array_like
        [a0, a1, ..., a_{n-1}]
    """
    a = np.asarray(a_ascending, dtype=float)
    n = len(a)
    A = np.zeros((n, n))
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -a
    b = np.zeros((n, 1))
    b[-1, 0] = 1.0
    return A, b


def controllability_matrix(A, b):
    """Construct M=[b, A b, ..., A^(n-1)b]."""
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float).reshape((-1, 1))
    n = A.shape[0]
    cols = []
    v = b.copy()
    for _ in range(n):
        cols.append(v)
        v = A @ v
    return np.hstack(cols)


def desired_coefficients_from_poles(poles):
    """Return [alpha0, alpha1, ..., alpha_{n-1}] from desired roots."""
    coeff_desc = np.poly(np.asarray(poles, dtype=complex))  # [1, alpha_{n-1}, ..., alpha0]
    coeff_desc = np.real_if_close(coeff_desc, tol=1000)
    return np.asarray(coeff_desc[1:][::-1], dtype=float)


def ccf_gain(open_coeffs_ascending, desired_poles):
    """Compute K_c by coefficient matching."""
    a = np.asarray(open_coeffs_ascending, dtype=float)
    alpha = desired_coefficients_from_poles(desired_poles)
    if len(a) != len(alpha):
        raise ValueError("Number of desired poles must match system order.")
    return alpha - a


def transform_to_ccf_gain(A, b, open_coeffs_ascending, desired_poles):
    """Compute physical-coordinate gain K_x from a controllable pair (A,b).

    Assumes x = T z, where (A_c,b_c) = (T^{-1} A T, T^{-1} b).
    Then K_x = K_c T^{-1}.
    """
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float).reshape((-1, 1))
    Ac, bc = companion_pair_from_coefficients(open_coeffs_ascending)

    M = controllability_matrix(A, b)
    Mc = controllability_matrix(Ac, bc)

    if np.linalg.matrix_rank(M) < A.shape[0]:
        raise ValueError("The pair (A,b) is not controllable; arbitrary pole assignment is impossible.")

    T = M @ np.linalg.inv(Mc)
    Kc = ccf_gain(open_coeffs_ascending, desired_poles).reshape((1, -1))
    Kx = Kc @ np.linalg.inv(T)
    return Kx, Kc, T, Ac, bc


def main():
    # Example: open-loop p(s)=s^3 + 6s^2 + 11s + 6 = (s+1)(s+2)(s+3)
    a = np.array([6.0, 11.0, 6.0])  # [a0, a1, a2]
    desired_poles = np.array([-4.0, -5.0, -6.0])

    Ac, bc = companion_pair_from_coefficients(a)
    Kc = ccf_gain(a, desired_poles)
    Acl = Ac - bc @ Kc.reshape(1, -1)

    print("A_c =\n", Ac)
    print("b_c =\n", bc)
    print("K_c =", Kc)
    print("Closed-loop eigenvalues =", np.linalg.eigvals(Acl))

    # Same system in a noncanonical coordinate x=T z
    T_true = np.array([[1.0, 2.0, 0.0],
                       [0.0, 1.0, 1.0],
                       [2.0, 0.0, 1.0]])
    A = T_true @ Ac @ np.linalg.inv(T_true)
    b = T_true @ bc

    Kx, Kc2, T, _, _ = transform_to_ccf_gain(A, b, a, desired_poles)
    print("\nK_x =", Kx)
    print("Recovered T =\n", T)
    print("Eigenvalues of A-bK_x =", np.linalg.eigvals(A - b @ Kx))


if __name__ == "__main__":
    main()
