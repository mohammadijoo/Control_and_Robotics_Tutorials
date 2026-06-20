# Chapter23_Lesson3.py
# Ackermann's formula for SISO pole placement: u = -K x
# Requires: numpy. Optional comparison requires scipy.

import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B] for a single-input system."""
    A = np.asarray(A, dtype=complex)
    B = np.asarray(B, dtype=complex)
    n = A.shape[0]
    blocks = [B]
    for k in range(1, n):
        blocks.append(np.linalg.matrix_power(A, k) @ B)
    return np.hstack(blocks)


def matrix_polynomial(A: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    """Evaluate monic polynomial p(A).

    coefficients are [1, alpha_{n-1}, ..., alpha_0], as returned by np.poly.
    p(A) = A^n + alpha_{n-1} A^(n-1) + ... + alpha_0 I.
    """
    A = np.asarray(A, dtype=complex)
    n = A.shape[0]
    result = np.linalg.matrix_power(A, n)
    for i, alpha in enumerate(coefficients[1:]):
        power = n - 1 - i
        term = np.eye(n, dtype=complex) if power == 0 else np.linalg.matrix_power(A, power)
        result = result + alpha * term
    return result


def ackermann_gain(A: np.ndarray, B: np.ndarray, desired_poles) -> np.ndarray:
    """Compute K from Ackermann's formula.

    For x_dot = A x + B u and u = -K x, the closed-loop matrix is A - B K.
    """
    A = np.asarray(A, dtype=complex)
    B = np.asarray(B, dtype=complex).reshape((-1, 1))
    n = A.shape[0]
    if A.shape != (n, n) or B.shape != (n, 1):
        raise ValueError("A must be n by n and B must be n by 1.")

    Ctrb = controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(Ctrb)
    if rank != n:
        raise ValueError(f"System is not controllable: rank(C)={rank}, n={n}.")

    p = np.poly(desired_poles)  # [1, alpha_{n-1}, ..., alpha_0]
    phi_A = matrix_polynomial(A, p)
    e_n_T = np.zeros((1, n), dtype=complex)
    e_n_T[0, -1] = 1.0
    K = e_n_T @ np.linalg.inv(Ctrb) @ phi_A
    return np.real_if_close(K)


def verify(A: np.ndarray, B: np.ndarray, K: np.ndarray):
    Acl = A - B @ K
    return np.linalg.eigvals(Acl), np.poly(Acl)


if __name__ == "__main__":
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    desired = [-2 + 2j, -2 - 2j]

    K = ackermann_gain(A, B, desired)
    poles, char_poly = verify(A, B, K)

    print("K =", K)
    print("closed-loop poles =", poles)
    print("closed-loop characteristic coefficients =", char_poly)

    # Optional comparison with SciPy:
    try:
        from scipy.signal import place_poles
        K_scipy = place_poles(A, B, desired).gain_matrix
        print("SciPy place_poles K =", K_scipy)
    except Exception as exc:
        print("SciPy comparison skipped:", exc)
