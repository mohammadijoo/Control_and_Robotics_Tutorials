# Chapter14_Lesson4.py
# Observability in canonical (observable) forms
# Requirements: numpy, scipy (optional for eigenvalues)

import numpy as np


def observable_companion(den_coeffs, num_coeffs=None):
    """
    Build the SISO observable companion form for

        G(s) = (b_{n-1}s^{n-1} + ... + b_0) /
               (s^n + a_{n-1}s^{n-1} + ... + a_0)

    with convention:
        A_o = [[-a_{n-1}, 1, 0, ..., 0],
               [-a_{n-2}, 0, 1, ..., 0],
               ...,
               [-a_0,     0, 0, ..., 0]],
        C_o = [1, 0, ..., 0].

    den_coeffs must be [a0, a1, ..., a_{n-1}].
    num_coeffs, if supplied, must be [b0, b1, ..., b_{n-1}].
    The returned B_o is [b_{n-1}, b_{n-2}, ..., b0]^T.
    """
    den = np.asarray(den_coeffs, dtype=float).reshape(-1)
    n = den.size
    A = np.zeros((n, n), dtype=float)
    A[:, 0] = -den[::-1]
    for i in range(n - 1):
        A[i, i + 1] = 1.0
    C = np.zeros((1, n), dtype=float)
    C[0, 0] = 1.0
    if num_coeffs is None:
        B = np.zeros((n, 1), dtype=float)
    else:
        num = np.asarray(num_coeffs, dtype=float).reshape(-1)
        if num.size != n:
            raise ValueError("num_coeffs must have length n: [b0, ..., b_{n-1}]")
        B = num[::-1].reshape(n, 1)
    return A, B, C


def observability_matrix(A, C):
    """Return O = [C; C A; ...; C A^{n-1}]."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def matrix_rank(M, tol=1e-10):
    """Numerical rank computed from singular values."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def pbh_observability_test(A, C, tol=1e-9):
    """
    PBH test for observability:
        rank([lambda I - A; C]) = n for every eigenvalue lambda of A.
    """
    A = np.asarray(A, dtype=complex)
    C = np.asarray(C, dtype=complex)
    n = A.shape[0]
    eigvals = np.linalg.eigvals(A)
    results = []
    for lam in eigvals:
        pbh = np.vstack((lam * np.eye(n) - A, C))
        r = matrix_rank(pbh, tol=tol)
        results.append((lam, r, r == n))
    return results


def reconstruct_initial_state_from_derivatives(A, C, y_derivatives_at_0):
    """
    For zero input, y^{(k)}(0) = C A^k x(0).
    Given [y(0), ydot(0), ..., y^{(n-1)}(0)], solve O x0 = eta.
    """
    O = observability_matrix(A, C)
    eta = np.asarray(y_derivatives_at_0, dtype=float).reshape(-1, 1)
    if O.shape[0] != eta.shape[0]:
        raise ValueError("Need exactly n output derivatives for an n-state SISO system.")
    return np.linalg.solve(O, eta)


if __name__ == "__main__":
    # Example: G(s) = (2 s^2 + 3 s + 4)/(s^3 + 6 s^2 + 11 s + 6)
    # denominator coefficients are [a0, a1, a2]
    den = [6.0, 11.0, 6.0]
    num = [4.0, 3.0, 2.0]

    A, B, C = observable_companion(den, num)
    O = observability_matrix(A, C)

    print("A_o =\n", A)
    print("B_o =\n", B)
    print("C_o =\n", C)
    print("Observability matrix O =\n", O)
    print("det(O) =", np.linalg.det(O))
    print("rank(O) =", matrix_rank(O), "out of", A.shape[0])

    print("\nPBH observability test:")
    for lam, r, passed in pbh_observability_test(A, C):
        print(f"lambda={lam:.6g}, rank={r}, observable_at_lambda={passed}")

    # Zero-input reconstruction example
    x0_true = np.array([[1.0], [-2.0], [0.5]])
    eta = O @ x0_true
    x0_hat = reconstruct_initial_state_from_derivatives(A, C, eta)
    print("\nTrue x0 =\n", x0_true)
    print("Reconstructed x0 =\n", x0_hat)
