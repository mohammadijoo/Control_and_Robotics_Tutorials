"""
Chapter16_Lesson5.py
Advantages and drawbacks of controllable canonical form (CCF).

Libraries used:
  - numpy for linear algebra
Optional production libraries for related Modern Control work:
  - scipy.signal, python-control, slycot
"""

import numpy as np


def companion_ccf(a_ascending, beta_descending, d=0.0):
    """
    Build the SISO controllable canonical form

        G(s) = d + (beta_0 s^(n-1) + ... + beta_(n-1)) /
                    (s^n + a_(n-1)s^(n-1) + ... + a_0)

    a_ascending = [a_0, a_1, ..., a_(n-1)]
    beta_descending = [beta_0, beta_1, ..., beta_(n-1)]
    """
    a = np.asarray(a_ascending, dtype=float)
    beta = np.asarray(beta_descending, dtype=float)
    n = len(a)
    if beta.shape[0] != n:
        raise ValueError("beta_descending must have length n")

    A = np.zeros((n, n))
    if n > 1:
        A[:-1, 1:] = np.eye(n - 1)
    A[-1, :] = -a

    B = np.zeros((n, 1))
    B[-1, 0] = 1.0

    # With this shift-up convention, C multiplies [1, s, ..., s^(n-1)] / denominator.
    C = beta[::-1].reshape(1, n)
    D = np.array([[float(d)]])
    return A, B, C, D


def controllability_matrix(A, B):
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])


def ccf_feedback_gain(a_ascending, desired_poles):
    """
    For u = -Kx + r in CCF:
        k_i = alpha_(i-1) - a_(i-1)
    where desired polynomial is
        s^n + alpha_(n-1)s^(n-1) + ... + alpha_0.
    """
    a = np.asarray(a_ascending, dtype=float)
    poly_desc = np.poly(np.asarray(desired_poles, dtype=complex)).real
    alpha_ascending = poly_desc[1:][::-1]
    return alpha_ascending - a


def main():
    # Denominator: s^3 + 6s^2 + 11s + 6
    # Numerator:   2s^2 + 5s + 3
    a = [6.0, 11.0, 6.0]
    beta = [2.0, 5.0, 3.0]
    A, B, C, D = companion_ccf(a, beta)

    Qc = controllability_matrix(A, B)
    print("A_c =\n", A)
    print("B_c =\n", B)
    print("C_c =\n", C)
    print("rank(Qc) =", np.linalg.matrix_rank(Qc))
    print("cond(Qc) =", np.linalg.cond(Qc))

    desired_poles = [-2.0, -3.0, -4.0]
    K = ccf_feedback_gain(a, desired_poles)
    Acl = A - B @ K.reshape(1, -1)
    print("K for desired poles", desired_poles, "=", K)
    print("closed-loop eigenvalues =", np.linalg.eigvals(Acl))

    # Drawback demonstration: small coefficient perturbations can move roots noticeably.
    rng = np.random.default_rng(4)
    for eps in [1e-6, 1e-4, 1e-2]:
        perturbed_a = np.asarray(a) * (1.0 + eps * rng.standard_normal(len(a)))
        p_desc = np.r_[1.0, perturbed_a[::-1]]
        print(f"eps={eps:g}, perturbed poles=", np.sort_complex(np.roots(p_desc)))


if __name__ == "__main__":
    main()
