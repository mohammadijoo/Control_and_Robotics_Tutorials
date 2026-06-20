"""
Chapter8_Lesson5.py

Numerical computation of the state transition matrix Phi(t)=exp(A t).

This file demonstrates:
1. SciPy's robust dense matrix exponential.
2. A compact from-scratch scaling-and-squaring Taylor implementation.
3. Exact continuous-to-discrete conversion using the Van Loan block matrix.
4. Practical verification tests: semigroup, inverse, residual, and conditioning.
"""

import numpy as np
from numpy.linalg import norm, inv, cond
from scipy.linalg import expm


def expm_taylor_scaling_squaring(M, order=40):
    """
    Compute exp(M) using scaling and squaring with a truncated Taylor series.

    This is educational. For production dense problems, prefer scipy.linalg.expm,
    which uses a high-quality scaling-and-squaring Pade algorithm.
    """
    n = M.shape[0]
    m_norm = norm(M, ord=np.inf)
    s = max(0, int(np.ceil(np.log2(m_norm)))) if m_norm > 0 else 0
    A_scaled = M / (2 ** s)

    E = np.eye(n)
    term = np.eye(n)
    for k in range(1, order + 1):
        term = term @ A_scaled / k
        E = E + term

    for _ in range(s):
        E = E @ E
    return E


def exact_c2d(A, B, h):
    """
    Exact zero-order-hold discretization:
        x_{k+1} = Ad x_k + Bd u_k
    where
        Ad = exp(A h),
        Bd = integral_0^h exp(A tau) B d tau.

    The Van Loan block exponential computes both without explicitly inverting A.
    """
    n, m = B.shape
    Z = np.zeros((m, m))
    M = np.block([[A, B],
                  [np.zeros((m, n)), Z]])
    EM = expm(M * h)
    Ad = EM[:n, :n]
    Bd = EM[:n, n:n + m]
    return Ad, Bd


def residual_check(A, t, Phi, eps=1e-6):
    """
    Approximate differential residual:
        d/dt exp(A t) - A exp(A t) = 0.
    """
    Phi_plus = expm(A * (t + eps))
    Phi_minus = expm(A * (t - eps))
    dPhi_dt = (Phi_plus - Phi_minus) / (2 * eps)
    return norm(dPhi_dt - A @ Phi, ord=np.inf)


def main():
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [-2.0, -3.0, -4.0]])

    B = np.array([[0.0],
                  [0.0],
                  [1.0]])

    t = 0.5
    h = 0.1

    Phi_scipy = expm(A * t)
    Phi_scratch = expm_taylor_scaling_squaring(A * t, order=45)

    print("A =")
    print(A)

    print("\nPhi(t) from scipy.linalg.expm:")
    print(Phi_scipy)

    print("\nPhi(t) from scratch scaling-and-squaring Taylor:")
    print(Phi_scratch)

    print("\nInfinity-norm difference:")
    print(norm(Phi_scipy - Phi_scratch, ord=np.inf))

    print("\nSemigroup test ||Phi(t)Phi(s)-Phi(t+s)||_inf:")
    s = 0.25
    semigroup_error = norm(expm(A * t) @ expm(A * s) - expm(A * (t + s)), ord=np.inf)
    print(semigroup_error)

    print("\nInverse test ||Phi(t)Phi(-t)-I||_inf:")
    inverse_error = norm(expm(A * t) @ expm(-A * t) - np.eye(A.shape[0]), ord=np.inf)
    print(inverse_error)

    print("\nDifferential residual ||dPhi/dt - A Phi||_inf:")
    print(residual_check(A, t, Phi_scipy))

    print("\nCondition number cond(Phi(t)):")
    print(cond(Phi_scipy))

    Ad, Bd = exact_c2d(A, B, h)
    print("\nExact zero-order-hold discretization with h=0.1:")
    print("Ad =")
    print(Ad)
    print("Bd =")
    print(Bd)

    # Optional comparison for nonsingular A:
    Bd_formula = inv(A) @ (Ad - np.eye(A.shape[0])) @ B
    print("\nBd comparison with A^{-1}(Ad-I)B:")
    print(norm(Bd - Bd_formula, ord=np.inf))


if __name__ == "__main__":
    main()
