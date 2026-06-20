# Chapter8_Lesson4.py
# Computing the state transition matrix Phi(t) using Jordan form.
# Requires: numpy, scipy (for validation only).

import numpy as np
from math import factorial
from scipy.linalg import expm

def jordan_block_exp(lam: complex, size: int, t: float) -> np.ndarray:
    """Return exp((lam I + N)t) for one Jordan block."""
    B = np.zeros((size, size), dtype=complex)
    for k in range(size):
        for j in range(k, size):
            power = j - k
            B[k, j] = np.exp(lam * t) * (t ** power) / factorial(power)
    return B

def block_diag(blocks):
    n = sum(B.shape[0] for B in blocks)
    M = np.zeros((n, n), dtype=complex)
    r = 0
    for B in blocks:
        m = B.shape[0]
        M[r:r+m, r:r+m] = B
        r += m
    return M

def phi_from_jordan(P: np.ndarray, blocks: list[tuple[complex, int]], t: float) -> np.ndarray:
    """Compute Phi(t)=P exp(Jt) P^{-1} from a supplied Jordan basis."""
    exp_blocks = [jordan_block_exp(lam, size, t) for lam, size in blocks]
    expJ = block_diag(exp_blocks)
    return P @ expJ @ np.linalg.inv(P)

if __name__ == "__main__":
    # Example: one size-2 Jordan block at lambda=2 and one size-1 block at lambda=-1.
    J = np.array([[2.0, 1.0, 0.0],
                  [0.0, 2.0, 0.0],
                  [0.0, 0.0, -1.0]], dtype=complex)

    # A nontrivial coordinate basis. The state matrix is A=P J P^{-1}.
    P = np.array([[1.0, 1.0, 0.0],
                  [0.0, 1.0, 1.0],
                  [1.0, 0.0, 1.0]], dtype=complex)
    A = P @ J @ np.linalg.inv(P)

    t = 0.40
    Phi_jordan = phi_from_jordan(P, [(2.0, 2), (-1.0, 1)], t)
    Phi_scipy = expm(A * t)

    np.set_printoptions(precision=6, suppress=True)
    print("A =")
    print(A)
    print("\nPhi(t) using Jordan form =")
    print(Phi_jordan)
    print("\nValidation with scipy.linalg.expm(A*t) =")
    print(Phi_scipy)
    print("\nFrobenius error =", np.linalg.norm(Phi_jordan - Phi_scipy, ord="fro"))

    x0 = np.array([1.0, -0.5, 0.25], dtype=complex)
    x_t = Phi_jordan @ x0
    print("\nx(t)=Phi(t)x(0) for x0=[1,-0.5,0.25]^T:")
    print(x_t)
