# Chapter12_Lesson1.py
# Definition and numerical computation of the finite-horizon controllability Gramian.
#
# Required packages:
#   pip install numpy scipy
#
# This script demonstrates:
#   1. Wc(T) = integral_0^T exp(A tau) B B^T exp(A^T tau) d tau
#   2. symmetry and positive semidefiniteness of Wc(T)
#   3. rank-based Gramian controllability test on a finite horizon
#   4. infinite-horizon Gramian for asymptotically stable A via the Lyapunov equation

import numpy as np
from scipy.linalg import expm, eigvalsh, solve_continuous_lyapunov
from scipy.integrate import quad_vec


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Apow = np.eye(n)
    for _ in range(n):
        blocks.append(Apow @ B)
        Apow = Apow @ A
    return np.hstack(blocks)


def finite_horizon_gramian(A: np.ndarray, B: np.ndarray, T: float) -> np.ndarray:
    """Compute Wc(T) by numerical matrix integration."""
    if T <= 0:
        raise ValueError("T must be positive.")

    def integrand(tau: float) -> np.ndarray:
        E = expm(A * tau)
        return E @ B @ B.T @ E.T

    W, _ = quad_vec(integrand, 0.0, T, epsabs=1e-11, epsrel=1e-11)
    return 0.5 * (W + W.T)  # remove numerical asymmetry


def infinite_horizon_gramian(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """
    For Hurwitz A, Wc_inf solves:
        A W + W A^T + B B^T = 0.
    SciPy uses A X + X A^T = Q, hence Q = -B B^T.
    """
    W = solve_continuous_lyapunov(A, -(B @ B.T))
    return 0.5 * (W + W.T)


def print_matrix(name: str, M: np.ndarray) -> None:
    print(f"\n{name} =")
    print(np.array2string(M, precision=6, suppress_small=True))


def main() -> None:
    # Example 1: controllable second-order system.
    A1 = np.array([[0.0, 1.0],
                   [-2.0, -3.0]])
    B1 = np.array([[0.0],
                   [1.0]])
    T = 2.0

    C1 = controllability_matrix(A1, B1)
    W1 = finite_horizon_gramian(A1, B1, T)
    Winf1 = infinite_horizon_gramian(A1, B1)

    print("Example 1: controllable system")
    print_matrix("Kalman controllability matrix", C1)
    print("rank(C) =", np.linalg.matrix_rank(C1))
    print_matrix(f"Wc({T})", W1)
    print("eigenvalues of Wc(T) =", eigvalsh(W1))
    print("rank(Wc(T)) =", np.linalg.matrix_rank(W1, tol=1e-9))
    print_matrix("Wc(infinity)", Winf1)

    # Example 2: uncontrollable system. Second state is not affected by input or A-propagation.
    A2 = np.array([[0.0, 0.0],
                   [0.0, -1.0]])
    B2 = np.array([[1.0],
                   [0.0]])
    W2 = finite_horizon_gramian(A2, B2, T)
    C2 = controllability_matrix(A2, B2)

    print("\n\nExample 2: uncontrollable system")
    print_matrix("Kalman controllability matrix", C2)
    print("rank(C) =", np.linalg.matrix_rank(C2))
    print_matrix(f"Wc({T})", W2)
    print("eigenvalues of Wc(T) =", eigvalsh(W2))
    print("rank(Wc(T)) =", np.linalg.matrix_rank(W2, tol=1e-9))

    # A simple geometric interpretation:
    # larger eigenvalue means a direction accumulates more control influence.
    vals, vecs = np.linalg.eigh(W1)
    print("\nPrincipal Gramian axes for Example 1:")
    for i, val in enumerate(vals):
        print(f"axis {i+1}: eigenvalue={val:.6f}, direction={vecs[:, i]}")


if __name__ == "__main__":
    main()
