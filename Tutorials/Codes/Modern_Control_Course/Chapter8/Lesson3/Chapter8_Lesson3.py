"""
Chapter8_Lesson3.py

Computing the state transition matrix Phi(t) = exp(A t) via eigen-decomposition.

Dependencies:
    pip install numpy scipy

This script demonstrates:
1. diagonalization A = V Lambda V^{-1},
2. Phi(t) = V exp(Lambda t) V^{-1},
3. comparison with scipy.linalg.expm,
4. modal coordinates z = V^{-1} x.
"""

import numpy as np
from scipy.linalg import expm, eig, solve


def phi_via_eigendecomposition(A: np.ndarray, t: float, cond_tol: float = 1e10) -> np.ndarray:
    """
    Compute Phi(t) = exp(A t) from A = V Lambda V^{-1}.

    The method is reliable only when A has a complete eigenvector basis and V
    is not severely ill-conditioned.
    """
    A = np.asarray(A, dtype=float)
    eigenvalues, V = eig(A)
    if np.linalg.matrix_rank(V) < A.shape[0]:
        raise ValueError("A is not diagonalizable: eigenvector matrix V is rank deficient.")

    cond_V = np.linalg.cond(V)
    if cond_V > cond_tol:
        print(f"Warning: eigenvector matrix is ill-conditioned, cond(V)={cond_V:.3e}")

    exp_Lambda_t = np.diag(np.exp(eigenvalues * t))

    # Prefer solve over explicit inverse: V exp(Dt) V^{-1} = V exp(Dt) solve(V, I)
    Phi = V @ exp_Lambda_t @ np.linalg.inv(V)

    return np.real_if_close(Phi, tol=1000)


def modal_coordinates(A: np.ndarray, x0: np.ndarray, t: float):
    """
    Return x(t) and modal coordinates z(t) for xdot = A x.
    """
    eigenvalues, V = eig(A)
    z0 = solve(V, x0.astype(complex))
    zt = np.exp(eigenvalues * t) * z0
    xt = V @ zt
    return np.real_if_close(xt), np.real_if_close(zt), eigenvalues


def main():
    A = np.array([
        [-1.0,  2.0,  0.0],
        [ 0.0, -2.0,  0.0],
        [ 0.0,  0.0, -0.5],
    ])

    t = 2.0
    Phi_eig = phi_via_eigendecomposition(A, t)
    Phi_expm = expm(A * t)

    print("A =")
    print(A)
    print("\nPhi(t) via eigen-decomposition =")
    print(Phi_eig)
    print("\nPhi(t) via scipy.linalg.expm =")
    print(Phi_expm)
    print("\nFrobenius error =")
    print(np.linalg.norm(Phi_eig - Phi_expm, ord="fro"))

    x0 = np.array([1.0, -1.0, 2.0])
    xt, zt, lam = modal_coordinates(A, x0, t)

    print("\nEigenvalues =")
    print(lam)
    print("\nInitial state x0 =")
    print(x0)
    print("\nState x(t) = Phi(t) x0 =")
    print(xt)
    print("\nModal coordinates z(t) = exp(lambda_i t) z_i(0) =")
    print(zt)


if __name__ == "__main__":
    main()
