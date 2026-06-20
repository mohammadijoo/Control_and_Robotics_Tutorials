"""
Chapter17_Lesson3.py
Diagonal modal form for a continuous-time LTI system with distinct eigenvalues.

Libraries:
    numpy      : matrix and vector operations
    scipy      : matrix exponential and linear solves
Optional:
    python-control: higher-level state-space workflows, not required here
"""

import numpy as np
from scipy.linalg import eig, inv, expm, norm


def modal_form(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
    """Return modal matrices Lambda, Bm, Cm, D, and eigenvector matrix V.

    Coordinate convention:
        x = V z,    z = V^{-1} x
        zdot = Lambda z + Bm u
        y    = Cm z + D u
    """
    eigenvalues, V = eig(A)
    Vinv = inv(V)
    Lambda = Vinv @ A @ V
    Bm = Vinv @ B
    Cm = C @ V
    return eigenvalues, V, Vinv, Lambda, Bm, Cm, D


def simulate_zero_input(A: np.ndarray, C: np.ndarray, x0: np.ndarray, t_grid):
    """Compute x(t) and y(t) for u(t)=0 using the matrix exponential."""
    xs, ys = [], []
    for t in t_grid:
        x_t = expm(A * t) @ x0
        y_t = C @ x_t
        xs.append(x_t)
        ys.append(y_t)
    return np.array(xs), np.array(ys)


def modal_zero_input(eigenvalues, V, Vinv, C, x0, t_grid):
    """Compute zero-input response from modal coordinates."""
    z0 = Vinv @ x0
    xs, ys = [], []
    for t in t_grid:
        exp_diag = np.diag(np.exp(eigenvalues * t))
        z_t = exp_diag @ z0
        x_t = V @ z_t
        y_t = C @ x_t
        xs.append(x_t)
        ys.append(y_t)
    return np.array(xs), np.array(ys)


def main():
    # A has distinct eigenvalues -1, -2, -4 and is diagonalizable.
    A = np.array([[0.0, 1.0, 0.0],
                  [-2.0, -3.0, 0.0],
                  [0.5, 0.0, -4.0]])
    B = np.array([[0.0],
                  [1.0],
                  [0.0]])
    C = np.array([[1.0, 0.0, 1.0]])
    D = np.array([[0.0]])
    x0 = np.array([1.0, 0.0, -0.5])

    lam, V, Vinv, Lambda, Bm, Cm, Dm = modal_form(A, B, C, D)

    print("Eigenvalues:")
    print(lam)
    print("\nEigenvector matrix V:")
    print(V)
    print("\nModal A matrix Lambda = V^{-1} A V:")
    print(np.real_if_close(Lambda))
    print("\nModal input matrix Bm = V^{-1} B:")
    print(np.real_if_close(Bm))
    print("\nModal output matrix Cm = C V:")
    print(np.real_if_close(Cm))
    print("\nCondition number of V:", np.linalg.cond(V))

    t_grid = np.linspace(0.0, 5.0, 6)
    x_direct, y_direct = simulate_zero_input(A, C, x0, t_grid)
    x_modal, y_modal = modal_zero_input(lam, V, Vinv, C, x0, t_grid)

    print("\nAgreement check ||x_direct - x_modal||:", norm(x_direct - x_modal))
    print("Agreement check ||y_direct - y_modal||:", norm(y_direct - y_modal))
    print("\nSample output response y(t):")
    for t, y in zip(t_grid, y_modal):
        print(f"t={t:4.2f}, y={np.real_if_close(y[0]): .6f}")


if __name__ == "__main__":
    main()
