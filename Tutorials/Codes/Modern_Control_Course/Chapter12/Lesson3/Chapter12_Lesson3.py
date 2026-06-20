# Chapter12_Lesson3.py
# Gramian-based controllability tests for continuous-time LTI systems.
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov, eigvalsh, matrix_rank
from scipy.integrate import quad_vec


def finite_horizon_gramian(A: np.ndarray, B: np.ndarray, T: float) -> np.ndarray:
    """Compute Wc(T)=int_0^T exp(A tau) B B^T exp(A^T tau) d tau."""
    if T <= 0:
        raise ValueError("T must be positive.")

    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    BBt = B @ B.T

    def integrand(tau: float) -> np.ndarray:
        E = expm(A * tau)
        return E @ BBt @ E.T

    W, _ = quad_vec(integrand, 0.0, T, epsabs=1e-10, epsrel=1e-10)
    return 0.5 * (W + W.T)


def stable_infinite_horizon_gramian(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """For Hurwitz A, solve A W + W A^T + B B^T = 0."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    W = solve_continuous_lyapunov(A, B @ B.T)
    # SciPy solves A X + X A^T = Q, so Q = -(B B^T) is often used in older texts.
    # In current SciPy convention, solve_continuous_lyapunov(A, Q) solves
    # A X + X A^H = Q. Therefore correct W is obtained with Q = -B B^T.
    W = solve_continuous_lyapunov(A, -(B @ B.T))
    return 0.5 * (W + W.T)


def gramian_test(W: np.ndarray, tol: float = 1e-9) -> dict:
    """Return eigenvalue, rank, determinant, and condition-number diagnostics."""
    lam = eigvalsh(0.5 * (W + W.T))
    rank = int(np.sum(lam > tol))
    det = float(np.linalg.det(W))
    cond = float(lam[-1] / lam[0]) if lam[0] > tol else np.inf
    return {
        "eigenvalues": lam,
        "rank_by_eigenvalues": rank,
        "rank_by_matrix_rank": int(matrix_rank(W, tol=tol)),
        "determinant": det,
        "condition_number": cond,
        "positive_definite": bool(lam[0] > tol),
    }


def minimum_energy_input(A: np.ndarray, B: np.ndarray, xf: np.ndarray, T: float, samples: int = 9):
    """
    Return the finite-horizon minimum-energy control:
    u*(t)=B^T exp(A^T (T-t)) Wc(T)^(-1) xf, for x(0)=0.
    """
    W = finite_horizon_gramian(A, B, T)
    eta = np.linalg.solve(W, xf)
    ts = np.linspace(0.0, T, samples)
    us = []
    for t in ts:
        u = B.T @ expm(A.T * (T - t)) @ eta
        us.append(u)
    return ts, np.array(us), W


if __name__ == "__main__":
    # Controllable example
    A1 = np.array([[0.0, 1.0],
                   [-2.0, -3.0]])
    B1 = np.array([[0.0],
                   [1.0]])

    T = 2.0
    W1 = finite_horizon_gramian(A1, B1, T)
    print("Controllable example: finite-horizon Wc(T)")
    print(W1)
    print(gramian_test(W1))

    Winf = stable_infinite_horizon_gramian(A1, B1)
    print("\nControllable example: infinite-horizon Wc")
    print(Winf)
    print(gramian_test(Winf))

    xf = np.array([1.0, 0.0])
    ts, us, W_for_energy = minimum_energy_input(A1, B1, xf, T)
    Emin = xf.T @ np.linalg.solve(W_for_energy, xf)
    print("\nMinimum energy to reach xf =", xf, "over T =", T, "is", Emin)
    print("Sampled u*(t):")
    for t, u in zip(ts, us):
        print(f"t={t:5.2f}, u={u.ravel()}")

    # Uncontrollable example: second state is never actuated or dynamically reached.
    A2 = np.array([[0.0, 0.0],
                   [0.0, -1.0]])
    B2 = np.array([[1.0],
                   [0.0]])
    W2 = finite_horizon_gramian(A2, B2, T)
    print("\nUncontrollable example: finite-horizon Wc(T)")
    print(W2)
    print(gramian_test(W2))
