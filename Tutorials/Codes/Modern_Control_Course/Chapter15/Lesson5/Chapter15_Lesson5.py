# Chapter15_Lesson5.py
"""
Duality between the observability Gramian of (A, C)
and the controllability Gramian of the dual pair (A.T, C.T).

Requires:
    numpy
    scipy

Run:
    python Chapter15_Lesson5.py
"""

import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov, eigvals


def finite_horizon_observability_gramian(A: np.ndarray, C: np.ndarray, T: float, steps: int = 4000) -> np.ndarray:
    """Compute W_o(0,T) = integral_0^T exp(A.T t) C.T C exp(A t) dt by trapezoidal quadrature."""
    W = np.zeros((A.shape[0], A.shape[0]))
    dt = T / steps
    Q = C.T @ C
    for k in range(steps + 1):
        t = k * dt
        E = expm(A * t)
        integrand = E.T @ Q @ E
        weight = 0.5 if (k == 0 or k == steps) else 1.0
        W += weight * integrand
    return W * dt


def finite_horizon_controllability_gramian(A: np.ndarray, B: np.ndarray, T: float, steps: int = 4000) -> np.ndarray:
    """Compute W_c(0,T) = integral_0^T exp(A t) B B.T exp(A.T t) dt by trapezoidal quadrature."""
    W = np.zeros((A.shape[0], A.shape[0]))
    dt = T / steps
    R = B @ B.T
    for k in range(steps + 1):
        t = k * dt
        E = expm(A * t)
        integrand = E @ R @ E.T
        weight = 0.5 if (k == 0 or k == steps) else 1.0
        W += weight * integrand
    return W * dt


def infinite_horizon_observability_gramian(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """
    For Hurwitz A, solve A.T W + W A + C.T C = 0.
    SciPy uses solve_continuous_lyapunov(a, q): a X + X a.T = q.
    Hence use a=A.T and q=-(C.T C).
    """
    return solve_continuous_lyapunov(A.T, -(C.T @ C))


def infinite_horizon_controllability_gramian(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """For Hurwitz A, solve A W + W A.T + B B.T = 0."""
    return solve_continuous_lyapunov(A, -(B @ B.T))


def main() -> None:
    A = np.array([[-1.0, 2.0],
                  [-3.0, -4.0]])
    C = np.array([[1.0, 0.5]])
    T = 3.0

    print("Eigenvalues of A:", eigvals(A))

    Wo_T = finite_horizon_observability_gramian(A, C, T)
    Wc_dual_T = finite_horizon_controllability_gramian(A.T, C.T, T)

    print("\nFinite-horizon W_o(A,C):")
    print(Wo_T)
    print("\nFinite-horizon W_c(A.T,C.T):")
    print(Wc_dual_T)
    print("\nFinite-horizon Frobenius difference:", np.linalg.norm(Wo_T - Wc_dual_T, ord="fro"))

    Wo_inf = infinite_horizon_observability_gramian(A, C)
    Wc_dual_inf = infinite_horizon_controllability_gramian(A.T, C.T)

    print("\nInfinite-horizon W_o(A,C):")
    print(Wo_inf)
    print("\nInfinite-horizon W_c(A.T,C.T):")
    print(Wc_dual_inf)
    print("\nInfinite-horizon Frobenius difference:", np.linalg.norm(Wo_inf - Wc_dual_inf, ord="fro"))

    eig_Wo = np.linalg.eigvalsh(Wo_inf)
    print("\nEigenvalues of W_o:", eig_Wo)
    print("Observable by Gramian test?", np.min(eig_Wo) > 1e-9)


if __name__ == "__main__":
    main()
