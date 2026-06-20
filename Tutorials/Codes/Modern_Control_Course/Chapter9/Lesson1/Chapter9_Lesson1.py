# Chapter9_Lesson1.py
"""
Modern Control — Chapter 9, Lesson 1
Stability, Asymptotic Stability, and Instability (State-Space View)

This script studies continuous-time LTI systems:
    x_dot = A x
through the state transition matrix exp(A t), trajectories, eigenvalue
abscissa, and a quadratic Lyapunov equation check.
"""

import numpy as np
from scipy.linalg import expm, eigvals, solve_continuous_lyapunov


def classify_ct_lti(A: np.ndarray, tol: float = 1e-9) -> str:
    """
    Basic continuous-time LTI classification from eigenvalues.

    For a complete rigorous marginal-stability decision, imaginary-axis
    eigenvalues must also be semisimple. This function flags the borderline
    case as "marginal-candidate" because Lesson 2 develops the full test.
    """
    lam = eigvals(A)
    alpha = np.max(np.real(lam))

    if alpha < -tol:
        return "asymptotically stable"
    if alpha > tol:
        return "unstable"

    return "marginal-candidate: inspect Jordan structure for imaginary-axis modes"


def exact_trajectory(A: np.ndarray, x0: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Compute x(t)=exp(A t)x0 on a grid."""
    return np.vstack([expm(A * t) @ x0 for t in t_grid])


def lyapunov_certificate(A: np.ndarray):
    """
    Solve A^T P + P A = -I.
    If P is symmetric positive definite, then V=x^T P x proves asymptotic stability.
    """
    Q = np.eye(A.shape[0])
    P = solve_continuous_lyapunov(A.T, -Q)
    eig_P = np.linalg.eigvalsh((P + P.T) / 2.0)
    return P, eig_P


def report_system(name: str, A: np.ndarray, x0: np.ndarray) -> None:
    print(f"\n=== {name} ===")
    print("A =")
    print(A)

    lam = eigvals(A)
    print("eigenvalues =", lam)
    print("classification =", classify_ct_lti(A))

    t_grid = np.linspace(0, 10, 101)
    X = exact_trajectory(A, x0, t_grid)
    norms = np.linalg.norm(X, axis=1)
    print("||x(0)|| =", norms[0])
    print("||x(10)|| =", norms[-1])
    print("max_t ||x(t)|| on [0,10] =", np.max(norms))

    if classify_ct_lti(A).startswith("asymptotically"):
        P, eig_P = lyapunov_certificate(A)
        print("Lyapunov P solving A^T P + P A = -I:")
        print(P)
        print("eigenvalues(P) =", eig_P)


if __name__ == "__main__":
    x0 = np.array([1.0, -0.5])

    A_stable = np.array([[-1.0, 2.0],
                         [-3.0, -2.0]])

    A_center = np.array([[0.0, 1.0],
                         [-1.0, 0.0]])

    A_unstable = np.array([[0.2, 1.0],
                           [0.0, -1.0]])

    report_system("Stable spiral", A_stable, x0)
    report_system("Marginal center", A_center, x0)
    report_system("Unstable saddle/source component", A_unstable, x0)
