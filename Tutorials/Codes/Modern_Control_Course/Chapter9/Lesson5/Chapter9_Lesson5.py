"""
Chapter9_Lesson5.py

Modern Control — Chapter 9, Lesson 5
Examples of Stable, Marginal, and Unstable State Matrices

This script classifies continuous-time LTI state matrices A for x_dot = A x
using eigenvalue/Jordan-block logic, then simulates representative systems.

Libraries:
    numpy
    scipy
    matplotlib

Install:
    pip install numpy scipy matplotlib
"""

import numpy as np
from scipy.linalg import expm, eigvals
import matplotlib.pyplot as plt


def classify_matrix(A: np.ndarray, tol: float = 1e-9) -> str:
    """
    Classify x_dot = A x for continuous-time stability.

    This routine uses eigenvalue real parts for the main classification and
    adds a conservative algebraic/geometric multiplicity check for eigenvalues
    on the imaginary axis. For teaching examples, this is sufficient.

    Returns:
        "asymptotically stable", "marginally stable", or "unstable"
    """
    lam = eigvals(A)
    spectral_abscissa = np.max(np.real(lam))

    if spectral_abscissa < -tol:
        return "asymptotically stable"

    if spectral_abscissa > tol:
        return "unstable"

    # Boundary case: every eigenvalue has nonpositive real part.
    # Check whether eigenvalues on the imaginary axis are semisimple.
    n = A.shape[0]
    used = np.zeros(len(lam), dtype=bool)

    for i, value in enumerate(lam):
        if used[i]:
            continue

        close = np.abs(lam - value) < 1e-7
        used[close] = True
        algebraic_mult = int(np.sum(close))

        if abs(np.real(value)) <= 1e-7:
            rank = np.linalg.matrix_rank(A - value * np.eye(n), tol=1e-7)
            geometric_mult = n - rank
            if geometric_mult < algebraic_mult:
                return "unstable"

    return "marginally stable"


def simulate(A: np.ndarray, x0: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Exact simulation x(t) = exp(A t) x0 over a time grid."""
    X = np.zeros((len(t_grid), len(x0)))
    for k, t in enumerate(t_grid):
        X[k, :] = np.real(expm(A * t) @ x0)
    return X


def print_report(name: str, A: np.ndarray) -> None:
    """Print eigenvalues and stability classification."""
    lam = eigvals(A)
    print(f"\n{name}")
    print("-" * len(name))
    print("A =")
    print(A)
    print("eigenvalues =", lam)
    print("classification =", classify_matrix(A))


def main() -> None:
    examples = {
        "Stable diagonal": np.array([[-2.0, 0.0], [0.0, -5.0]]),
        "Stable damped oscillator": np.array([[0.0, 1.0], [-4.0, -2.0]]),
        "Stable but nonnormal": np.array([[-1.0, 50.0], [0.0, -2.0]]),
        "Marginal oscillator": np.array([[0.0, 1.0], [-1.0, 0.0]]),
        "Marginal with semisimple zero": np.array([[0.0, 0.0], [0.0, -2.0]]),
        "Unstable positive eigenvalue": np.array([[1.0, 0.0], [0.0, -2.0]]),
        "Unstable defective zero": np.array([[0.0, 1.0], [0.0, 0.0]]),
    }

    for name, A in examples.items():
        print_report(name, A)

    # Compare representative trajectories.
    t = np.linspace(0.0, 10.0, 500)
    x0 = np.array([1.0, 1.0])

    selected = [
        ("Stable damped oscillator", examples["Stable damped oscillator"]),
        ("Marginal oscillator", examples["Marginal oscillator"]),
        ("Unstable defective zero", examples["Unstable defective zero"]),
    ]

    plt.figure(figsize=(9, 5))
    for label, A in selected:
        X = simulate(A, x0, t)
        norm_x = np.linalg.norm(X, axis=1)
        plt.plot(t, norm_x, label=label)

    plt.xlabel("time t")
    plt.ylabel("state norm ||x(t)||")
    plt.title("Stable, Marginal, and Unstable State-Matrix Examples")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
