# Chapter9_Lesson2.py
"""
Eigenvalue-based stability criteria for continuous-time and discrete-time LTI systems.

Required packages:
    numpy
    scipy

Optional package:
    python-control (not required here, but useful for state-space workflows)
"""

import numpy as np
from scipy.linalg import eigvals, expm, norm


def classify_continuous_lti(A: np.ndarray, tol: float = 1e-10) -> str:
    """
    Classify x_dot = A x using eigenvalue criteria.

    For a complete rigorous marginal-stability test, one must also verify
    semisimplicity of eigenvalues on the imaginary axis. This script estimates
    that by comparing algebraic multiplicity and geometric multiplicity.
    """
    lam = eigvals(A)
    max_real = np.max(np.real(lam))

    if max_real < -tol:
        return "asymptotically stable"

    if max_real > tol:
        return "unstable"

    if imaginary_axis_modes_are_semisimple(A, tol):
        return "stable but not asymptotically stable"

    return "unstable"


def classify_discrete_lti(A: np.ndarray, tol: float = 1e-10) -> str:
    """
    Classify x[k+1] = A x[k] using eigenvalue criteria.
    """
    lam = eigvals(A)
    rho = np.max(np.abs(lam))

    if rho < 1.0 - tol:
        return "asymptotically stable"

    if rho > 1.0 + tol:
        return "unstable"

    if unit_circle_modes_are_semisimple(A, tol):
        return "stable but not asymptotically stable"

    return "unstable"


def algebraic_multiplicity(values: np.ndarray, target: complex, tol: float) -> int:
    return int(np.sum(np.abs(values - target) < 50.0 * tol))


def geometric_multiplicity(A: np.ndarray, target: complex, tol: float) -> int:
    M = A.astype(complex) - target * np.eye(A.shape[0], dtype=complex)
    singular_values = np.linalg.svd(M, compute_uv=False)
    rank = int(np.sum(singular_values > tol))
    return A.shape[0] - rank


def unique_eigenvalues(values: np.ndarray, tol: float) -> list[complex]:
    unique = []
    for value in values:
        if not any(abs(value - old) < 50.0 * tol for old in unique):
            unique.append(value)
    return unique


def imaginary_axis_modes_are_semisimple(A: np.ndarray, tol: float) -> bool:
    lam = eigvals(A)
    for value in unique_eigenvalues(lam, tol):
        if abs(np.real(value)) <= tol:
            am = algebraic_multiplicity(lam, value, tol)
            gm = geometric_multiplicity(A, value, tol)
            if gm < am:
                return False
    return True


def unit_circle_modes_are_semisimple(A: np.ndarray, tol: float) -> bool:
    lam = eigvals(A)
    for value in unique_eigenvalues(lam, tol):
        if abs(abs(value) - 1.0) <= tol:
            am = algebraic_multiplicity(lam, value, tol)
            gm = geometric_multiplicity(A, value, tol)
            if gm < am:
                return False
    return True


def print_report(name: str, A: np.ndarray, system_type: str) -> None:
    lam = eigvals(A)
    print("=" * 72)
    print(name)
    print("A =")
    print(A)
    print("eigenvalues =", np.array2string(lam, precision=6))
    if system_type == "continuous":
        print("classification:", classify_continuous_lti(A))
        sample_times = [0, 1, 2, 5, 10]
        print("||exp(A t)||_2 samples:")
        for t in sample_times:
            print(f"  t={t:>2}: {norm(expm(A * t), 2):.6f}")
    elif system_type == "discrete":
        print("classification:", classify_discrete_lti(A))
        sample_steps = [0, 1, 2, 5, 10]
        print("||A^k||_2 samples:")
        for k in sample_steps:
            print(f"  k={k:>2}: {norm(np.linalg.matrix_power(A, k), 2):.6f}")
    else:
        raise ValueError("system_type must be 'continuous' or 'discrete'")


if __name__ == "__main__":
    # Continuous-time asymptotically stable: eigenvalues -1 and -2
    A1 = np.array([[-1.0, 0.0],
                   [ 0.0,-2.0]])

    # Continuous-time stable but not asymptotically stable:
    # pure rotation, eigenvalues +j and -j, no growing Jordan block.
    A2 = np.array([[0.0, -1.0],
                   [1.0,  0.0]])

    # Continuous-time unstable because the Jordan block at lambda=0 grows linearly.
    A3 = np.array([[0.0, 1.0],
                   [0.0, 0.0]])

    # Discrete-time asymptotically stable: spectral radius 0.8
    Ad1 = np.array([[0.8, 0.0],
                    [0.0, 0.5]])

    # Discrete-time unstable: eigenvalue magnitude larger than 1
    Ad2 = np.array([[1.02, 0.0],
                    [0.0, 0.7]])

    print_report("Continuous example 1", A1, "continuous")
    print_report("Continuous example 2", A2, "continuous")
    print_report("Continuous example 3", A3, "continuous")
    print_report("Discrete example 1", Ad1, "discrete")
    print_report("Discrete example 2", Ad2, "discrete")
