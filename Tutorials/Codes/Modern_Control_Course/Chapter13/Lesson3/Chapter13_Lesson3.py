# Chapter13_Lesson3.py
# Detectability and Stable Unobservable Modes
#
# Requirements:
#   pip install numpy
#
# This script implements detectability from the unobservable-subspace viewpoint.
# Continuous time: all unobservable modes must satisfy Re(lambda) < 0.
# Discrete time: all unobservable modes must satisfy |lambda| < 1.

import numpy as np


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Build O = [C; C A; ...; C A^(n-1)]."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)
    n = A.shape[0]

    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def nullspace(M: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """Return an orthonormal basis for ker(M), using the SVD."""
    U, s, Vt = np.linalg.svd(M, full_matrices=True)
    if s.size == 0:
        rank = 0
    else:
        rank = int(np.sum(s > tol * max(M.shape) * max(s)))
    return Vt[rank:].T


def unobservable_modes(A: np.ndarray, C: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """Return eigenvalues of A restricted to the unobservable subspace."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)

    O = observability_matrix(A, C)
    N = nullspace(O, tol=tol)

    if N.shape[1] == 0:
        return np.array([], dtype=complex)

    # For an LTI system, ker(O) is A-invariant. With orthonormal N,
    # the hidden dynamics matrix is A_N = N^T A N.
    A_hidden = N.T @ A @ N
    return np.linalg.eigvals(A_hidden)


def is_detectable(A: np.ndarray, C: np.ndarray, system: str = "continuous",
                  tol: float = 1e-9) -> bool:
    """Check detectability from the unobservable modes."""
    lambdas_hidden = unobservable_modes(A, C, tol=tol)

    if lambdas_hidden.size == 0:
        return True

    if system.lower().startswith("cont"):
        return bool(np.all(np.real(lambdas_hidden) < -tol))

    if system.lower().startswith("disc"):
        return bool(np.all(np.abs(lambdas_hidden) < 1.0 - tol))

    raise ValueError("system must be 'continuous' or 'discrete'")


def report(A: np.ndarray, C: np.ndarray, name: str, system: str = "continuous") -> None:
    O = observability_matrix(A, C)
    hidden = unobservable_modes(A, C)
    print(f"\n{name}")
    print("-" * len(name))
    print("A =\n", A)
    print("C =\n", C)
    print("rank(O) =", np.linalg.matrix_rank(O), "of n =", A.shape[0])
    print("unobservable modes =", hidden)
    print("detectable =", is_detectable(A, C, system=system))


if __name__ == "__main__":
    # Example 1: not observable, but detectable.
    # The unstable mode +2 is measured; hidden modes -1 and -0.5 are stable.
    A1 = np.diag([-1.0, 2.0, -0.5])
    C1 = np.array([[0.0, 1.0, 0.0]])
    report(A1, C1, "Continuous-time example: detectable but not observable")

    # Example 2: not detectable.
    # The hidden mode +1 is unstable.
    A2 = np.diag([1.0, -2.0, -0.5])
    C2 = np.array([[0.0, 1.0, 0.0]])
    report(A2, C2, "Continuous-time example: not detectable")

    # Example 3: discrete-time detectable.
    # Hidden modes 0.3 and -0.7 lie inside the unit disk; measured mode 1.2 is visible.
    A3 = np.diag([0.3, 1.2, -0.7])
    C3 = np.array([[0.0, 1.0, 0.0]])
    report(A3, C3, "Discrete-time example: detectable but not observable",
           system="discrete")
