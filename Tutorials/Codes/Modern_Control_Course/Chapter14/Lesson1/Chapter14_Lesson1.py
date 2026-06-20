"""
Chapter14_Lesson1.py

Kalman observability matrix and rank condition for continuous-time LTI systems.

Model:
    x_dot = A x + B u
    y     = C x + D u

For observability of the initial state, only (A, C) matters.
"""

import numpy as np


def observability_matrix(A: np.ndarray, C: np.ndarray, n: int | None = None) -> np.ndarray:
    """Return O_n = [C; C A; ...; C A^(n-1)]."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)

    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be square.")
    if C.ndim != 2 or C.shape[1] != A.shape[0]:
        raise ValueError("C must have the same number of columns as A has states.")

    n_states = A.shape[0]
    n = n_states if n is None else int(n)
    blocks = []
    Ak = np.eye(n_states)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def numerical_rank(M: np.ndarray, tol: float | None = None) -> tuple[int, np.ndarray, float]:
    """SVD-based numerical rank."""
    M = np.asarray(M, dtype=float)
    singular_values = np.linalg.svd(M, compute_uv=False)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (singular_values[0] if singular_values.size else 0.0)
    rank = int(np.sum(singular_values > tol))
    return rank, singular_values, float(tol)


def null_space_svd(M: np.ndarray, tol: float | None = None) -> np.ndarray:
    """Approximate null space basis using SVD."""
    M = np.asarray(M, dtype=float)
    U, S, Vt = np.linalg.svd(M, full_matrices=True)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (S[0] if S.size else 0.0)
    rank = int(np.sum(S > tol))
    return Vt[rank:, :].T


def analyze_observability(A: np.ndarray, C: np.ndarray, name: str = "system") -> None:
    """Print observability matrix, rank, singular values, and unobservable directions."""
    O = observability_matrix(A, C)
    rank, singular_values, tol = numerical_rank(O)
    n = np.asarray(A).shape[0]
    N = null_space_svd(O, tol)

    print(f"\n{name}")
    print("-" * len(name))
    print("A =\n", np.asarray(A, dtype=float))
    print("C =\n", np.asarray(C, dtype=float))
    print("Observability matrix O_n =\n", O)
    print("Singular values =", singular_values)
    print("Tolerance =", tol)
    print(f"rank(O_n) = {rank} of n = {n}")
    print("Observable?", rank == n)
    if rank < n:
        print("Basis for unobservable initial-state directions:")
        print(N)


if __name__ == "__main__":
    # Example 1: position is measured in a double-integrator-like system.
    A1 = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
    C1 = np.array([[1.0, 0.0]])
    analyze_observability(A1, C1, "Example 1: observable double integrator")

    # Example 2: second state does not affect the output or its derivatives.
    A2 = np.array([[0.0, 0.0],
                   [0.0, -2.0]])
    C2 = np.array([[1.0, 0.0]])
    analyze_observability(A2, C2, "Example 2: unobservable second mode")

    # Example 3: same input matrix B would not change observability.
    B = np.array([[0.0],
                  [1.0]])
    print("\nB does not enter the Kalman observability rank test for (A, C).")
    print("B =\n", B)
