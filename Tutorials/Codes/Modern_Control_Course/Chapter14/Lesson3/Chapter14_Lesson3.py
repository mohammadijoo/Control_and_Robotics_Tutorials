# Chapter14_Lesson3.py
# Duality Between Controllability and Observability
# Requires: numpy, scipy (optional only for comparison with control libraries)

import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return Ctrb(A,B) = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return Obsv(A,C) = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def rank(M: np.ndarray, tol: float = 1e-10) -> int:
    """Numerical rank from singular values."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def report_duality(A: np.ndarray, B: np.ndarray, C: np.ndarray) -> None:
    """Print primal and dual controllability/observability ranks."""
    n = A.shape[0]

    Ctrb = controllability_matrix(A, B)
    Obsv = observability_matrix(A, C)

    # Dual system: A_d = A.T, B_d = C.T, C_d = B.T
    Ctrb_dual = controllability_matrix(A.T, C.T)
    Obsv_dual = observability_matrix(A.T, B.T)

    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)

    print("\nrank Ctrb(A,B) =", rank(Ctrb), "of", n)
    print("rank Obsv(A,C) =", rank(Obsv), "of", n)

    print("\nDuality checks:")
    print("Obsv(A,C) == Ctrb(A.T,C.T).T ?",
          np.allclose(Obsv, Ctrb_dual.T))
    print("Ctrb(A,B).T == Obsv(A.T,B.T) ?",
          np.allclose(Ctrb.T, Obsv_dual))

    print("\nrank Ctrb(A.T,C.T) =", rank(Ctrb_dual), "of", n)
    print("rank Obsv(A.T,B.T) =", rank(Obsv_dual), "of", n)


if __name__ == "__main__":
    # Example: a third-order SISO realization
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [-2.0, -3.0, -4.0]])
    B = np.array([[0.0],
                  [0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0, 0.0]])

    report_duality(A, B, C)
