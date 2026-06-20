"""
Chapter19_Lesson1.py
Modern Control - Chapter 19, Lesson 1
Controllable/Uncontrollable Subspaces for continuous-time LTI systems.

Required packages:
    pip install numpy
Optional packages for broader control workflows:
    pip install scipy control slycot
"""

import numpy as np


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return Wc = [B, AB, ..., A^(n-1)B]."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def numerical_rank(M: np.ndarray, tol: float | None = None) -> tuple[int, np.ndarray, float]:
    """Return numerical rank, singular values, and tolerance."""
    s = np.linalg.svd(M, compute_uv=False)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (s[0] if len(s) else 1.0)
    return int(np.sum(s > tol)), s, tol


def controllability_decomposition(A: np.ndarray, B: np.ndarray, tol: float | None = None):
    """
    Construct an orthonormal coordinate matrix T = [Qc Qu].

    Qc spans the reachable/controllable subspace R(A,B).
    Qu spans the orthogonal complement used here as one possible coordinate
    complement. The complement itself is not unique.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    Wc = controllability_matrix(A, B)
    U, s, _ = np.linalg.svd(Wc, full_matrices=True)
    if tol is None:
        tol = max(Wc.shape) * np.finfo(float).eps * (s[0] if len(s) else 1.0)
    r = int(np.sum(s > tol))
    Qc = U[:, :r]
    Qu = U[:, r:n]
    T = np.hstack((Qc, Qu))

    # T is orthogonal because it was built from left singular vectors.
    Abar = T.T @ A @ T
    Bbar = T.T @ B
    return Wc, r, s, tol, Qc, Qu, T, Abar, Bbar


def print_matrix(name: str, M: np.ndarray, digits: int = 6) -> None:
    print(f"\n{name} =")
    print(np.array2string(M, precision=digits, suppress_small=True))


def main() -> None:
    # Example: x3 is an autonomous uncontrollable state.
    A = np.array([
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, -2.0],
    ])
    B = np.array([
        [0.0],
        [1.0],
        [0.0],
    ])

    Wc, r, s, tol, Qc, Qu, T, Abar, Bbar = controllability_decomposition(A, B)
    n = A.shape[0]

    print_matrix("A", A)
    print_matrix("B", B)
    print_matrix("Wc = [B AB ... A^(n-1)B]", Wc)
    print(f"\nSingular values: {s}")
    print(f"Numerical rank: {r} out of n = {n}; tolerance = {tol:.3e}")
    print("System is controllable." if r == n else "System is not controllable.")

    print_matrix("Qc: basis for controllable subspace", Qc)
    print_matrix("Qu: orthogonal complement basis", Qu)
    print_matrix("T = [Qc Qu]", T)
    print_matrix("Abar = T^T A T", Abar)
    print_matrix("Bbar = T^T B", Bbar)

    lower_left = Abar[r:, :r]
    lower_B = Bbar[r:, :]
    print_matrix("lower-left block Abar_uc", lower_left)
    print_matrix("lower block Bbar_u", lower_B)
    print("\nFor a correct reachable decomposition, both blocks should be numerically zero.")


if __name__ == "__main__":
    main()
