#!/usr/bin/env python3
"""
Chapter19_Lesson2.py
Observable/unobservable subspaces for a continuous-time LTI system.

Libraries covered:
  - NumPy: matrix powers, SVD rank, numerical null space.
  - SciPy / python-control can be used for larger control workflows, but this
    file intentionally implements the core observability operations from scratch.
"""
import numpy as np


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O_n = [C; C A; ...; C A^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def numerical_rank(M: np.ndarray, tol: float | None = None) -> int:
    """SVD numerical rank with MATLAB-like default tolerance."""
    singular_values = np.linalg.svd(M, compute_uv=False)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (singular_values[0] if singular_values.size else 1.0)
    return int(np.sum(singular_values > tol))


def nullspace(M: np.ndarray, tol: float | None = None) -> np.ndarray:
    """Orthonormal basis for ker(M), returned as columns."""
    U, S, Vt = np.linalg.svd(M, full_matrices=True)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (S[0] if S.size else 1.0)
    rank = int(np.sum(S > tol))
    return Vt[rank:, :].T.copy()


def row_space_basis(M: np.ndarray, tol: float | None = None) -> np.ndarray:
    """Orthonormal basis for range(M^T), returned as columns."""
    U, S, Vt = np.linalg.svd(M, full_matrices=False)
    if tol is None:
        tol = max(M.shape) * np.finfo(float).eps * (S[0] if S.size else 1.0)
    rank = int(np.sum(S > tol))
    return Vt[:rank, :].T.copy()


def a_invariance_residual(A: np.ndarray, N: np.ndarray) -> float:
    """
    For an orthonormal basis N of the unobservable subspace, compute
    ||(I - N N^T) A N||_F.  It should be near zero when ker(O_n) is A-invariant.
    """
    if N.size == 0:
        return 0.0
    n = A.shape[0]
    projector_perp = np.eye(n) - N @ N.T
    return float(np.linalg.norm(projector_perp @ A @ N, ord="fro"))


def observable_decomposition(A: np.ndarray, C: np.ndarray):
    """
    Build an orthonormal coordinate matrix Q = [Q_o Q_u], where Q_u spans the
    unobservable subspace and Q_o spans its orthogonal complement.  With x = Q z,
    the transformed pair has Cbar = [C_o 0] and Abar has a zero top-right block.
    """
    O = observability_matrix(A, C)
    Qu = nullspace(O)
    Qo = row_space_basis(O)

    # Complete the basis robustly if roundoff or rank decisions leave a gap.
    Q = np.hstack([Qo, Qu]) if Qu.size else Qo
    if Q.shape[1] < A.shape[0]:
        # QR completion from random directions projected against existing Q.
        rng = np.random.default_rng(0)
        R = rng.normal(size=(A.shape[0], A.shape[0] - Q.shape[1]))
        R = R - Q @ (Q.T @ R)
        extra, _ = np.linalg.qr(R)
        Q = np.hstack([Q, extra[:, : A.shape[0] - Q.shape[1]]])

    Abar = Q.T @ A @ Q
    Cbar = C @ Q
    return O, Qo, Qu, Q, Abar, Cbar


def main() -> None:
    # Two decoupled second-order modes.  The sensor measures only the first mode.
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [-2.0, -3.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, -5.0, -1.0],
    ])
    B = np.array([[0.0], [1.0], [0.0], [1.0]])
    C = np.array([[1.0, 0.0, 0.0, 0.0]])
    D = np.array([[0.0]])

    O, Qo, Qu, Q, Abar, Cbar = observable_decomposition(A, C)

    print("Observability matrix O_n:")
    print(O)
    print("rank(O_n) =", numerical_rank(O), "out of n =", A.shape[0])
    print("\nBasis for unobservable subspace ker(O_n), columns of Q_u:")
    print(Qu)
    print("A-invariance residual ||(I-Q_u Q_u^T) A Q_u||_F =", a_invariance_residual(A, Qu))
    print("\nTransformed Abar = Q^T A Q:")
    print(np.round(Abar, 10))
    print("\nTransformed Cbar = C Q:")
    print(np.round(Cbar, 10))

    # Demonstrate output indistinguishability: adding an unobservable initial state
    # produces no change in zero-input output y(t) = C exp(A t) x0.
    try:
        from scipy.linalg import expm
        x0_observable = np.array([1.0, 0.0, 0.0, 0.0])
        x0_hidden = np.array([0.0, 0.0, 1.0, -1.0])
        times = np.linspace(0.0, 5.0, 6)
        print("\nZero-input output comparison y(t):")
        for t in times:
            y1 = C @ expm(A * t) @ x0_observable
            y2 = C @ expm(A * t) @ (x0_observable + x0_hidden)
            print(f"t={t:4.1f}: y1={y1[0]: .8f}, y2={y2[0]: .8f}, difference={abs(y1[0]-y2[0]):.2e}")
    except ImportError:
        print("\nSciPy not installed; skipping expm-based output comparison.")


if __name__ == "__main__":
    main()
