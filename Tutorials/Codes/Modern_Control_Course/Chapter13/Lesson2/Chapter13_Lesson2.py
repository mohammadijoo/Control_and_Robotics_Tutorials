# Chapter13_Lesson2.py
# Observable states and observable subspace for a continuous-time LTI system.
# Dependencies: numpy only. Optional modern-control packages: scipy.signal and python-control
# can be used for state-space simulation, but the subspace computation below is from scratch.

import numpy as np


def observability_signature_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
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


def numerical_rank(M: np.ndarray, tol: float = 1e-10) -> int:
    """SVD-based numerical rank."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def null_space(M: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """Return an orthonormal basis for the null space of M as columns."""
    _, s, vh = np.linalg.svd(M, full_matrices=True)
    rank = int(np.sum(s > tol))
    return vh[rank:].T


def observable_projection(A: np.ndarray, C: np.ndarray, tol: float = 1e-10):
    """Compute the observable and unobservable subspace projectors.

    The unobservable subspace is ker(O). The observable subspace, in the
    standard Euclidean inner product, is ker(O)^perp = range(O.T).
    """
    O = observability_signature_matrix(A, C)
    N = null_space(O, tol)
    # SVD row-space basis for range(O.T)
    u, s, vh = np.linalg.svd(O, full_matrices=True)
    rank = int(np.sum(s > tol))
    V_o = vh[:rank].T
    P_o = V_o @ V_o.T
    P_u = N @ N.T if N.size else np.zeros((A.shape[0], A.shape[0]))
    return O, V_o, N, P_o, P_u


def output_signature(A: np.ndarray, C: np.ndarray, x0: np.ndarray) -> np.ndarray:
    """Finite derivative signature [C x0, C A x0, ..., C A^(n-1) x0]."""
    O = observability_signature_matrix(A, C)
    return O @ np.asarray(x0, dtype=float)


if __name__ == "__main__":
    # Example: x3 is dynamically separated from the measured coordinates.
    A = np.array([[0.0, 1.0, 0.0],
                  [-2.0, -3.0, 0.0],
                  [0.0, 0.0, -4.0]])
    C = np.array([[1.0, 0.0, 0.0]])

    O, V_o, N_u, P_o, P_u = observable_projection(A, C)

    print("Observability signature matrix O:")
    print(O)
    print("rank(O) =", numerical_rank(O))
    print("Observable subspace basis columns (range of O.T):")
    print(V_o)
    print("Unobservable subspace basis columns (ker O):")
    print(N_u)

    x0 = np.array([2.0, -1.0, 5.0])
    x_obs = P_o @ x0
    x_unobs = P_u @ x0
    print("x0 =", x0)
    print("observable component =", x_obs)
    print("unobservable component =", x_unobs)
    print("finite output derivative signature of x0 =", output_signature(A, C, x0))
    print("signature of observable component =", output_signature(A, C, x_obs))
    print("signature of unobservable component =", output_signature(A, C, x_unobs))
