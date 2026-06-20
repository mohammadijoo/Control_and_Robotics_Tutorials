# Chapter19_Lesson4.py
# Identification of a minimal realization via sequential Kalman decomposition.
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import block_diag


def controllability_matrix(A, B):
    """Return Wc = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def observability_matrix(A, C):
    """Return Wo = [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def svd_rank(M, tol=1e-10):
    s = np.linalg.svd(M, compute_uv=False)
    if s.size == 0:
        return 0
    return int(np.sum(s > tol * max(M.shape) * max(s[0], 1.0)))


def kalman_minimal_realization(A, B, C, D, tol=1e-10):
    """
    Reduce a continuous-time LTI realization to a minimal one by:
      1. retaining only the reachable subspace,
      2. quotienting/removing the unobservable subspace of that reachable part.

    The construction uses orthonormal bases from SVD, so transformations are
    similarity transformations with T^{-1} = T.T whenever T is square.
    """
    n = A.shape[0]

    # Step 1: reachable reduction.
    Wc = controllability_matrix(A, B)
    Uc, sc, _ = np.linalg.svd(Wc, full_matrices=True)
    rc = svd_rank(Wc, tol)
    Tc = Uc  # first rc columns span reachable subspace
    Ac = Tc.T @ A @ Tc
    Bc = Tc.T @ B
    Cc = C @ Tc
    Ar = Ac[:rc, :rc]
    Br = Bc[:rc, :]
    Cr = Cc[:, :rc]

    # Step 2: observable reduction of the reachable subsystem.
    Wo = observability_matrix(Ar, Cr)
    Uo, so, Vho = np.linalg.svd(Wo, full_matrices=True)
    ro = svd_rank(Wo, tol)
    V = Vho.T

    # V[:, ro:] spans the unobservable subspace; V[:, :ro] is an orthonormal
    # complement. Put unobservable coordinates first, observable coordinates last.
    To = np.hstack([V[:, ro:], V[:, :ro]]) if ro < rc else V[:, :ro]
    Ao = To.T @ Ar @ To
    Bo = To.T @ Br
    Co = Cr @ To

    n_unobs = rc - ro
    Amin = Ao[n_unobs:, n_unobs:]
    Bmin = Bo[n_unobs:, :]
    Cmin = Co[:, n_unobs:]

    return {
        "Amin": Amin,
        "Bmin": Bmin,
        "Cmin": Cmin,
        "Dmin": D.copy(),
        "rank_controllability": rc,
        "rank_observability_after_reachable": ro,
        "Tc": Tc,
        "To": To,
        "reachable_realization": (Ar, Br, Cr, D.copy()),
    }


def transfer_value(A, B, C, D, s):
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def demo():
    np.set_printoptions(precision=6, suppress=True)

    # Transparent nonminimal realization: first two states are controllable and observable.
    Aco = np.array([[0.0, 1.0], [-2.0, -3.0]])
    Bco = np.array([[0.0], [1.0]])
    Cco = np.array([[1.0, 0.0]])

    # Hidden modes: controllable-unobservable and uncontrollable-observable.
    A0 = block_diag(Aco, np.array([[-4.0]]), np.array([[-5.0]]))
    B0 = np.array([[0.0], [1.0], [1.0], [0.0]])
    C0 = np.array([[1.0, 0.0, 0.0, 2.0]])
    D0 = np.array([[0.0]])

    # Hide the structure with an orthogonal similarity transformation.
    rng = np.random.default_rng(19)
    Q, _ = np.linalg.qr(rng.normal(size=(4, 4)))
    A = Q @ A0 @ Q.T
    B = Q @ B0
    C = C0 @ Q.T
    D = D0

    result = kalman_minimal_realization(A, B, C, D)
    Amin, Bmin, Cmin, Dmin = result["Amin"], result["Bmin"], result["Cmin"], result["Dmin"]

    print("rank controllability =", result["rank_controllability"])
    print("rank observability after reachable reduction =", result["rank_observability_after_reachable"])
    print("Amin =\n", Amin)
    print("Bmin =\n", Bmin)
    print("Cmin =\n", Cmin)

    for s in [0.0, 1.0, 2.0, 1.0 + 2.0j]:
        G_full = transfer_value(A, B, C, D, s)
        G_min = transfer_value(Amin, Bmin, Cmin, Dmin, s)
        print(f"s={s:>8}: G_full={G_full[0,0]: .8f}, G_min={G_min[0,0]: .8f}, error={abs(G_full[0,0]-G_min[0,0]):.2e}")


if __name__ == "__main__":
    demo()
