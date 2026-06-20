# Chapter20_Lesson5.py
"""
Numerical issues in realization and model reduction.

This script demonstrates:
1. numerical rank tests for controllability/observability,
2. balancing transformation from Lyapunov Gramians,
3. balanced truncation and a frequency-domain error check.

Dependencies:
    numpy scipy
Optional:
    matplotlib (only if you want plots)
"""

import numpy as np
from numpy.linalg import svd, cond, norm
from scipy.linalg import solve_continuous_lyapunov, cholesky


def ctrb(A, B):
    """Kalman controllability matrix [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ap = np.eye(n)
    for _ in range(1, n):
        Ap = Ap @ A
        blocks.append(Ap @ B)
    return np.hstack(blocks)


def obsv(A, C):
    """Kalman observability matrix [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = [C]
    Ap = np.eye(n)
    for _ in range(1, n):
        Ap = Ap @ A
        blocks.append(C @ Ap)
    return np.vstack(blocks)


def numerical_rank(M, rtol=None):
    """SVD-based numerical rank."""
    s = svd(M, compute_uv=False)
    if rtol is None:
        rtol = max(M.shape) * np.finfo(float).eps
    threshold = rtol * s[0] if s.size else 0.0
    return int(np.sum(s > threshold)), s, threshold


def continuous_gramians(A, B, C):
    """
    Solve continuous-time Lyapunov equations:
        A Wc + Wc A.T + B B.T = 0
        A.T Wo + Wo A + C.T C = 0
    Requires A Hurwitz.
    """
    Wc = solve_continuous_lyapunov(A, -(B @ B.T))
    Wo = solve_continuous_lyapunov(A.T, -(C.T @ C))
    Wc = 0.5 * (Wc + Wc.T)
    Wo = 0.5 * (Wo + Wo.T)
    return Wc, Wo


def low_rank_cholesky_psd(W, jitter=1e-12):
    """
    Cholesky factor for a positive semidefinite Gramian.
    Adds small diagonal jitter if roundoff produces tiny negative eigenvalues.
    """
    n = W.shape[0]
    try:
        return cholesky(W, lower=True)
    except Exception:
        return cholesky(W + jitter * np.eye(n), lower=True)


def balanced_realization(A, B, C, D):
    """
    Square-root balanced realization.
    If Wc = S S.T and Wo = R R.T, compute SVD:
        R.T S = U Sigma V.T
    Then:
        T    = S V Sigma^(-1/2)
        Tinv = Sigma^(-1/2) U.T R.T
    """
    Wc, Wo = continuous_gramians(A, B, C)
    S = low_rank_cholesky_psd(Wc)
    R = low_rank_cholesky_psd(Wo)

    U, hsv, Vt = svd(R.T @ S)
    inv_sqrt = np.diag(1.0 / np.sqrt(hsv))

    T = S @ Vt.T @ inv_sqrt
    Tinv = inv_sqrt @ U.T @ R.T

    Ab = Tinv @ A @ T
    Bb = Tinv @ B
    Cb = C @ T
    Db = D.copy()

    return Ab, Bb, Cb, Db, hsv, Wc, Wo, T, Tinv


def balanced_truncate(A, B, C, D, r):
    """Keep the first r balanced states."""
    Ab, Bb, Cb, Db, hsv, Wc, Wo, T, Tinv = balanced_realization(A, B, C, D)
    Ar = Ab[:r, :r]
    Br = Bb[:r, :]
    Cr = Cb[:, :r]
    Dr = Db
    return Ar, Br, Cr, Dr, hsv


def freq_response(A, B, C, D, w):
    """Evaluate G(jw)=C(jwI-A)^(-1)B + D for SISO or MIMO systems."""
    n = A.shape[0]
    I = np.eye(n)
    values = []
    for wi in w:
        G = C @ np.linalg.solve(1j * wi * I - A, B) + D
        values.append(G)
    return np.array(values)


def main():
    # A stable fourth-order example with weakly coupled fast states.
    A = np.array([
        [-0.20,  0.05,  0.00,  0.00],
        [ 0.00, -1.00,  0.10,  0.00],
        [ 0.00,  0.00, -8.00,  0.20],
        [ 0.00,  0.00,  0.00, -20.0],
    ], dtype=float)
    B = np.array([[1.0], [0.4], [0.05], [0.01]])
    C = np.array([[1.0, 0.3, 0.02, 0.005]])
    D = np.array([[0.0]])

    print("Condition number of A:", cond(A))

    Rc = ctrb(A, B)
    Ro = obsv(A, C)
    rank_c, sc, thc = numerical_rank(Rc)
    rank_o, so, tho = numerical_rank(Ro)

    print("\nControllability singular values:", sc)
    print("Numerical controllability rank:", rank_c, "threshold:", thc)
    print("\nObservability singular values:", so)
    print("Numerical observability rank:", rank_o, "threshold:", tho)

    Wc, Wo = continuous_gramians(A, B, C)
    print("\ncond(Wc):", cond(Wc))
    print("cond(Wo):", cond(Wo))

    Ar, Br, Cr, Dr, hsv = balanced_truncate(A, B, C, D, r=2)
    print("\nHankel singular values:", hsv)
    print("Reduced A:\n", Ar)
    print("Reduced B:\n", Br)
    print("Reduced C:\n", Cr)

    # Balanced truncation H-infinity error upper bound:
    # ||G-Gr||_inf <= 2 * sum(discarded Hankel singular values)
    bound = 2.0 * np.sum(hsv[2:])
    print("\nBalanced-truncation error bound:", bound)

    # Coarse numerical frequency-response check.
    w = np.logspace(-3, 3, 300)
    G = freq_response(A, B, C, D, w)
    Gr = freq_response(Ar, Br, Cr, Dr, w)
    err = np.max(np.abs(G[:, 0, 0] - Gr[:, 0, 0]))
    print("Coarse sampled max frequency error:", err)


if __name__ == "__main__":
    main()
