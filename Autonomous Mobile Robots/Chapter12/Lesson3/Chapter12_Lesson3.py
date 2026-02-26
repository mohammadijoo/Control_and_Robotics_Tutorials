#!/usr/bin/env python3
# Chapter12_Lesson3.py
# Pose Graph Optimization in SE(2) via Gauss-Newton (from scratch, educational)
#
# Dependencies:
#   numpy
#   scipy  (sparse linear algebra)
#
# Run:
#   python Chapter12_Lesson3.py

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import spsolve

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)

@dataclass
class EdgeSE2:
    i: int
    j: int
    z: np.ndarray      # measurement [dx, dy, dtheta] from i to j (in i-frame)
    Omega: np.ndarray  # information matrix (3x3), inverse of covariance

def predict_relative(xi: np.ndarray, xj: np.ndarray) -> np.ndarray:
    """
    Predicted relative pose from i to j:
      zhat = inv(xi) ⊕ xj
    where pose x = [x, y, theta].
    """
    ti = xi[:2]
    tj = xj[:2]
    Ri = rot2(xi[2])
    dt = Ri.T @ (tj - ti)
    dtheta = wrap_angle(xj[2] - xi[2])
    return np.array([dt[0], dt[1], dtheta], dtype=float)

def se2_error(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> np.ndarray:
    """
    Simple SE(2) 'between' error:
      e = inv(z) ⊕ (inv(xi) ⊕ xj)
    implemented using a minimal (translation + angle) approximation.
    """
    zhat = predict_relative(xi, xj)
    Rz = rot2(z[2])
    terr = Rz.T @ (zhat[:2] - z[:2])
    therr = wrap_angle(zhat[2] - z[2])
    return np.array([terr[0], terr[1], therr], dtype=float)

def se2_jacobians(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Analytic Jacobians for the simple between error used in se2_error.

    e_t = Rz^T ( Ri^T (t_j - t_i) - t_z )
    e_th = (theta_j - theta_i) - theta_z  (wrapped)

    Returns (A, B) where:
      A = de/dxi  (3x3), B = de/dxj  (3x3)
    """
    ti = xi[:2]
    tj = xj[:2]
    Ri = rot2(xi[2])
    Rz = rot2(z[2])

    # A2 = Rz^T Ri^T
    A2 = Rz.T @ Ri.T

    # skew-symmetric matrix for 2D cross product
    S = np.array([[0.0, -1.0],
                  [1.0,  0.0]])

    dt = (tj - ti)

    A = np.zeros((3, 3), dtype=float)
    B = np.zeros((3, 3), dtype=float)

    # translation derivatives
    A[0:2, 0:2] = -A2
    B[0:2, 0:2] =  A2

    # derivative wrt theta_i affects translation part via Ri^T
    # d(Ri^T dt)/dtheta_i = -Ri^T S dt
    dti_dtheta = Rz.T @ (-Ri.T @ (S @ dt))
    A[0:2, 2] = dti_dtheta

    # theta derivatives
    A[2, 2] = -1.0
    B[2, 2] =  1.0

    return A, B

def build_normal_equations(
    X: np.ndarray,
    edges: List[EdgeSE2],
    anchor_first_pose: bool = True
) -> Tuple[csr_matrix, np.ndarray, float]:
    """
    Build sparse normal equations H dx = -b for Gauss-Newton.

    If anchor_first_pose=True, pose 0 is held fixed (gauge fixing) and variables
    are only for poses 1..N-1.
    """
    N = X.shape[0]
    dim = 3 * (N - 1) if anchor_first_pose else 3 * N

    H = lil_matrix((dim, dim), dtype=float)
    b = np.zeros(dim, dtype=float)
    chi2 = 0.0

    def idx(pose_id: int) -> int:
        if anchor_first_pose:
            return 3 * (pose_id - 1)  # pose 1 -> 0
        return 3 * pose_id

    for e in edges:
        xi = X[e.i]
        xj = X[e.j]
        r = se2_error(xi, xj, e.z)
        A, B = se2_jacobians(xi, xj, e.z)

        Omega = e.Omega
        chi2 += float(r.T @ Omega @ r)

        # local blocks
        AtOmega = A.T @ Omega
        BtOmega = B.T @ Omega
        Hii = AtOmega @ A
        Hij = AtOmega @ B
        Hji = BtOmega @ A
        Hjj = BtOmega @ B
        bi = AtOmega @ r
        bj = BtOmega @ r

        # scatter-add into global H and b, skipping anchored pose if needed
        for (p, q, Hpq) in [(e.i, e.i, Hii), (e.i, e.j, Hij), (e.j, e.i, Hji), (e.j, e.j, Hjj)]:
            if anchor_first_pose and (p == 0 or q == 0):
                continue
            ip = idx(p)
            iq = idx(q)
            for a in range(3):
                for bb in range(3):
                    H[ip + a, iq + bb] += Hpq[a, bb]

        # b terms
        if not (anchor_first_pose and e.i == 0):
            ii = idx(e.i)
            b[ii:ii+3] += bi
        if not (anchor_first_pose and e.j == 0):
            jj = idx(e.j)
            b[jj:jj+3] += bj

    return H.tocsr(), b, chi2

def apply_increment(X: np.ndarray, dx: np.ndarray, anchor_first_pose: bool = True) -> np.ndarray:
    """Apply increments to poses; theta updated additively and wrapped."""
    X_new = X.copy()
    if anchor_first_pose:
        for k in range(1, X.shape[0]):
            d = dx[3*(k-1):3*(k-1)+3]
            X_new[k, 0] += d[0]
            X_new[k, 1] += d[1]
            X_new[k, 2] = wrap_angle(X_new[k, 2] + d[2])
    else:
        for k in range(X.shape[0]):
            d = dx[3*k:3*k+3]
            X_new[k, 0] += d[0]
            X_new[k, 1] += d[1]
            X_new[k, 2] = wrap_angle(X_new[k, 2] + d[2])
    return X_new

def gauss_newton(
    X0: np.ndarray,
    edges: List[EdgeSE2],
    iters: int = 10,
    damping: float = 0.0,
    anchor_first_pose: bool = True
) -> np.ndarray:
    X = X0.copy()
    for it in range(iters):
        H, b, chi2 = build_normal_equations(X, edges, anchor_first_pose=anchor_first_pose)
        if damping > 0.0:
            H = H + damping * csr_matrix(np.eye(H.shape[0]))

        dx = spsolve(H, -b)
        X = apply_increment(X, dx, anchor_first_pose=anchor_first_pose)

        print(f"iter={it:02d}  chi2={chi2:.6f}  |dx|={np.linalg.norm(dx):.3e}")

        if np.linalg.norm(dx) < 1e-8:
            break
    return X

def make_synthetic_pose_graph(N: int = 20, noise_xy: float = 0.02, noise_th: float = 0.01):
    """
    Build a simple chain with one loop-closure edge.
    Ground truth: gentle arc.
    Measurements: relative odometry edges (i,i+1) + loop edge (0,N-1).
    """
    X_true = np.zeros((N, 3), dtype=float)
    for k in range(1, N):
        X_true[k, 0] = X_true[k-1, 0] + 0.5 * np.cos(0.1 * k)
        X_true[k, 1] = X_true[k-1, 1] + 0.5 * np.sin(0.1 * k)
        X_true[k, 2] = wrap_angle(0.05 * k)

    edges = []
    Sigma = np.diag([noise_xy**2, noise_xy**2, noise_th**2])
    Omega = np.linalg.inv(Sigma)

    rng = np.random.default_rng(7)

    for k in range(N-1):
        z = predict_relative(X_true[k], X_true[k+1])
        z_noisy = z + rng.normal(0.0, [noise_xy, noise_xy, noise_th])
        z_noisy[2] = wrap_angle(z_noisy[2])
        edges.append(EdgeSE2(i=k, j=k+1, z=z_noisy, Omega=Omega))

    zlc = predict_relative(X_true[0], X_true[N-1])
    zlc_noisy = zlc + rng.normal(0.0, [noise_xy, noise_xy, noise_th])
    zlc_noisy[2] = wrap_angle(zlc_noisy[2])
    edges.append(EdgeSE2(i=0, j=N-1, z=zlc_noisy, Omega=Omega))

    X0 = np.zeros_like(X_true)
    for k in range(N-1):
        z = edges[k].z
        Rk = rot2(X0[k,2])
        X0[k+1,:2] = X0[k,:2] + (Rk @ z[:2])
        X0[k+1,2] = wrap_angle(X0[k,2] + z[2])

    return X0, edges

def main():
    X0, edges = make_synthetic_pose_graph(N=30)
    print("Optimizing pose graph (Gauss-Newton, anchored pose 0)...")
    X_opt = gauss_newton(X0, edges, iters=15, damping=0.0, anchor_first_pose=True)

    print("\nFirst 5 poses (x,y,theta) initial vs optimized:")
    for k in range(5):
        print(f"k={k:02d}  X0={X0[k]}  Xopt={X_opt[k]}")

if __name__ == "__main__":
    main()
