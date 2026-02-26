"""
Chapter10_Lesson2.py
ICP Variants for Mobile Robots (2D LiDAR focus)

Implements:
  - Point-to-Point ICP in SE(2) (closed-form SVD update)
  - Point-to-Line ICP in SE(2) (linearized least squares)
  - Optional robust weighting (Huber) and trimming

Dependencies: numpy, scipy (cKDTree)
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any

try:
    from scipy.spatial import cKDTree
except ImportError as e:
    raise ImportError("This script requires scipy. Install with: pip install scipy") from e


@dataclass
class ICPResult:
    R: np.ndarray          # 2x2
    t: np.ndarray          # 2,
    theta: float
    history: Dict[str, Any]


def rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def se2_compose(R1, t1, R2, t2):
    """(R1,t1) o (R2,t2)"""
    R = R1 @ R2
    t = R1 @ t2 + t1
    return R, t


def huber_weights(r: np.ndarray, delta: float) -> np.ndarray:
    """Huber IRLS weights for residuals r (1D)."""
    a = np.abs(r)
    w = np.ones_like(r)
    mask = a > delta
    w[mask] = delta / (a[mask] + 1e-12)
    return w


def estimate_normals_2d(polyline: np.ndarray) -> np.ndarray:
    """
    Estimate 2D normals for ordered scan points.
    polyline: (M,2) assumed ordered by angle.
    """
    M = polyline.shape[0]
    prev = np.roll(polyline, 1, axis=0)
    nxt  = np.roll(polyline, -1, axis=0)
    tang = nxt - prev
    # normal is perpendicular to tangent: n = [-ty, tx]
    n = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    norm = np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    return n / norm


def nearest_neighbor(src_xy: np.ndarray, dst_xy: np.ndarray):
    tree = cKDTree(dst_xy)
    d, idx = tree.query(src_xy, k=1)
    return d, idx


def solve_point_to_point(src: np.ndarray, dst: np.ndarray, w: Optional[np.ndarray] = None):
    """
    Weighted 2D Procrustes: minimize sum_i w_i || R p_i + t - q_i ||^2
    Returns R,t.
    """
    if w is None:
        w = np.ones(src.shape[0])
    w = w.reshape(-1, 1)
    wsum = np.sum(w) + 1e-12
    pbar = np.sum(w * src, axis=0) / wsum
    qbar = np.sum(w * dst, axis=0) / wsum
    P = src - pbar
    Q = dst - qbar
    H = (w * P).T @ Q
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # enforce det(R)=+1
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = qbar - (R @ pbar)
    return R, t


def solve_point_to_line(src: np.ndarray, dst: np.ndarray, n: np.ndarray, R0: np.ndarray, t0: np.ndarray,
                        w: Optional[np.ndarray] = None):
    """
    Linearized point-to-line (2D analogue of point-to-plane):
      r_i = n_i^T ( R p_i + t - q_i )
    Around current pose (R0,t0), solve for delta = [dx, dy, dtheta].

    Returns (R_delta, t_delta) to be composed with (R0,t0).
    """
    if w is None:
        w = np.ones(src.shape[0])
    w = w.astype(float)

    # transform source with current estimate
    p = (R0 @ src.T).T + t0

    # residual r = n^T (p - q)
    e = p - dst
    r = np.sum(n * e, axis=1)

    # Jacobian wrt [dx, dy, dtheta]
    # dr/ddx = n_x, dr/ddy = n_y
    # dr/dtheta = n^T * d(R p_src)/dtheta.
    # For 2D, d/dtheta (R0 * p_src) = R0 * [[0,-1],[1,0]] p_src
    Jt = n  # (N,2)
    psrc = src
    perp = np.stack([-psrc[:, 1], psrc[:, 0]], axis=1)  # 90deg rotation of psrc
    Jtheta = np.sum(n * (R0 @ perp.T).T, axis=1)  # (N,)

    A = np.column_stack([Jt, Jtheta])  # (N,3)
    # Weighted least squares: minimize || sqrt(w)*(A delta + r) ||^2
    W = np.sqrt(w).reshape(-1, 1)
    Aw = W * A
    bw = W[:, 0] * r
    # Solve Aw delta = -bw
    delta, *_ = np.linalg.lstsq(Aw, -bw, rcond=None)
    dx, dy, dtheta = delta

    R_delta = rot2(dtheta)
    t_delta = np.array([dx, dy], dtype=float)
    return R_delta, t_delta, r


def icp_se2(
    src_xy: np.ndarray,
    dst_xy: np.ndarray,
    variant: str = "p2p",
    max_iter: int = 40,
    tol: float = 1e-6,
    reject_dist: Optional[float] = None,
    huber_delta: Optional[float] = None,
    trim_ratio: Optional[float] = None,
    init_theta: float = 0.0,
    init_t: Tuple[float, float] = (0.0, 0.0),
) -> ICPResult:
    """
    ICP in SE(2).
    variant: 'p2p' (point-to-point) or 'p2l' (point-to-line).
    trim_ratio: keep this fraction of smallest-distance correspondences (0<ratio<=1).
    """
    assert variant in ("p2p", "p2l")
    R = rot2(init_theta)
    t = np.array(init_t, dtype=float)

    hist_cost = []
    hist_inliers = []

    # Precompute "ordered" normals for dst (2D scan assumed ordered)
    dst_normals = estimate_normals_2d(dst_xy) if variant == "p2l" else None

    prev_cost = np.inf
    for it in range(max_iter):
        src_trans = (R @ src_xy.T).T + t
        d, idx = nearest_neighbor(src_trans, dst_xy)
        matched = dst_xy[idx]

        # basic rejection
        mask = np.ones_like(d, dtype=bool)
        if reject_dist is not None:
            mask &= (d <= reject_dist)

        # trimming by distance
        if trim_ratio is not None:
            assert 0.0 < trim_ratio <= 1.0
            # apply trim among currently accepted
            cand = np.where(mask)[0]
            if cand.size > 0:
                k = max(3, int(np.floor(trim_ratio * cand.size)))
                order = np.argsort(d[cand])[:k]
                new_mask = np.zeros_like(mask)
                new_mask[cand[order]] = True
                mask = new_mask

        if np.sum(mask) < 3:
            break

        P = src_xy[mask]
        Q = matched[mask]

        if variant == "p2p":
            w = None
            if huber_delta is not None:
                # residual vector norm
                e = (R @ P.T).T + t - Q
                rnorm = np.linalg.norm(e, axis=1)
                w = huber_weights(rnorm, huber_delta)
            R_new, t_new = solve_point_to_point(P, Q, w=w)
            R, t = R_new, t_new
            # compute cost
            e = (R @ P.T).T + t - Q
            cost = float(np.mean(np.sum(e * e, axis=1)))

        else:  # p2l
            n = dst_normals[idx][mask]
            w = np.ones(P.shape[0])
            if huber_delta is not None:
                # line residuals after linearization r (computed inside solve)
                pass
            R_delta, t_delta, r = solve_point_to_line(P, Q, n, R, t, w=w)
            R, t = se2_compose(R_delta, t_delta, R, t)
            # robust reweight (optional): one IRLS outer step
            if huber_delta is not None:
                # recompute residual with updated pose
                p_upd = (R @ P.T).T + t
                r2 = np.sum(n * (p_upd - Q), axis=1)
                w2 = huber_weights(r2, huber_delta)
                R_delta, t_delta, _ = solve_point_to_line(P, Q, n, R, t, w=w2)
                R, t = se2_compose(R_delta, t_delta, R, t)
                p_upd = (R @ P.T).T + t
                r2 = np.sum(n * (p_upd - Q), axis=1)
                cost = float(np.mean(r2 * r2))
            else:
                cost = float(np.mean(r * r))

        hist_cost.append(cost)
        hist_inliers.append(int(np.sum(mask)))

        if np.abs(prev_cost - cost) < tol:
            break
        prev_cost = cost

    theta = float(np.arctan2(R[1, 0], R[0, 0]))
    return ICPResult(R=R, t=t, theta=theta, history={"cost": hist_cost, "inliers": hist_inliers})


def demo():
    """
    Synthetic demo: create a 2D "scan", transform it, add noise, then recover pose.
    """
    np.random.seed(2)
    # "scan" points on a rounded rectangle
    t1 = np.linspace(0, 2*np.pi, 300)
    dst = np.column_stack([2*np.cos(t1) + 0.3*np.cos(5*t1),
                           1.0*np.sin(t1) + 0.2*np.sin(3*t1)])
    true_theta = 0.25
    true_t = np.array([0.8, -0.4])
    src = (rot2(true_theta) @ dst.T).T + true_t
    src += 0.01*np.random.randn(*src.shape)

    # recover with p2l (faster locally)
    res = icp_se2(src, dst, variant="p2l", max_iter=60, reject_dist=0.5, huber_delta=0.05,
                  trim_ratio=0.9, init_theta=0.0, init_t=(0.0, 0.0))
    print("Estimated theta:", res.theta, "t:", res.t)
    print("True theta:", true_theta, "t:", true_t)

    # Optional: plot if matplotlib is available
    try:
        import matplotlib.pyplot as plt
        src_aligned = (res.R @ src.T).T + res.t
        plt.figure()
        plt.plot(dst[:,0], dst[:,1], '.', label="dst (reference)")
        plt.plot(src[:,0], src[:,1], '.', label="src (raw)")
        plt.plot(src_aligned[:,0], src_aligned[:,1], '.', label="src aligned")
        plt.axis('equal'); plt.legend(); plt.title("ICP alignment (2D)")
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    demo()
