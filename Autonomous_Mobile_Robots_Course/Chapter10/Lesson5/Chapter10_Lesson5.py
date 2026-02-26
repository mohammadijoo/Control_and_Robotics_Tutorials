# Chapter10_Lesson5.py
"""
Autonomous Mobile Robots — Chapter 10, Lesson 5
Lab: ICP-Based Motion Estimation (2D)

This script implements a robust 2D point-to-point ICP to estimate relative motion
between two LiDAR scans (represented as 2D point sets).

Dependencies: numpy, scipy (optional but recommended for KD-tree)
Run: python Chapter10_Lesson5.py
"""

from __future__ import annotations
import math
import numpy as np

try:
    from scipy.spatial import cKDTree as KDTree
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False
    KDTree = None


def rot2(theta: float) -> np.ndarray:
    """2D rotation matrix."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def apply_transform(P: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Apply rigid transform: P' = R P + t."""
    return (R @ P.T).T + t.reshape(1, 2)


def huber_weights(r: np.ndarray, delta: float) -> np.ndarray:
    """Huber weights on residual norms."""
    # r: (N,) residual magnitudes
    w = np.ones_like(r)
    mask = r > delta
    w[mask] = delta / (r[mask] + 1e-12)
    return w


def weighted_centroid(P: np.ndarray, w: np.ndarray) -> np.ndarray:
    wsum = float(np.sum(w)) + 1e-12
    return (w.reshape(-1, 1) * P).sum(axis=0) / wsum


def best_fit_transform_2d(src: np.ndarray,
                          dst: np.ndarray,
                          w: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute optimal 2D rigid transform (R,t) minimizing sum_i w_i || R src_i + t - dst_i ||^2.
    Uses SVD (Kabsch) on 2x2 cross-covariance.
    """
    assert src.shape == dst.shape and src.shape[1] == 2
    N = src.shape[0]
    if w is None:
        w = np.ones(N, dtype=float)

    mu_s = weighted_centroid(src, w)
    mu_d = weighted_centroid(dst, w)

    X = src - mu_s
    Y = dst - mu_d

    # Weighted cross-covariance
    H = (w.reshape(-1, 1) * X).T @ Y  # 2x2

    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation (det = +1)
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1.0
        R = Vt.T @ U.T

    t = mu_d - R @ mu_s
    return R, t


def nearest_neighbors(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    For each src point, find nearest neighbor in dst.
    Returns:
        nn: (N,2) matched dst points
        d2: (N,) squared distances
    """
    if _HAS_SCIPY:
        tree = KDTree(dst)
        d, idx = tree.query(src, k=1)
        return dst[idx], d**2
    # Fallback: brute-force (O(N^2))
    nn = np.empty_like(src)
    d2 = np.empty(src.shape[0], dtype=float)
    for i, p in enumerate(src):
        diffs = dst - p.reshape(1, 2)
        di2 = np.sum(diffs * diffs, axis=1)
        j = int(np.argmin(di2))
        nn[i] = dst[j]
        d2[i] = di2[j]
    return nn, d2


def icp_2d(src: np.ndarray,
           dst: np.ndarray,
           init_R: np.ndarray | None = None,
           init_t: np.ndarray | None = None,
           max_iter: int = 50,
           tol: float = 1e-6,
           reject_dist: float | None = None,
           trim_fraction: float | None = 0.8,
           huber_delta: float | None = 0.25) -> dict:
    """
    Robust 2D ICP (point-to-point).

    Args:
        src, dst: (N,2) and (M,2) point sets.
        init_R, init_t: optional initial guess mapping src -> dst.
        reject_dist: if set, reject correspondences with distance > reject_dist.
        trim_fraction: keep best fraction of correspondences by distance (0,1].
        huber_delta: if set, apply Huber weights based on residual norms.

    Returns: dict with R, t, history, rmse, iters.
    """
    src0 = np.array(src, dtype=float)
    dst0 = np.array(dst, dtype=float)

    if init_R is None:
        R = np.eye(2)
    else:
        R = np.array(init_R, dtype=float)

    if init_t is None:
        t = np.zeros(2)
    else:
        t = np.array(init_t, dtype=float).reshape(2,)

    history = []
    prev_rmse = None

    for it in range(max_iter):
        src_trans = apply_transform(src0, R, t)
        nn, d2 = nearest_neighbors(src_trans, dst0)
        d = np.sqrt(d2 + 1e-12)

        # Optional distance gating
        mask = np.ones(src_trans.shape[0], dtype=bool)
        if reject_dist is not None:
            mask &= (d <= reject_dist)

        # Optional trimming (keep smallest distances)
        if trim_fraction is not None and 0.0 < trim_fraction < 1.0:
            idx_sorted = np.argsort(d)
            k = max(3, int(trim_fraction * len(d)))
            keep = np.zeros_like(mask)
            keep[idx_sorted[:k]] = True
            mask &= keep

        src_m = src_trans[mask]
        nn_m = nn[mask]
        if src_m.shape[0] < 3:
            raise RuntimeError("Too few correspondences after filtering. Relax thresholds.")

        # Robust weights
        w = None
        if huber_delta is not None and huber_delta > 0:
            w = huber_weights(np.linalg.norm(src_m - nn_m, axis=1), huber_delta)

        dR, dt = best_fit_transform_2d(src_m, nn_m, w=w)

        # Compose: new transform maps original src0 -> dst0
        R = dR @ R
        t = dR @ t + dt

        rmse = float(np.sqrt(np.mean(np.sum((apply_transform(src0, R, t) - nearest_neighbors(apply_transform(src0, R, t), dst0)[0])**2, axis=1))))
        history.append(rmse)

        if prev_rmse is not None and abs(prev_rmse - rmse) < tol:
            break
        prev_rmse = rmse

    return {"R": R, "t": t, "rmse": history[-1], "history": history, "iters": len(history)}


def simulate_world_points(n: int = 400, seed: int = 7) -> np.ndarray:
    """
    Create a structured point set (walls + scatter) to mimic a 2D LiDAR environment.
    """
    rng = np.random.default_rng(seed)
    # Two perpendicular walls (an "L" corner)
    wall1_x = rng.uniform(0.0, 8.0, size=n//4)
    wall1_y = np.zeros_like(wall1_x)
    wall2_y = rng.uniform(0.0, 6.0, size=n//4)
    wall2_x = np.zeros_like(wall2_y)
    # Random scatter points
    scat = rng.uniform([0.0, 0.0], [8.0, 6.0], size=(n//2, 2))
    P = np.vstack([
        np.stack([wall1_x, wall1_y], axis=1),
        np.stack([wall2_x, wall2_y], axis=1),
        scat
    ])
    return P


def scan_from_pose(world_pts: np.ndarray, R_wb: np.ndarray, t_wb: np.ndarray,
                   noise_std: float = 0.01,
                   max_range: float = 10.0,
                   fov: float = math.radians(270.0),
                   seed: int = 0) -> np.ndarray:
    """
    Generate a 2D "scan" in the body (sensor) frame from world points.
    Pose is (R_wb, t_wb): world = R_wb * body + t_wb.
    Points returned are p_b = R_wb^T (p_w - t_wb), then filtered by range and FOV.
    """
    rng = np.random.default_rng(seed)
    R_bw = R_wb.T
    P_b = (R_bw @ (world_pts - t_wb.reshape(1, 2)).T).T

    r = np.linalg.norm(P_b, axis=1)
    ang = np.arctan2(P_b[:, 1], P_b[:, 0])

    # FOV symmetric around forward axis
    mask = (r <= max_range) & (np.abs(ang) <= fov / 2.0)

    P = P_b[mask].copy()
    P += rng.normal(0.0, noise_std, size=P.shape)

    # Add a few outliers (dynamic obstacles / spurious returns)
    k_out = max(5, P.shape[0] // 40)
    out = rng.uniform([-2.0, -2.0], [2.0, 2.0], size=(k_out, 2))
    P = np.vstack([P, out])
    return P


def main():
    # Create a world and two robot poses
    world = simulate_world_points(n=600, seed=7)

    # Pose k (identity)
    R0 = rot2(0.0)
    t0 = np.array([0.0, 0.0])

    # Pose k+1 (ground truth motion)
    theta_gt = math.radians(6.0)     # 6 degrees yaw
    t_gt = np.array([0.35, 0.10])    # 35 cm forward, 10 cm left (in world)

    R1 = rot2(theta_gt)
    t1 = t_gt.copy()

    # Generate scans in body frames
    scan_k  = scan_from_pose(world, R0, t0, noise_std=0.01, seed=1)
    scan_k1 = scan_from_pose(world, R1, t1, noise_std=0.01, seed=2)

    # Estimate motion that maps scan_{k+1} -> scan_k (should recover R1, t1)
    # Provide a weak initial guess (e.g., from wheel odometry)
    init_R = rot2(math.radians(3.0))
    init_t = np.array([0.20, 0.00])

    res = icp_2d(
        src=scan_k1,
        dst=scan_k,
        init_R=init_R,
        init_t=init_t,
        max_iter=60,
        tol=1e-7,
        reject_dist=0.5,
        trim_fraction=0.85,
        huber_delta=0.20
    )

    R_est, t_est = res["R"], res["t"]

    def rot_to_angle(R):
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    print("=== ICP-Based Motion Estimation (2D) ===")
    print(f"Ground truth: theta = {math.degrees(theta_gt):.3f} deg, t = {t_gt}")
    print(f"Estimated   : theta = {rot_to_angle(R_est):.3f} deg, t = {t_est}")
    print(f"Iterations  : {res['iters']}, final RMSE ≈ {res['rmse']:.6f}")
    print(f"Used SciPy KDTree: {_HAS_SCIPY}")

    # Simple sanity check: transform scan_k1 into frame k and compute mean NN distance
    scan_k1_to_k = apply_transform(scan_k1, R_est, t_est)
    nn, _ = nearest_neighbors(scan_k1_to_k, scan_k)
    mean_err = np.mean(np.linalg.norm(scan_k1_to_k - nn, axis=1))
    print(f"Mean NN error after alignment: {mean_err:.6f} m")


if __name__ == "__main__":
    main()
