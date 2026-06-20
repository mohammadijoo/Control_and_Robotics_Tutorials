# Chapter10_Lesson4.py
# Robust scan matching (2D) under dynamic obstacles using IRLS-Huber + trimming
# Dependencies: numpy (only)

import numpy as np


def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def weighted_kabsch_2d(P, Q, w):
    # Solve: min_{R,t} sum_i w_i || R P_i + t - Q_i ||^2
    # P, Q: (N,2) arrays, w: (N,) nonnegative weights
    # Returns R (2x2), t (2,)
    w = np.asarray(w, dtype=float).reshape(-1)
    w_sum = np.sum(w) + 1e-12
    p_bar = (w[:, None] * P).sum(axis=0) / w_sum
    q_bar = (w[:, None] * Q).sum(axis=0) / w_sum

    X = P - p_bar
    Y = Q - q_bar

    # Cross-covariance S = sum w * (X)(Y)^T  (2x2)
    S = (w[:, None, None] * (X[:, :, None] @ Y[:, None, :])).sum(axis=0)

    # Closed-form 2D Kabsch: theta = atan2(S12 - S21, S11 + S22)
    num = S[0, 1] - S[1, 0]
    den = S[0, 0] + S[1, 1]
    theta = np.arctan2(num, den)
    R = rot2(theta)
    t = q_bar - R @ p_bar
    return R, t


def nearest_neighbors_bruteforce(A, B):
    # For each point in A (N,2), find closest in B (M,2).
    # Returns indices (N,), and squared distances (N,).
    diff = A[:, None, :] - B[None, :, :]
    d2 = np.sum(diff * diff, axis=2)
    idx = np.argmin(d2, axis=1)
    return idx, d2[np.arange(A.shape[0]), idx]


def huber_weights(r, delta):
    # r: residual norms (N,)
    # delta: Huber threshold
    # w = 1, if r <= delta
    #     delta/r, otherwise
    r = np.asarray(r, dtype=float)
    w = np.ones_like(r)
    mask = r > delta
    w[mask] = delta / (r[mask] + 1e-12)
    return w


def robust_icp_2d(source, target, iters=25, delta=0.25, keep_ratio=0.85):
    # Robust ICP using:
    #  - nearest neighbor correspondences
    #  - IRLS with Huber weights
    #  - trimming (keep only a ratio of best correspondences)
    # Returns R, t, and history dict.
    src = np.asarray(source, dtype=float)
    tgt = np.asarray(target, dtype=float)

    R = np.eye(2)
    t = np.zeros(2)

    hist = {"rmse": [], "inliers": []}

    for _ in range(iters):
        src_w = (src @ R.T) + t

        nn_idx, d2 = nearest_neighbors_bruteforce(src_w, tgt)
        Q = tgt[nn_idx]
        residuals = src_w - Q
        r = np.sqrt(np.sum(residuals * residuals, axis=1))

        # Trimming: keep smallest residuals
        thr = np.quantile(r, keep_ratio)
        inlier_mask = r <= thr
        P_in = src[inlier_mask]
        Q_in = Q[inlier_mask]
        r_in = r[inlier_mask]

        w = huber_weights(r_in, delta)

        dR, dt = weighted_kabsch_2d(P_in, Q_in, w)

        # Compose transforms: new (R,t) = (dR,dt) o (R,t)
        R = dR @ R
        t = dR @ t + dt

        rmse = float(np.sqrt(np.mean(r_in ** 2)))
        hist["rmse"].append(rmse)
        hist["inliers"].append(int(np.sum(inlier_mask)))

    return R, t, hist


def vanilla_icp_2d(source, target, iters=25):
    # Plain ICP (no robust weighting).
    src = np.asarray(source, dtype=float)
    tgt = np.asarray(target, dtype=float)

    R = np.eye(2)
    t = np.zeros(2)

    for _ in range(iters):
        src_w = (src @ R.T) + t
        nn_idx, _ = nearest_neighbors_bruteforce(src_w, tgt)
        Q = tgt[nn_idx]
        w = np.ones(src.shape[0])
        dR, dt = weighted_kabsch_2d(src, Q, w)
        R = dR @ R
        t = dR @ t + dt

    return R, t


def make_synthetic_scene(n_static=250, n_dyn=60, seed=7):
    # Create two scans:
    #  - static environment points
    #  - dynamic obstacle cluster that moves between scans
    #  - global robot motion between scans (unknown pose)
    rng = np.random.default_rng(seed)

    # Static points: circle + two lines
    ang = rng.uniform(0, 2*np.pi, size=n_static//2)
    circle = np.c_[2.0*np.cos(ang), 2.0*np.sin(ang)]
    line1 = np.c_[rng.uniform(-3, 3, size=n_static//4), -1.5*np.ones(n_static//4)]
    line2 = np.c_[1.5*np.ones(n_static//4), rng.uniform(-2, 2, size=n_static//4)]
    static = np.vstack([circle, line1, line2])

    # Dynamic obstacle cluster (e.g., a pedestrian)
    dyn_center_1 = np.array([-0.5, 0.8])
    dyn1 = dyn_center_1 + 0.15*rng.normal(size=(n_dyn, 2))

    # Robot motion between scans (true transform from scan1 -> scan2 frame)
    theta_true = np.deg2rad(12.0)
    t_true = np.array([0.35, -0.10])
    R_true = rot2(theta_true)

    # Scan1
    scan1 = np.vstack([static, dyn1])
    scan1 += 0.02*rng.normal(size=scan1.shape)

    # Dynamic obstacle moves
    dyn_center_2 = dyn_center_1 + np.array([0.55, -0.25])
    dyn2_world = dyn_center_2 + 0.15*rng.normal(size=(n_dyn, 2))

    # Scan2: static transformed, dynamic inconsistent
    static2 = (static @ R_true.T) + t_true
    dyn2 = dyn2_world

    scan2 = np.vstack([static2, dyn2])
    scan2 += 0.02*rng.normal(size=scan2.shape)

    return scan1, scan2, R_true, t_true


def pose_error(R_est, t_est, R_true, t_true):
    # rotation error angle
    dR = R_est @ R_true.T
    ang = np.arctan2(dR[1, 0], dR[0, 0])
    t_err = np.linalg.norm(t_est - t_true)
    return float(np.rad2deg(abs(ang))), float(t_err)


if __name__ == "__main__":
    scan1, scan2, R_true, t_true = make_synthetic_scene()

    # Vanilla ICP (often pulled toward moving cluster)
    R_v, t_v = vanilla_icp_2d(scan1, scan2, iters=25)
    aerr_v, terr_v = pose_error(R_v, t_v, R_true, t_true)

    # Robust ICP (Huber + trimming)
    R_r, t_r, hist = robust_icp_2d(scan1, scan2, iters=25, delta=0.18, keep_ratio=0.85)
    aerr_r, terr_r = pose_error(R_r, t_r, R_true, t_true)

    print("True theta(deg):", 12.0, " True t:", t_true)
    print("Vanilla ICP: angle_err(deg) =", aerr_v, " trans_err =", terr_v, " t_est =", t_v)
    print("Robust  ICP: angle_err(deg) =", aerr_r, " trans_err =", terr_r, " t_est =", t_r)
    print("Robust ICP RMSE history (first 5):", hist["rmse"][:5], " ... last:", hist["rmse"][-1])
