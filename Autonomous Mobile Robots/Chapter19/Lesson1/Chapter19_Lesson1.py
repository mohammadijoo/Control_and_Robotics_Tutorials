# Chapter19_Lesson1.py
# Metrics for Localization and SLAM (Autonomous Mobile Robots)
# Python reference implementation: SE(2) alignment, ATE, RPE, NEES/NIS, occupancy metrics

import numpy as np

def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def align_se2(gt_xy, est_xy):
    """Least-squares rigid alignment in 2D (rotation + translation, no scale)."""
    gt_xy = np.asarray(gt_xy, dtype=float)
    est_xy = np.asarray(est_xy, dtype=float)
    c_gt = gt_xy.mean(axis=0)
    c_est = est_xy.mean(axis=0)

    X = est_xy - c_est
    Y = gt_xy - c_gt

    # 2D orthogonal Procrustes via closed-form angle
    s = np.sum(X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0])
    c = np.sum(X[:, 0] * Y[:, 0] + X[:, 1] * Y[:, 1])
    theta = np.arctan2(s, c)

    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    t = c_gt - R @ c_est
    return R, t, theta

def apply_se2(xy, yaw, R, t, theta):
    xy = np.asarray(xy, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    xy_aligned = (R @ xy.T).T + t
    yaw_aligned = wrap_to_pi(yaw + theta)
    return xy_aligned, yaw_aligned

def ate_rmse(gt_xy, est_xy_aligned):
    e = np.asarray(gt_xy) - np.asarray(est_xy_aligned)
    return float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

def rpe(gt_xy, gt_yaw, est_xy, est_yaw, delta=1):
    gt_xy = np.asarray(gt_xy); est_xy = np.asarray(est_xy)
    gt_yaw = np.asarray(gt_yaw); est_yaw = np.asarray(est_yaw)
    trans_err = []
    rot_err = []
    for k in range(len(gt_xy) - delta):
        dg = gt_xy[k + delta] - gt_xy[k]
        de = est_xy[k + delta] - est_xy[k]
        # rotate into local frame not necessary for norm metric; norm is invariant
        trans_err.append(np.linalg.norm(dg - de))
        dpsi_g = wrap_to_pi(gt_yaw[k + delta] - gt_yaw[k])
        dpsi_e = wrap_to_pi(est_yaw[k + delta] - est_yaw[k])
        rot_err.append(abs(wrap_to_pi(dpsi_g - dpsi_e)))
    trans_err = np.array(trans_err)
    rot_err = np.array(rot_err)
    return {
        "rpe_trans_rmse": float(np.sqrt(np.mean(trans_err ** 2))),
        "rpe_rot_rmse_rad": float(np.sqrt(np.mean(rot_err ** 2))),
        "rpe_trans_series": trans_err,
        "rpe_rot_series": rot_err,
    }

def anees(errors, covariances):
    """Average normalized estimation error squared.
    errors: (N, n), covariances: (N, n, n)
    """
    errors = np.asarray(errors, dtype=float)
    covariances = np.asarray(covariances, dtype=float)
    vals = []
    for e, P in zip(errors, covariances):
        vals.append(float(e.T @ np.linalg.inv(P) @ e))
    vals = np.array(vals)
    return float(np.mean(vals)), vals

def nis(innovations, S_mats):
    innovations = np.asarray(innovations, dtype=float)
    S_mats = np.asarray(S_mats, dtype=float)
    vals = []
    for nu, S in zip(innovations, S_mats):
        vals.append(float(nu.T @ np.linalg.inv(S) @ nu))
    vals = np.array(vals)
    return float(np.mean(vals)), vals

def occupancy_metrics(prob_map, gt_binary, thr=0.5, eps=1e-9):
    """prob_map in [0,1], gt_binary in {0,1}"""
    p = np.clip(np.asarray(prob_map, dtype=float), eps, 1 - eps)
    q = np.asarray(gt_binary, dtype=float)
    ce = -np.mean(q * np.log(p) + (1 - q) * np.log(1 - p))
    brier = np.mean((p - q) ** 2)

    pred = (p >= thr).astype(int)
    tp = int(np.sum((pred == 1) & (q == 1)))
    fp = int(np.sum((pred == 1) & (q == 0)))
    fn = int(np.sum((pred == 0) & (q == 1)))
    tn = int(np.sum((pred == 0) & (q == 0)))

    precision = tp / (tp + fp + 1e-12)
    recall = tp / (tp + fn + 1e-12)
    f1 = 2 * precision * recall / (precision + recall + 1e-12)
    iou = tp / (tp + fp + fn + 1e-12)

    return {
        "cross_entropy": float(ce),
        "brier": float(brier),
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "iou": float(iou),
    }

def demo():
    np.random.seed(19)
    N = 200
    t = np.linspace(0, 20, N)
    gt_xy = np.column_stack([0.5 * t, np.sin(0.4 * t) * 2.0])
    gt_yaw = np.arctan2(np.gradient(gt_xy[:,1]), np.gradient(gt_xy[:,0]))

    # Estimated trajectory: global transform + local drift + noise
    theta0 = np.deg2rad(7.0)
    R0 = np.array([[np.cos(theta0), -np.sin(theta0)],
                   [np.sin(theta0),  np.cos(theta0)]])
    tr0 = np.array([1.5, -0.8])
    drift = np.column_stack([0.01 * t, -0.004 * t])
    est_xy = (R0.T @ (gt_xy - tr0).T).T + drift + 0.03 * np.random.randn(N, 2)
    est_yaw = wrap_to_pi(gt_yaw - theta0 + 0.01 * np.sin(0.2 * t) + 0.01 * np.random.randn(N))

    # Align estimated to ground truth
    R, tt, theta = align_se2(gt_xy, est_xy)
    est_xy_a, est_yaw_a = apply_se2(est_xy, est_yaw, R, tt, theta)

    ate = ate_rmse(gt_xy, est_xy_a)
    rpe_stats = rpe(gt_xy, gt_yaw, est_xy_a, est_yaw_a, delta=5)

    # Simulated filter consistency (pose error x,y,psi)
    n = 3
    errs = np.column_stack([
        0.05 * np.random.randn(N),
        0.05 * np.random.randn(N),
        np.deg2rad(1.0) * np.random.randn(N)
    ])
    Pk = np.zeros((N, n, n))
    for k in range(N):
        Pk[k] = np.diag([0.05**2, 0.05**2, np.deg2rad(1.1)**2])
    anees_mean, anees_vals = anees(errs, Pk)

    # Simulated innovation consistency (range-bearing sensor)
    m = 2
    nu = np.column_stack([0.08 * np.random.randn(N), np.deg2rad(2.0) * np.random.randn(N)])
    Sk = np.zeros((N, m, m))
    for k in range(N):
        Sk[k] = np.diag([0.1**2, np.deg2rad(2.5)**2])
    nis_mean, nis_vals = nis(nu, Sk)

    # Occupancy grid metrics
    H, W = 80, 100
    yy, xx = np.mgrid[0:H, 0:W]
    gt_occ = (((xx - 40)**2 + (yy - 35)**2) < 14**2).astype(int)
    gt_occ |= (((xx > 70) & (yy > 10) & (yy < 55))).astype(int)

    logits = -2.2 + 4.8 * gt_occ + 0.8 * np.random.randn(H, W)
    p_occ = 1.0 / (1.0 + np.exp(-logits))
    occ_stats = occupancy_metrics(p_occ, gt_occ, thr=0.5)

    print("=== Chapter19 Lesson1 Metrics Demo ===")
    print(f"ATE RMSE [m]: {ate:.4f}")
    print(f"RPE translational RMSE [m] (delta=5): {rpe_stats['rpe_trans_rmse']:.4f}")
    print(f"RPE rotational RMSE [rad] (delta=5): {rpe_stats['rpe_rot_rmse_rad']:.6f}")
    print(f"ANEES mean (n=3 expected near 3): {anees_mean:.4f}")
    print(f"NIS mean (m=2 expected near 2): {nis_mean:.4f}")
    print("Occupancy metrics:", occ_stats)

if __name__ == "__main__":
    demo()
