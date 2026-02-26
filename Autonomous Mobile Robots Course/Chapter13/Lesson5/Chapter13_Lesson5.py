#!/usr/bin/env python3
"""
Chapter13_Lesson5.py
Trajectory evaluation utilities for VIO/Visual SLAM labs.

Input trajectory format (TUM):
t tx ty tz qx qy qz qw

Typical usage:
  python Chapter13_Lesson5.py --gt groundtruth.txt --est estimated.txt --allow-scale --plot

Notes:
- "allow-scale" is useful for monocular VO/SLAM where scale is unknown.
- Association matches by nearest timestamp within --max-dt.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np


@dataclass
class Trajectory:
    t: np.ndarray          # (N,)
    p: np.ndarray          # (N,3)
    q: np.ndarray          # (N,4) [x,y,z,w] unit quaternions


def _read_tum(path: str) -> Trajectory:
    ts, ps, qs = [], [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if (not s) or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            tx, ty, tz = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])
            ts.append(t)
            ps.append([tx, ty, tz])
            qs.append([qx, qy, qz, qw])
    t = np.asarray(ts, dtype=float)
    p = np.asarray(ps, dtype=float)
    q = np.asarray(qs, dtype=float)
    # normalize quaternions defensively
    n = np.linalg.norm(q, axis=1, keepdims=True) + 1e-12
    q = q / n
    return Trajectory(t=t, p=p, q=q)


def _associate_by_time(t_a: np.ndarray, t_b: np.ndarray, max_dt: float) -> List[Tuple[int, int]]:
    """
    Greedy nearest-neighbor association between time arrays t_a and t_b.
    Returns list of index pairs (i_a, i_b), sorted by time in a.
    """
    pairs: List[Tuple[int, int]] = []
    j = 0
    used_b = set()
    for i, ta in enumerate(t_a):
        # advance j while t_b[j] < ta
        while j + 1 < len(t_b) and t_b[j] < ta:
            j += 1
        # check candidates j and j-1
        cand = []
        for jj in (j, j - 1):
            if 0 <= jj < len(t_b) and jj not in used_b:
                cand.append(jj)
        if not cand:
            continue
        # pick nearest
        jj_best = min(cand, key=lambda jj: abs(t_b[jj] - ta))
        if abs(t_b[jj_best] - ta) <= max_dt:
            pairs.append((i, jj_best))
            used_b.add(jj_best)
    return pairs


def _umeyama_alignment(A: np.ndarray, B: np.ndarray, with_scale: bool) -> Tuple[float, np.ndarray, np.ndarray]:
    """
    Umeyama similarity alignment: find s,R,t that minimizes sum || s R A_i + t - B_i ||^2.
    A, B: (N,3) point sets.
    Returns (s, R, t).
    """
    if A.shape != B.shape or A.shape[1] != 3:
        raise ValueError("A and B must be (N,3) and same shape.")

    n = A.shape[0]
    if n < 3:
        raise ValueError("Need at least 3 point correspondences.")

    mu_A = A.mean(axis=0)
    mu_B = B.mean(axis=0)
    X = A - mu_A
    Y = B - mu_B

    # covariance
    Sigma = (Y.T @ X) / n  # 3x3

    U, D, Vt = np.linalg.svd(Sigma)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0

    R = U @ S @ Vt
    if with_scale:
        var_A = (X * X).sum() / n
        s = float(np.trace(np.diag(D) @ S) / (var_A + 1e-12))
    else:
        s = 1.0

    t = mu_B - s * (R @ mu_A)
    return s, R, t


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    # q = [x,y,z,w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    # normalized assumed
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=float)


def _so3_log(R: np.ndarray) -> np.ndarray:
    """
    Log map for SO(3). Returns angle-axis vector (3,).
    """
    tr = np.trace(R)
    cos_theta = (tr - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3)
    w = (1.0 / (2.0 * math.sin(theta))) * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ], dtype=float)
    return theta * w


def _ate(gt: Trajectory, est: Trajectory, max_dt: float, allow_scale: bool) -> dict:
    pairs = _associate_by_time(est.t, gt.t, max_dt=max_dt)
    if len(pairs) < 3:
        raise RuntimeError(f"Too few associated poses: {len(pairs)}")

    A = np.array([est.p[i] for i, _ in pairs], dtype=float)
    B = np.array([gt.p[j] for _, j in pairs], dtype=float)

    s, R, t = _umeyama_alignment(A, B, with_scale=allow_scale)
    A_aligned = (s * (R @ A.T)).T + t.reshape(1, 3)

    e = A_aligned - B
    se = np.sum(e * e, axis=1)
    rmse = float(np.sqrt(np.mean(se)))
    mean = float(np.sqrt(np.mean(se)))  # same as rmse for Euclidean norms under mean of squared
    med = float(np.median(np.sqrt(se)))
    mx = float(np.max(np.sqrt(se)))

    return {
        "n_pairs": int(len(pairs)),
        "scale": float(s),
        "R": R,
        "t": t,
        "rmse": rmse,
        "median": med,
        "max": mx,
        "aligned_est_positions": A_aligned,
        "gt_positions": B,
        "assoc_pairs": pairs,
    }


def _rpe(gt: Trajectory, est: Trajectory, pairs: List[Tuple[int, int]], R_align: np.ndarray, t_align: np.ndarray, s_align: float) -> dict:
    """
    Relative Pose Error for consecutive associated pairs.
    Translation RPE uses aligned positions. Rotation RPE uses quaternion composition
    but is only approximate unless orientations are in same frame; we align rotations using R_align.
    """
    if len(pairs) < 2:
        raise RuntimeError("Need at least 2 associated poses for RPE.")

    # positions
    A = np.array([est.p[i] for i, _ in pairs], dtype=float)
    B = np.array([gt.p[j] for _, j in pairs], dtype=float)
    A_aligned = (s_align * (R_align @ A.T)).T + t_align.reshape(1, 3)

    dt = []
    dr = []

    for k in range(len(pairs) - 1):
        # translation delta error
        dA = A_aligned[k + 1] - A_aligned[k]
        dB = B[k + 1] - B[k]
        dt.append(np.linalg.norm(dA - dB))

        # rotation delta error (approx)
        qi, qj = est.q[pairs[k][0]], est.q[pairs[k + 1][0]]
        qgi, qgj = gt.q[pairs[k][1]], gt.q[pairs[k + 1][1]]

        # delta rotations
        dq_est = _quat_mul(_quat_conj(qi), qj)
        dq_gt = _quat_mul(_quat_conj(qgi), qgj)

        # align est delta by R_align (SO3): R(dq_est) -> R_align * R_est * R_align^T
        R_est = _quat_to_rot(dq_est)
        R_gt = _quat_to_rot(dq_gt)
        R_est_al = R_align @ R_est @ R_align.T
        dR = R_est_al.T @ R_gt
        w = _so3_log(dR)
        dr.append(np.linalg.norm(w))  # radians

    dt = np.asarray(dt, dtype=float)
    dr = np.asarray(dr, dtype=float)
    return {
        "trans_rmse": float(np.sqrt(np.mean(dt*dt))),
        "rot_rmse_rad": float(np.sqrt(np.mean(dr*dr))),
        "trans_mean": float(np.mean(dt)),
        "rot_mean_rad": float(np.mean(dr)),
        "n_edges": int(len(dt)),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True, help="Ground truth TUM trajectory file")
    ap.add_argument("--est", required=True, help="Estimated TUM trajectory file")
    ap.add_argument("--max-dt", type=float, default=0.02, help="Max timestamp difference for association (seconds)")
    ap.add_argument("--allow-scale", action="store_true", help="Estimate similarity scale (useful for monocular)")
    ap.add_argument("--plot", action="store_true", help="Plot aligned trajectory (requires matplotlib)")
    args = ap.parse_args()

    gt = _read_tum(args.gt)
    est = _read_tum(args.est)

    ate = _ate(gt, est, max_dt=args.max_dt, allow_scale=args.allow_scale)
    rpe = _rpe(gt, est, ate["assoc_pairs"], ate["R"], ate["t"], ate["scale"])

    print("=== ATE (aligned) ===")
    print("pairs:", ate["n_pairs"])
    print("scale:", ate["scale"])
    print("rmse [m]:", ate["rmse"])
    print("median [m]:", ate["median"])
    print("max [m]:", ate["max"])
    print("=== RPE (consecutive) ===")
    print("edges:", rpe["n_edges"])
    print("trans_rmse [m]:", rpe["trans_rmse"])
    print("rot_rmse [rad]:", rpe["rot_rmse_rad"])

    if args.plot:
        import matplotlib.pyplot as plt
        A = ate["aligned_est_positions"]
        B = ate["gt_positions"]
        plt.figure()
        plt.plot(B[:, 0], B[:, 1], label="gt")
        plt.plot(A[:, 0], A[:, 1], label="est_aligned")
        plt.xlabel("x [m]"); plt.ylabel("y [m]")
        plt.axis("equal")
        plt.legend()
        plt.title("Trajectory (top-down)")
        plt.show()


if __name__ == "__main__":
    main()
