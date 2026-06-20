#!/usr/bin/env python3
"""
Chapter5_Lesson5.py

Lab: Characterizing Odometry Error (Differential Drive)

This script:
1) Loads a CSV log containing time, encoder counts, odometry pose, and ground truth pose.
2) Computes endpoint error, trajectory RMSE, and drift-per-meter.
3) Estimates differential-drive parameters (r_L, r_R, b) via Gauss-Newton least squares.
4) Computes increment residual covariance (empirical Q) after calibration.
5) Produces diagnostic plots.

Expected CSV columns (minimum):
  t
  nL, nR                          (cumulative encoder ticks)
  x_odom, y_odom, th_odom          (odometry pose)
  x_gt, y_gt, th_gt                (ground truth pose)

Optional:
  trial_id                         (integer trial label for repeated experiments)
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
import pandas as pd


# -----------------------------
# Utilities
# -----------------------------

def wrap_angle(theta: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) to (-pi, pi]."""
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def interp_angle(t_src: np.ndarray, th_src: np.ndarray, t_new: np.ndarray) -> np.ndarray:
    """Interpolate angles by unwrapping then wrapping."""
    th_unwrap = np.unwrap(th_src)
    th_new = np.interp(t_new, t_src, th_unwrap)
    return wrap_angle(th_new)


@dataclass
class Params:
    rL: float
    rR: float
    b: float


# -----------------------------
# Odometry model (increment form)
# -----------------------------

def ticks_to_dphi(n: np.ndarray, ticks_per_rev: float) -> np.ndarray:
    """Convert cumulative ticks to wheel angle increments (rad) with same length as n (first increment is 0)."""
    dn = np.diff(n, prepend=n[0])
    return (2.0 * np.pi / ticks_per_rev) * dn


def predict_increment_from_enc(
    th: np.ndarray,
    dphiL: np.ndarray,
    dphiR: np.ndarray,
    p: Params
) -> np.ndarray:
    """
    Predict pose increments [dx, dy, dth] for each step k using midpoint integration,
    given heading th[k-1] (so pass th aligned to increments) and wheel increments.
    Returns array shape (N,3) for k=1..N (note: first row corresponds to first increment).
    """
    sL = p.rL * dphiL
    sR = p.rR * dphiR
    ds = 0.5 * (sR + sL)
    dth = (sR - sL) / p.b

    # midpoint heading
    th_mid = wrap_angle(th + 0.5 * dth)

    dx = ds * np.cos(th_mid)
    dy = ds * np.sin(th_mid)
    return np.vstack([dx, dy, dth]).T


def cumulative_from_increments(x0: np.ndarray, inc: np.ndarray) -> np.ndarray:
    """Integrate increments to cumulative pose. x0 = [x,y,th]. inc: (N,3). returns (N+1,3)."""
    out = np.zeros((inc.shape[0] + 1, 3), dtype=float)
    out[0, :] = x0
    out[1:, :] = np.cumsum(inc, axis=0) + x0
    out[:, 2] = wrap_angle(out[:, 2])
    return out


# -----------------------------
# Metrics
# -----------------------------

def compute_errors(df: pd.DataFrame) -> Dict[str, float]:
    """Compute RMSE and endpoint metrics for one trajectory (single trial)."""
    ex = df["x_odom"].to_numpy() - df["x_gt"].to_numpy()
    ey = df["y_odom"].to_numpy() - df["y_gt"].to_numpy()
    eth = wrap_angle(df["th_odom"].to_numpy() - df["th_gt"].to_numpy())

    epos = np.sqrt(ex**2 + ey**2)
    rmse_pos = float(np.sqrt(np.mean(epos**2)))
    rmse_th = float(np.sqrt(np.mean(eth**2)))

    end_pos = float(np.sqrt(ex[-1] ** 2 + ey[-1] ** 2))
    end_th = float(abs(eth[-1]))

    # ground-truth traveled distance
    xg = df["x_gt"].to_numpy()
    yg = df["y_gt"].to_numpy()
    ds = np.sqrt(np.diff(xg) ** 2 + np.diff(yg) ** 2)
    s_total = float(np.sum(ds)) if len(ds) else 0.0
    drift_per_m = end_pos / s_total if s_total > 1e-12 else float("nan")

    return {
        "rmse_pos": rmse_pos,
        "rmse_th": rmse_th,
        "end_pos": end_pos,
        "end_th": end_th,
        "s_total": s_total,
        "drift_per_m": drift_per_m,
    }


def compute_increment_residuals(
    df: pd.DataFrame, ticks_per_rev: float, p: Params
) -> np.ndarray:
    """
    Compute increment residuals eps_k = inc_pred(p) - inc_gt, k=1..N.
    inc_gt is formed from successive ground-truth pose differences.
    """
    nL = df["nL"].to_numpy()
    nR = df["nR"].to_numpy()
    dphiL = ticks_to_dphi(nL, ticks_per_rev)
    dphiR = ticks_to_dphi(nR, ticks_per_rev)

    # Use heading at previous step. Align by shifting: th_prev[k] = th_gt[k-1]
    th_gt = df["th_gt"].to_numpy()
    th_prev = np.roll(th_gt, 1)
    th_prev[0] = th_gt[0]

    inc_pred = predict_increment_from_enc(th_prev, dphiL, dphiR, p)

    xg = df["x_gt"].to_numpy()
    yg = df["y_gt"].to_numpy()
    thg = df["th_gt"].to_numpy()

    dx = np.diff(xg, prepend=xg[0])
    dy = np.diff(yg, prepend=yg[0])
    dth = wrap_angle(np.diff(thg, prepend=thg[0]))

    inc_gt = np.vstack([dx, dy, dth]).T
    eps = inc_pred - inc_gt
    # Remove first "increment" which is zero by construction
    return eps[1:, :]


# -----------------------------
# Parameter estimation via Gauss-Newton
# -----------------------------

def stack_residuals_and_jacobian(
    df: pd.DataFrame,
    ticks_per_rev: float,
    p: Params,
    w_theta: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build residual vector r(p) and Jacobian J for Gauss-Newton using increment residuals.
    Residual per step: [dx_pred - dx_gt, dy_pred - dy_gt, w_theta*(dth_pred - dth_gt)].

    Returns:
      r: (3N,) residual
      J: (3N,3) Jacobian wrt [rL, rR, b]
    """
    nL = df["nL"].to_numpy()
    nR = df["nR"].to_numpy()
    dphiL = ticks_to_dphi(nL, ticks_per_rev)
    dphiR = ticks_to_dphi(nR, ticks_per_rev)

    th_gt = df["th_gt"].to_numpy()
    th_prev = np.roll(th_gt, 1)
    th_prev[0] = th_gt[0]

    # Ground-truth increments
    xg = df["x_gt"].to_numpy()
    yg = df["y_gt"].to_numpy()
    thg = df["th_gt"].to_numpy()
    dx_gt = np.diff(xg, prepend=xg[0])
    dy_gt = np.diff(yg, prepend=yg[0])
    dth_gt = wrap_angle(np.diff(thg, prepend=thg[0]))

    # Predicted increments
    sL = p.rL * dphiL
    sR = p.rR * dphiR
    ds = 0.5 * (sR + sL)
    dth = (sR - sL) / p.b
    th_mid = wrap_angle(th_prev + 0.5 * dth)

    dx = ds * np.cos(th_mid)
    dy = ds * np.sin(th_mid)

    # Residuals (skip k=0)
    rx = dx - dx_gt
    ry = dy - dy_gt
    rth = wrap_angle(dth - dth_gt)

    rx = rx[1:]
    ry = ry[1:]
    rth = rth[1:]
    N = rx.shape[0]

    r = np.zeros((3 * N,), dtype=float)
    r[0::3] = rx
    r[1::3] = ry
    r[2::3] = w_theta * rth

    # Jacobian
    # d(ds)/d(rL)=0.5*dphiL, d(ds)/d(rR)=0.5*dphiR
    dds_drL = 0.5 * dphiL[1:]
    dds_drR = 0.5 * dphiR[1:]
    # d(dth)/d(rL)=-(dphiL)/b, d(dth)/d(rR)=(dphiR)/b, d(dth)/d(b)=-(sR-sL)/b^2
    ddth_drL = -(dphiL[1:]) / p.b
    ddth_drR = (dphiR[1:]) / p.b
    ddth_db = -(sR[1:] - sL[1:]) / (p.b ** 2)

    # dx = ds*cos(th_prev + 0.5*dth)
    # Let a = th_prev + 0.5*dth. Then:
    # d(dx) = cos(a)*d(ds) + ds*(-sin(a))*d(a)
    # d(a) = 0.5*d(dth)
    a = th_mid[1:]
    ca = np.cos(a)
    sa = np.sin(a)
    ds_k = ds[1:]

    # dx derivatives
    ddx_drL = ca * dds_drL + ds_k * (-sa) * (0.5 * ddth_drL)
    ddx_drR = ca * dds_drR + ds_k * (-sa) * (0.5 * ddth_drR)
    ddx_db  = ds_k * (-sa) * (0.5 * ddth_db)

    # dy = ds*sin(a)
    # d(dy) = sin(a)*d(ds) + ds*cos(a)*d(a)
    ddy_drL = sa * dds_drL + ds_k * ca * (0.5 * ddth_drL)
    ddy_drR = sa * dds_drR + ds_k * ca * (0.5 * ddth_drR)
    ddy_db  = ds_k * ca * (0.5 * ddth_db)

    # dth derivatives (scaled)
    ddth_drL_s = w_theta * ddth_drL
    ddth_drR_s = w_theta * ddth_drR
    ddth_db_s  = w_theta * ddth_db

    J = np.zeros((3 * N, 3), dtype=float)
    # rows for step i: 3i,3i+1,3i+2
    J[0::3, 0] = ddx_drL
    J[0::3, 1] = ddx_drR
    J[0::3, 2] = ddx_db

    J[1::3, 0] = ddy_drL
    J[1::3, 1] = ddy_drR
    J[1::3, 2] = ddy_db

    J[2::3, 0] = ddth_drL_s
    J[2::3, 1] = ddth_drR_s
    J[2::3, 2] = ddth_db_s

    return r, J


def gauss_newton_calibrate(
    df: pd.DataFrame,
    ticks_per_rev: float,
    p0: Params,
    w_theta: float = 1.0,
    max_iter: int = 15,
    damping: float = 1e-9
) -> Params:
    """
    Estimate (rL, rR, b) by Gauss-Newton on increment residuals.
    Damping adds lambda*I to J^T J for numerical stability.
    """
    p = Params(p0.rL, p0.rR, p0.b)
    for _it in range(max_iter):
        r, J = stack_residuals_and_jacobian(df, ticks_per_rev, p, w_theta=w_theta)
        A = J.T @ J + damping * np.eye(3)
        g = J.T @ r
        try:
            dp = -np.linalg.solve(A, g)
        except np.linalg.LinAlgError:
            break

        # simple line search (optional): enforce positive parameters
        alpha = 1.0
        for _ in range(10):
            cand = Params(p.rL + alpha * dp[0], p.rR + alpha * dp[1], p.b + alpha * dp[2])
            if cand.rL > 0 and cand.rR > 0 and cand.b > 0:
                break
            alpha *= 0.5

        p_new = Params(p.rL + alpha * dp[0], p.rR + alpha * dp[1], p.b + alpha * dp[2])

        # stop if step is tiny
        if np.linalg.norm(dp) < 1e-12:
            p = p_new
            break
        p = p_new

    return p


# -----------------------------
# I/O and analysis pipeline
# -----------------------------

def load_log_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = ["t", "nL", "nR", "x_odom", "y_odom", "th_odom", "x_gt", "y_gt", "th_gt"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}")

    # Ensure numeric
    for c in required:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=required).reset_index(drop=True)
    df["th_odom"] = wrap_angle(df["th_odom"].to_numpy())
    df["th_gt"] = wrap_angle(df["th_gt"].to_numpy())

    if "trial_id" not in df.columns:
        df["trial_id"] = 0
    return df


def analyze_trials(
    df: pd.DataFrame,
    ticks_per_rev: float,
    p_nominal: Params,
    w_theta: float
) -> Dict[str, object]:
    """
    Compute metrics per trial, calibrate on all data, then compute metrics again using
    re-integrated odometry from encoders with calibrated params.
    """
    results: Dict[str, object] = {}

    # Metrics "as logged" (whatever odom source provides)
    trial_metrics = []
    for tid, g in df.groupby("trial_id"):
        m = compute_errors(g)
        m["trial_id"] = int(tid)
        trial_metrics.append(m)
    metrics_df = pd.DataFrame(trial_metrics).sort_values("trial_id")
    results["metrics_logged"] = metrics_df

    # Calibrate on entire dataset (concatenate)
    p_hat = gauss_newton_calibrate(df, ticks_per_rev, p_nominal, w_theta=w_theta)
    results["p_hat"] = p_hat

    # Reconstruct odometry from encoders using p_hat and compare to gt (per trial)
    trial_metrics_cal = []
    for tid, g in df.groupby("trial_id"):
        nL = g["nL"].to_numpy()
        nR = g["nR"].to_numpy()
        dphiL = ticks_to_dphi(nL, ticks_per_rev)
        dphiR = ticks_to_dphi(nR, ticks_per_rev)

        th_gt = g["th_gt"].to_numpy()
        th_prev = np.roll(th_gt, 1)
        th_prev[0] = th_gt[0]

        inc = predict_increment_from_enc(th_prev, dphiL, dphiR, p_hat)
        x0 = np.array([g["x_gt"].iloc[0], g["y_gt"].iloc[0], g["th_gt"].iloc[0]], dtype=float)
        x_hat = cumulative_from_increments(x0, inc)

        g2 = g.copy()
        g2["x_odom"] = x_hat[:, 0]
        g2["y_odom"] = x_hat[:, 1]
        g2["th_odom"] = x_hat[:, 2]

        m2 = compute_errors(g2)
        m2["trial_id"] = int(tid)
        trial_metrics_cal.append(m2)

    metrics_df_cal = pd.DataFrame(trial_metrics_cal).sort_values("trial_id")
    results["metrics_calibrated"] = metrics_df_cal

    # Noise estimates from increment residuals after calibration
    eps_all = []
    for _tid, g in df.groupby("trial_id"):
        eps = compute_increment_residuals(g, ticks_per_rev, p_hat)
        if eps.shape[0] > 0:
            eps_all.append(eps)
    if eps_all:
        E = np.vstack(eps_all)
        eps_mean = np.mean(E, axis=0)
        Q = np.cov((E - eps_mean).T, bias=False)
    else:
        eps_mean = np.zeros((3,), dtype=float)
        Q = np.full((3, 3), np.nan)

    results["increment_residual_mean"] = eps_mean
    results["Q_hat"] = Q

    # Endpoint covariance across trials (logged and calibrated) on scalars [end_pos, end_th]
    def endpoint_cov(mdf: pd.DataFrame) -> np.ndarray:
        A = np.vstack([mdf["end_pos"].to_numpy(), mdf["end_th"].to_numpy()]).T
        if A.shape[0] < 2:
            return np.full((2, 2), np.nan)
        return np.cov(A.T, bias=False)

    results["endpoint_cov_logged_pos_th"] = endpoint_cov(metrics_df)
    results["endpoint_cov_cal_pos_th"] = endpoint_cov(metrics_df_cal)

    return results


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Path to CSV log")
    ap.add_argument("--ticks_per_rev", type=float, required=True, help="Encoder ticks per wheel revolution")
    ap.add_argument("--r_nom", type=float, default=0.05, help="Nominal wheel radius (m)")
    ap.add_argument("--b_nom", type=float, default=0.30, help="Nominal wheelbase (m)")
    ap.add_argument("--w_theta", type=float, default=1.0, help="Heading weight (scale) in least squares")
    ap.add_argument("--out_prefix", type=str, default="chapter5_lesson5", help="Prefix for outputs")
    args = ap.parse_args()

    df = load_log_csv(args.csv)

    p0 = Params(rL=args.r_nom, rR=args.r_nom, b=args.b_nom)
    res = analyze_trials(df, args.ticks_per_rev, p0, w_theta=args.w_theta)

    print("=== Calibration Result ===")
    p_hat: Params = res["p_hat"]  # type: ignore[assignment]
    print(f"rL_hat = {p_hat.rL:.8f} m")
    print(f"rR_hat = {p_hat.rR:.8f} m")
    print(f"b_hat  = {p_hat.b:.8f} m")
    print()

    print("=== Metrics (logged odom) per trial ===")
    print(res["metrics_logged"].to_string(index=False))
    print()

    print("=== Metrics (reconstructed from encoders, calibrated params) per trial ===")
    print(res["metrics_calibrated"].to_string(index=False))
    print()

    print("=== Increment residual mean (after calibration) [dx, dy, dth] ===")
    print(res["increment_residual_mean"])
    print()

    print("=== Increment residual covariance Q_hat (after calibration) ===")
    print(res["Q_hat"])
    print()

    print("=== Endpoint covariance on scalars [end_pos, end_th] (logged) ===")
    print(res["endpoint_cov_logged_pos_th"])
    print()

    print("=== Endpoint covariance on scalars [end_pos, end_th] (calibrated) ===")
    print(res["endpoint_cov_cal_pos_th"])
    print()

    # Optional plotting if matplotlib is available
    try:
        import matplotlib.pyplot as plt

        m1 = res["metrics_logged"]
        m2 = res["metrics_calibrated"]

        plt.figure()
        plt.plot(m1["trial_id"], m1["end_pos"], marker="o", label="logged")
        plt.plot(m2["trial_id"], m2["end_pos"], marker="o", label="calibrated")
        plt.xlabel("trial_id")
        plt.ylabel("endpoint position error (m)")
        plt.legend()
        plt.title("Endpoint position error per trial")
        plt.tight_layout()
        plt.savefig(f"{args.out_prefix}_endpoint_pos.png", dpi=200)

        plt.figure()
        plt.plot(m1["trial_id"], m1["rmse_pos"], marker="o", label="logged")
        plt.plot(m2["trial_id"], m2["rmse_pos"], marker="o", label="calibrated")
        plt.xlabel("trial_id")
        plt.ylabel("trajectory RMSE position (m)")
        plt.legend()
        plt.title("Trajectory position RMSE per trial")
        plt.tight_layout()
        plt.savefig(f"{args.out_prefix}_rmse_pos.png", dpi=200)

        plt.figure()
        plt.plot(m1["trial_id"], m1["end_th"], marker="o", label="logged")
        plt.plot(m2["trial_id"], m2["end_th"], marker="o", label="calibrated")
        plt.xlabel("trial_id")
        plt.ylabel("endpoint heading error (rad)")
        plt.legend()
        plt.title("Endpoint heading error per trial")
        plt.tight_layout()
        plt.savefig(f"{args.out_prefix}_endpoint_th.png", dpi=200)

        print(f"Saved plots with prefix: {args.out_prefix}_*.png")

    except Exception as e:
        print("Plotting skipped (matplotlib not available or failed):", e)


if __name__ == "__main__":
    main()
