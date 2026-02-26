#!/usr/bin/env python3
"""
Chapter5_Lesson5_Ex1.py

Exercise 1:
Generate a synthetic differential-drive dataset with known (rL, rR, b),
add encoder noise, then run the calibration routine from Chapter5_Lesson5.py.

This is a self-contained script that produces a CSV log compatible with the lab pipeline.
"""

from __future__ import annotations

import argparse
import numpy as np
import pandas as pd


def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def simulate(
    T: float,
    dt: float,
    ticks_per_rev: float,
    rL_true: float,
    rR_true: float,
    b_true: float,
    v_cmd: float,
    w_cmd: float,
    sigma_ticks: float,
    trial_id: int,
):
    """
    Simulate robot motion with commanded (v,w) in continuous time, and produce encoder ticks.
    Ground truth integrates the true kinematics. "Odom" uses a biased parameter set (deliberately wrong).
    """
    N = int(T / dt) + 1
    t = np.linspace(0, T, N)

    # True wheel angular velocities
    # v = (rR*wR + rL*wL)/2, w = (rR*wR - rL*wL)/b
    # Solve: wR = (v + 0.5*b*w)/rR, wL = (v - 0.5*b*w)/rL
    wR = (v_cmd + 0.5 * b_true * w_cmd) / rR_true
    wL = (v_cmd - 0.5 * b_true * w_cmd) / rL_true

    dphiR = wR * dt * np.ones(N)
    dphiL = wL * dt * np.ones(N)
    dphiR[0] = 0.0
    dphiL[0] = 0.0

    # Convert to ticks and add noise, then accumulate
    ticks_per_rad = ticks_per_rev / (2 * np.pi)
    dnR = dphiR * ticks_per_rad + np.random.normal(0.0, sigma_ticks, size=N)
    dnL = dphiL * ticks_per_rad + np.random.normal(0.0, sigma_ticks, size=N)
    dnR[0] = 0.0
    dnL[0] = 0.0

    nR = np.cumsum(dnR)
    nL = np.cumsum(dnL)

    # Integrate ground truth from true wheel arcs
    sR = rR_true * dphiR
    sL = rL_true * dphiL
    ds = 0.5 * (sR + sL)
    dth = (sR - sL) / b_true
    th = np.zeros(N)
    th_mid = wrap_angle(th + 0.5 * dth)
    dx = ds * np.cos(th_mid)
    dy = ds * np.sin(th_mid)

    x = np.cumsum(dx)
    y = np.cumsum(dy)
    th = wrap_angle(np.cumsum(dth))

    # Build a deliberately biased odometry (wrong parameters)
    rL_odom = rL_true * 1.02
    rR_odom = rR_true * 0.98
    b_odom = b_true * 1.05

    sR_o = rR_odom * dphiR
    sL_o = rL_odom * dphiL
    ds_o = 0.5 * (sR_o + sL_o)
    dth_o = (sR_o - sL_o) / b_odom
    th_o = np.zeros(N)
    th_mid_o = wrap_angle(th_o + 0.5 * dth_o)
    dx_o = ds_o * np.cos(th_mid_o)
    dy_o = ds_o * np.sin(th_mid_o)
    x_odom = np.cumsum(dx_o)
    y_odom = np.cumsum(dy_o)
    th_odom = wrap_angle(np.cumsum(dth_o))

    df = pd.DataFrame(
        {
            "t": t,
            "nL": nL,
            "nR": nR,
            "x_odom": x_odom,
            "y_odom": y_odom,
            "th_odom": th_odom,
            "x_gt": x,
            "y_gt": y,
            "th_gt": th,
            "trial_id": trial_id,
        }
    )
    return df


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out_csv", default="synthetic_ch5_l5.csv")
    ap.add_argument("--ticks_per_rev", type=float, default=4096.0)
    ap.add_argument("--rL_true", type=float, default=0.05)
    ap.add_argument("--rR_true", type=float, default=0.05)
    ap.add_argument("--b_true", type=float, default=0.30)
    ap.add_argument("--sigma_ticks", type=float, default=0.5)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--T", type=float, default=10.0)
    ap.add_argument("--v", type=float, default=0.2)
    ap.add_argument("--w", type=float, default=0.4)
    ap.add_argument("--trials", type=int, default=5)
    args = ap.parse_args()

    np.random.seed(0)
    dfs = []
    for j in range(args.trials):
        dfj = simulate(
            T=args.T,
            dt=args.dt,
            ticks_per_rev=args.ticks_per_rev,
            rL_true=args.rL_true,
            rR_true=args.rR_true,
            b_true=args.b_true,
            v_cmd=args.v,
            w_cmd=args.w,
            sigma_ticks=args.sigma_ticks,
            trial_id=j,
        )
        dfs.append(dfj)

    df = pd.concat(dfs, ignore_index=True)
    df.to_csv(args.out_csv, index=False)
    print("Wrote:", args.out_csv)
    print("Now run:")
    print(
        f"  python3 Chapter5_Lesson5.py --csv {args.out_csv} --ticks_per_rev {args.ticks_per_rev} "
        f"--r_nom {args.rL_true} --b_nom {args.b_true}"
    )


if __name__ == "__main__":
    main()
