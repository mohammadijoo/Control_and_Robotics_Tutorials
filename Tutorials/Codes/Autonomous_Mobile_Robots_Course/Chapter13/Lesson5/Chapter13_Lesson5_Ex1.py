#!/usr/bin/env python3
"""
Chapter13_Lesson5_Ex1.py
Estimate an approximate camera-IMU time offset from two time series by cross-correlation.

This is a *diagnostic* utility for the lab:
- Visual stack provides a scalar proxy for angular motion (e.g., delta-yaw rate from VO).
- IMU provides gyro magnitude.
If the sensors are shifted by dt, the cross-correlation peaks near that dt.

Input CSV format (two files):
t,value  (header optional)

Example:
  python Chapter13_Lesson5_Ex1.py --imu imu_gyro_mag.csv --vis vo_yawrate.csv --max-shift 0.2
"""
from __future__ import annotations

import argparse
import numpy as np


def _read_csv_tv(path: str) -> tuple[np.ndarray, np.ndarray]:
    t, v = [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.lower().startswith("t"):
                continue
            parts = s.split(",")
            if len(parts) < 2:
                continue
            t.append(float(parts[0]))
            v.append(float(parts[1]))
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    return t, v


def _resample_uniform(t: np.ndarray, v: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    t0, t1 = t.min(), t.max()
    tu = np.arange(t0, t1, dt)
    vu = np.interp(tu, t, v)
    return tu, vu


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--imu", required=True)
    ap.add_argument("--vis", required=True)
    ap.add_argument("--dt", type=float, default=0.002, help="Resample dt (seconds)")
    ap.add_argument("--max-shift", type=float, default=0.2, help="Search +/- max shift (seconds)")
    args = ap.parse_args()

    t_i, v_i = _read_csv_tv(args.imu)
    t_v, v_v = _read_csv_tv(args.vis)

    ti, vi = _resample_uniform(t_i, v_i, args.dt)
    tv, vv = _resample_uniform(t_v, v_v, args.dt)

    # common overlap window
    t0 = max(ti.min(), tv.min())
    t1 = min(ti.max(), tv.max())
    mask_i = (ti >= t0) & (ti <= t1)
    mask_v = (tv >= t0) & (tv <= t1)
    ti, vi = ti[mask_i], vi[mask_i]
    tv, vv = tv[mask_v], vv[mask_v]

    # make same length by truncating to min
    n = min(len(vi), len(vv))
    vi, vv = vi[:n], vv[:n]

    # normalize
    vi = (vi - vi.mean()) / (vi.std() + 1e-12)
    vv = (vv - vv.mean()) / (vv.std() + 1e-12)

    max_lag = int(round(args.max_shift / args.dt))
    lags = np.arange(-max_lag, max_lag + 1)
    corr = np.zeros_like(lags, dtype=float)

    for k, lag in enumerate(lags):
        if lag >= 0:
            a = vi[lag:]
            b = vv[:len(a)]
        else:
            a = vi[:lag]
            b = vv[-lag:]
        if len(a) < 10:
            corr[k] = np.nan
        else:
            corr[k] = float(np.dot(a, b) / len(a))

    best = int(np.nanargmax(corr))
    best_lag = lags[best]
    dt_est = -best_lag * args.dt  # sign: shift applied to VIS to align to IMU

    print("Estimated time offset (VIS relative to IMU) [s]:", dt_est)
    print("Peak correlation:", corr[best])


if __name__ == "__main__":
    main()
