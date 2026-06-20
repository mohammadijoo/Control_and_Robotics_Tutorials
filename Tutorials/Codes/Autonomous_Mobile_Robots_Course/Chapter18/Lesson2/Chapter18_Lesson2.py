#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Chapter18_Lesson2.py
Traversability + terrain classification demo for outdoor AMR.

Pipeline (simplified):
1) Points -> 2.5D height map (mean height per grid cell)
2) Geometric features per cell: slope, roughness, step-height
3) Calibrate a logistic traversability model from labeled samples (optional)
4) Produce traversability probability map and export to CSV

This script is intentionally minimal and educational (no ROS required).
For ROS2/Nav2 integration you would publish the map as a grid/costmap layer.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np


@dataclass
class GridSpec:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution: float

    @property
    def width(self) -> int:
        return int(math.ceil((self.x_max - self.x_min) / self.resolution))

    @property
    def height(self) -> int:
        return int(math.ceil((self.y_max - self.y_min) / self.resolution))


def read_xyz_csv(path: str) -> np.ndarray:
    """CSV columns: x,y,z (no header required). Returns (N,3)."""
    pts = []
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if not row:
                continue
            try:
                x, y, z = float(row[0]), float(row[1]), float(row[2])
            except ValueError:
                # allow header line
                continue
            pts.append((x, y, z))
    if not pts:
        raise ValueError(f"No valid points found in {path}")
    return np.asarray(pts, dtype=float)


def synthetic_point_cloud(n: int = 200_000, seed: int = 0) -> np.ndarray:
    """
    Generate a toy outdoor scene:
    - sloped plane + bumps + a 'ditch' region
    """
    rng = np.random.default_rng(seed)
    x = rng.uniform(-10.0, 10.0, size=n)
    y = rng.uniform(-10.0, 10.0, size=n)

    # base plane
    z = 0.05 * x + 0.02 * y

    # bumps / rocks (Gaussian bumps)
    for (cx, cy, a, s) in [(-3.0, 2.0, 0.35, 1.2), (4.0, -1.5, 0.25, 0.9), (1.0, 5.0, 0.30, 1.0)]:
        z += a * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2.0 * s ** 2))

    # ditch (negative obstacle)
    ditch = (x > -2.0) & (x < 2.0) & (y > -6.0) & (y < -3.0)
    z[ditch] -= 0.6

    # sensor noise
    z += rng.normal(0.0, 0.02, size=n)
    return np.stack([x, y, z], axis=1)


def points_to_height_map(points: np.ndarray, spec: GridSpec) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns:
      H: (Ny, Nx) mean height with NaNs for empty cells
      C: (Ny, Nx) counts per cell
    """
    Nx, Ny = spec.width, spec.height
    H_sum = np.zeros((Ny, Nx), dtype=float)
    C = np.zeros((Ny, Nx), dtype=int)

    ix = np.floor((points[:, 0] - spec.x_min) / spec.resolution).astype(int)
    iy = np.floor((points[:, 1] - spec.y_min) / spec.resolution).astype(int)

    valid = (ix >= 0) & (ix < Nx) & (iy >= 0) & (iy < Ny)
    ix, iy = ix[valid], iy[valid]
    z = points[valid, 2]

    # accumulate
    np.add.at(H_sum, (iy, ix), z)
    np.add.at(C, (iy, ix), 1)

    H = np.full((Ny, Nx), np.nan, dtype=float)
    mask = C > 0
    H[mask] = H_sum[mask] / C[mask]
    return H, C


def nan_fill_nearest(H: np.ndarray, max_iter: int = 6) -> np.ndarray:
    """
    Cheap hole filling: iteratively average neighbors for NaNs.
    """
    out = H.copy()
    Ny, Nx = out.shape
    for _ in range(max_iter):
        nan = np.isnan(out)
        if not nan.any():
            break
        new = out.copy()
        for y in range(Ny):
            for x in range(Nx):
                if not nan[y, x]:
                    continue
                ys = slice(max(0, y - 1), min(Ny, y + 2))
                xs = slice(max(0, x - 1), min(Nx, x + 2))
                neigh = out[ys, xs]
                vals = neigh[~np.isnan(neigh)]
                if vals.size:
                    new[y, x] = float(vals.mean())
        out = new
    return out


def local_window_stats(A: np.ndarray, k: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute local mean and std over (2k+1)x(2k+1) window.
    Assumes A has no NaNs (fill before calling).
    """
    Ny, Nx = A.shape
    mean = np.zeros_like(A)
    std = np.zeros_like(A)

    for y in range(Ny):
        ys = slice(max(0, y - k), min(Ny, y + k + 1))
        for x in range(Nx):
            xs = slice(max(0, x - k), min(Nx, x + k + 1))
            patch = A[ys, xs].ravel()
            mean[y, x] = patch.mean()
            std[y, x] = patch.std(ddof=0)
    return mean, std


def compute_features(H: np.ndarray, spec: GridSpec) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns:
      slope_rad: atan(||grad h||)
      rough: local std of height
      step: local (max-min)
      valid: boolean where original H had data (before fill)
    """
    valid = ~np.isnan(H)
    Hf = nan_fill_nearest(H)

    # finite differences for gradient
    dhdy, dhdx = np.gradient(Hf, spec.resolution, spec.resolution)  # dh/dy, dh/dx
    grad_norm = np.sqrt(dhdx ** 2 + dhdy ** 2)
    slope_rad = np.arctan(grad_norm)

    # roughness and step height over 3x3
    _, rough = local_window_stats(Hf, k=1)

    Ny, Nx = Hf.shape
    step = np.zeros_like(Hf)
    for y in range(Ny):
        ys = slice(max(0, y - 1), min(Ny, y + 2))
        for x in range(Nx):
            xs = slice(max(0, x - 1), min(Nx, x + 2))
            patch = Hf[ys, xs]
            step[y, x] = float(np.max(patch) - np.min(patch))

    return slope_rad, rough, step, valid


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def fit_logistic_gd(X: np.ndarray, y: np.ndarray, lam: float = 1e-2, iters: int = 2000, lr: float = 0.2) -> np.ndarray:
    """
    Logistic regression with L2 regularization via gradient descent.
    Returns weights w (including bias as w[0]).
    """
    n, d = X.shape
    Xb = np.concatenate([np.ones((n, 1)), X], axis=1)
    w = np.zeros(d + 1)

    for _ in range(iters):
        p = sigmoid(Xb @ w)
        grad = (Xb.T @ (p - y)) / n
        grad[1:] += lam * w[1:]
        w -= lr * grad
    return w


def make_synthetic_labels(slope_rad: np.ndarray, rough: np.ndarray, step: np.ndarray) -> np.ndarray:
    """
    Toy oracle label: traversable iff slope<22deg, rough<0.05m, step<0.12m.
    """
    slope_deg = slope_rad * 180.0 / np.pi
    y = (slope_deg < 22.0) & (rough < 0.05) & (step < 0.12)
    return y.astype(float)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--points_csv", type=str, default="", help="Optional input CSV with x,y,z points.")
    ap.add_argument("--resolution", type=float, default=0.25, help="Grid resolution [m].")
    ap.add_argument("--out_csv", type=str, default="traversability_map.csv", help="Output CSV path.")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.points_csv:
        pts = read_xyz_csv(args.points_csv)
        x_min, x_max = float(np.min(pts[:, 0])), float(np.max(pts[:, 0]))
        y_min, y_max = float(np.min(pts[:, 1])), float(np.max(pts[:, 1]))
    else:
        pts = synthetic_point_cloud(seed=args.seed)
        x_min, x_max = -10.0, 10.0
        y_min, y_max = -10.0, 10.0

    spec = GridSpec(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max, resolution=args.resolution)

    H, C = points_to_height_map(pts, spec)
    slope, rough, step, valid = compute_features(H, spec)

    # training samples from valid cells only
    mask = valid
    X = np.stack([slope[mask], rough[mask], step[mask]], axis=1)
    y = make_synthetic_labels(slope[mask], rough[mask], step[mask])

    # Try sklearn if available; otherwise use our GD
    w = None
    try:
        from sklearn.linear_model import LogisticRegression

        clf = LogisticRegression(C=50.0, fit_intercept=True, solver="lbfgs", max_iter=500)
        clf.fit(X, y)
        w = np.concatenate([[clf.intercept_[0]], clf.coef_[0]])
    except Exception:
        w = fit_logistic_gd(X, y, lam=1e-2, iters=2500, lr=0.3)

    # apply
    Ny, Nx = H.shape
    X_all = np.stack([slope.ravel(), rough.ravel(), step.ravel()], axis=1)
    Xb = np.concatenate([np.ones((X_all.shape[0], 1)), X_all], axis=1)
    p = sigmoid(Xb @ w).reshape((Ny, Nx))
    p[~valid] = np.nan

    # export
    with open(args.out_csv, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["x", "y", "p_trav", "slope_deg", "rough_m", "step_m", "count"])
        for iy in range(Ny):
            y0 = spec.y_min + (iy + 0.5) * spec.resolution
            for ix in range(Nx):
                x0 = spec.x_min + (ix + 0.5) * spec.resolution
                if np.isnan(p[iy, ix]):
                    continue
                wr.writerow([
                    f"{x0:.3f}",
                    f"{y0:.3f}",
                    f"{p[iy, ix]:.6f}",
                    f"{(slope[iy, ix]*180.0/np.pi):.3f}",
                    f"{rough[iy, ix]:.4f}",
                    f"{step[iy, ix]:.4f}",
                    int(C[iy, ix]),
                ])

    print("W (bias, slope_rad, rough, step) =", np.array2string(w, precision=4))
    print("Exported:", os.path.abspath(args.out_csv))


if __name__ == "__main__":
    main()
