#!/usr/bin/env python3
# Chapter2_Lesson5.py
"""
Chapter 2 — Wheeled Locomotion Kinematics (Mobile-Specific)
Lesson 5 — Kinematic Calibration of Wheel Parameters

This script performs nonlinear least-squares calibration of differential-drive wheel parameters:
  p = [r_L, r_R, b]
from paired data:
  (dphi_L[k], dphi_R[k]) and ground-truth relative increments (dx_gt[k], dy_gt[k], dtheta_gt[k])
where (dx, dy, dtheta) are expressed in the robot body frame at the beginning of each segment.

Dependencies (recommended):
  - numpy
  - scipy  (scipy.optimize.least_squares)

Data format (CSV, header allowed):
  dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt
Angles in radians; dx,dy in meters; dtheta in radians.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np

try:
    from scipy.optimize import least_squares
except Exception as e:
    least_squares = None


@dataclass
class CalibData:
    dphi_L: np.ndarray
    dphi_R: np.ndarray
    dx_gt: np.ndarray
    dy_gt: np.ndarray
    dtheta_gt: np.ndarray


def wrap_to_pi(a: np.ndarray) -> np.ndarray:
    """Wrap angles to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _sinc1(x: np.ndarray) -> np.ndarray:
    """sin(x)/x with a 4th-order Taylor safeguard near 0."""
    y = np.empty_like(x)
    small = np.abs(x) < 1e-6
    xs = x[small]
    # 1 - x^2/6 + x^4/120
    y[small] = 1.0 - (xs * xs) / 6.0 + (xs ** 4) / 120.0
    xb = x[~small]
    y[~small] = np.sin(xb) / xb
    return y


def _cosc1(x: np.ndarray) -> np.ndarray:
    """(1-cos(x))/x with a 5th-order Taylor safeguard near 0."""
    y = np.empty_like(x)
    small = np.abs(x) < 1e-6
    xs = x[small]
    # x/2 - x^3/24 + x^5/720
    y[small] = xs / 2.0 - (xs ** 3) / 24.0 + (xs ** 5) / 720.0
    xb = x[~small]
    y[~small] = (1.0 - np.cos(xb)) / xb
    return y


def _dsinc1(x: np.ndarray) -> np.ndarray:
    """Derivative of sinc1: d/dx [sin(x)/x] = (x cos x - sin x) / x^2 with safeguard."""
    y = np.empty_like(x)
    small = np.abs(x) < 1e-5
    xs = x[small]
    # derivative series: -(x)/3 + (x^3)/30 - (x^5)/840
    y[small] = -(xs) / 3.0 + (xs ** 3) / 30.0 - (xs ** 5) / 840.0
    xb = x[~small]
    y[~small] = (xb * np.cos(xb) - np.sin(xb)) / (xb * xb)
    return y


def _dcosc1(x: np.ndarray) -> np.ndarray:
    """Derivative of cosc1: d/dx[(1-cos x)/x] = (x sin x - (1-cos x)) / x^2 with safeguard."""
    y = np.empty_like(x)
    small = np.abs(x) < 1e-5
    xs = x[small]
    # series: 1/2 - x^2/8 + x^4/144
    y[small] = 0.5 - (xs * xs) / 8.0 + (xs ** 4) / 144.0
    xb = x[~small]
    y[~small] = (xb * np.sin(xb) - (1.0 - np.cos(xb))) / (xb * xb)
    return y


def predict_body_increment(dphi_L: np.ndarray, dphi_R: np.ndarray, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Predict (dx, dy, dtheta) in body frame for each segment given encoder angle increments.
    p = [r_L, r_R, b].
    """
    rL, rR, b = float(p[0]), float(p[1]), float(p[2])
    sL = rL * dphi_L
    sR = rR * dphi_R
    ds = 0.5 * (sR + sL)
    dtheta = (sR - sL) / b

    f = _sinc1(dtheta)             # sin(dtheta)/dtheta
    g = _cosc1(dtheta)             # (1-cos(dtheta))/dtheta
    dx = ds * f
    dy = ds * g
    return dx, dy, dtheta


def residuals_and_jacobian(p: np.ndarray, data: CalibData) -> Tuple[np.ndarray, np.ndarray]:
    """
    Stacked residual vector and Jacobian for Gauss-Newton / Levenberg-Marquardt.

    Residual ordering per k:
      e_x[k] = dx(p) - dx_gt
      e_y[k] = dy(p) - dy_gt
      e_th[k] = wrap(dtheta(p) - dtheta_gt)
    """
    rL, rR, b = float(p[0]), float(p[1]), float(p[2])

    dphiL = data.dphi_L
    dphiR = data.dphi_R

    sL = rL * dphiL
    sR = rR * dphiR
    A = sR + sL
    B = sR - sL

    ds = 0.5 * A
    dtheta = B / b

    f = _sinc1(dtheta)
    g = _cosc1(dtheta)
    fp = _dsinc1(dtheta)
    gp = _dcosc1(dtheta)

    dx = ds * f
    dy = ds * g

    ex = dx - data.dx_gt
    ey = dy - data.dy_gt
    eth = wrap_to_pi(dtheta - data.dtheta_gt)

    # derivatives of ds
    dds_drL = 0.5 * dphiL
    dds_drR = 0.5 * dphiR
    dds_db = 0.0

    # derivatives of dtheta
    ddth_drL = -(dphiL) / b
    ddth_drR = (dphiR) / b
    ddth_db = -(B) / (b * b)  # = -dtheta/b

    # dx = ds*f(dtheta)
    ddx_drL = dds_drL * f + ds * fp * ddth_drL
    ddx_drR = dds_drR * f + ds * fp * ddth_drR
    ddx_db  = dds_db  * f + ds * fp * ddth_db

    # dy = ds*g(dtheta)
    ddy_drL = dds_drL * g + ds * gp * ddth_drL
    ddy_drR = dds_drR * g + ds * gp * ddth_drR
    ddy_db  = dds_db  * g + ds * gp * ddth_db

    # dtheta residual uses wrapped difference; Jacobian uses local derivative (valid when |residual| < pi)
    dth_drL = ddth_drL
    dth_drR = ddth_drR
    dth_db  = ddth_db

    N = dphiL.size
    r = np.empty(3 * N, dtype=float)
    J = np.empty((3 * N, 3), dtype=float)

    r[0::3] = ex
    r[1::3] = ey
    r[2::3] = eth

    J[0::3, 0] = ddx_drL
    J[0::3, 1] = ddx_drR
    J[0::3, 2] = ddx_db

    J[1::3, 0] = ddy_drL
    J[1::3, 1] = ddy_drR
    J[1::3, 2] = ddy_db

    J[2::3, 0] = dth_drL
    J[2::3, 1] = dth_drR
    J[2::3, 2] = dth_db

    return r, J


def load_csv(path: str) -> CalibData:
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if row[0].strip().lower().startswith("dphi"):
                continue
            rows.append([float(x) for x in row[:5]])
    arr = np.asarray(rows, dtype=float)
    return CalibData(
        dphi_L=arr[:, 0],
        dphi_R=arr[:, 1],
        dx_gt=arr[:, 2],
        dy_gt=arr[:, 3],
        dtheta_gt=arr[:, 4],
    )


def calibrate(data: CalibData, p0: np.ndarray, use_scipy: bool = True) -> Tuple[np.ndarray, dict]:
    """
    Calibrate p = [r_L, r_R, b]. Returns (p_hat, info).
    """
    if use_scipy and least_squares is not None:
        def fun(p):
            r, _ = residuals_and_jacobian(p, data)
            return r

        def jac(p):
            _, J = residuals_and_jacobian(p, data)
            return J

        res = least_squares(fun, x0=p0, jac=jac, method="trf", loss="huber", f_scale=0.01)
        info = {"success": bool(res.success), "message": res.message, "cost": float(res.cost), "nfev": int(res.nfev)}
        return res.x.astype(float), info

    # Toolbox-free fallback: damped Gauss-Newton
    p = p0.astype(float).copy()
    lam = 1e-3
    for it in range(50):
        r, J = residuals_and_jacobian(p, data)
        A = J.T @ J + lam * np.eye(3)
        g = J.T @ r
        dp = -np.linalg.solve(A, g)
        if np.linalg.norm(dp) < 1e-10:
            break
        p_new = p + dp
        r_new, _ = residuals_and_jacobian(p_new, data)
        if (r_new @ r_new) < (r @ r):
            p = p_new
            lam *= 0.7
        else:
            lam *= 2.0
    info = {"success": True, "message": "Gauss-Newton fallback", "cost": float(0.5 * (r @ r)), "nfev": 0}
    return p, info


def simulate_dataset(N: int = 200, p_true: np.ndarray = np.array([0.050, 0.051, 0.300]), noise: float = 0.002, seed: int = 7) -> CalibData:
    """
    Create synthetic calibration data with small noise on ground-truth increments.
    """
    rng = np.random.default_rng(seed)
    # diverse motions: mix of straight-ish, arcs, and in-place rotations
    dphiL = rng.normal(loc=0.0, scale=0.4, size=N)
    dphiR = rng.normal(loc=0.0, scale=0.4, size=N)

    mix = rng.uniform(0, 1, size=N)
    # encourage some straight segments
    dphiR[mix < 0.35] = dphiL[mix < 0.35] + rng.normal(0.0, 0.03, size=np.sum(mix < 0.35))
    # encourage some in-place rotations
    mask = (mix >= 0.35) & (mix < 0.55)
    dphiR[mask] = -dphiL[mask] + rng.normal(0.0, 0.03, size=np.sum(mask))

    dx, dy, dth = predict_body_increment(dphiL, dphiR, p_true)

    dxn = dx + rng.normal(0.0, noise, size=N)
    dyn = dy + rng.normal(0.0, noise, size=N)
    dthn = dth + rng.normal(0.0, noise * 0.5, size=N)

    return CalibData(dphi_L=dphiL, dphi_R=dphiR, dx_gt=dxn, dy_gt=dyn, dtheta_gt=dthn)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=str, default="", help="Path to CSV (dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt).")
    parser.add_argument("--rL0", type=float, default=0.05)
    parser.add_argument("--rR0", type=float, default=0.05)
    parser.add_argument("--b0", type=float, default=0.30)
    args = parser.parse_args()

    if args.csv:
        data = load_csv(args.csv)
    else:
        data = simulate_dataset()

    p0 = np.array([args.rL0, args.rR0, args.b0], dtype=float)
    phat, info = calibrate(data, p0=p0, use_scipy=True)

    print("Initial p0  = [rL, rR, b] =", p0)
    print("Estimated p = [rL, rR, b] =", phat)
    print("Info:", info)

    # sanity: RMS residual
    r, _ = residuals_and_jacobian(phat, data)
    r3 = r.reshape(-1, 3)
    rms = np.sqrt(np.mean(r3 * r3, axis=0))
    print("RMS residuals [dx, dy, dtheta] =", rms)


if __name__ == "__main__":
    main()
