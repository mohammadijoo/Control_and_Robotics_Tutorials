#!/usr/bin/env python3
"""Chapter 3 - Lesson 2: Curvature and Turning Radius Limits

This script provides:
1) Continuous and discrete curvature computation for planar paths.
2) Turning-radius/curvature limits for:
   - Unicycle (v, omega) model
   - Differential-drive (wheel speeds)
   - Car-like (bicycle/Ackermann) model
3) Speed-dependent feasibility checks from lateral-acceleration bounds and steering-rate bounds.

Dependencies:
  - numpy
  - matplotlib (optional, for plotting)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass(frozen=True)
class BicycleLimits:
    L: float                 # wheelbase [m]
    delta_max: float         # max steering angle magnitude [rad]
    delta_dot_max: float     # max steering rate magnitude [rad/s]


@dataclass(frozen=True)
class LateralAccelLimits:
    a_lat_max: float         # max lateral acceleration magnitude [m/s^2]


def curvature_parametric(x: np.ndarray, y: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Compute signed curvature kappa(t) for a parametric curve (x(t), y(t)).
    Uses kappa = (x' y'' - y' x'') / ( (x'^2 + y'^2)^(3/2) ).

    Notes:
      - Requires t to be strictly increasing and sufficiently smooth sampling.
      - Uses central finite differences via numpy.gradient.
    """
    x1 = np.gradient(x, t)
    y1 = np.gradient(y, t)
    x2 = np.gradient(x1, t)
    y2 = np.gradient(y1, t)
    denom = (x1**2 + y1**2) ** 1.5
    eps = 1e-12
    kappa = (x1 * y2 - y1 * x2) / (denom + eps)
    return kappa


def curvature_three_points(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    """Signed curvature from three points in R^2 using the circumcircle formula.

    For triangle (p0, p1, p2):
      kappa = 4 * A / (a*b*c)
    where A is signed area, a=|p1-p2|, b=|p0-p2|, c=|p0-p1|.

    Returns:
      Signed curvature (1/m). If points are nearly collinear, returns 0.
    """
    p0 = np.asarray(p0, dtype=float).reshape(2)
    p1 = np.asarray(p1, dtype=float).reshape(2)
    p2 = np.asarray(p2, dtype=float).reshape(2)

    v01 = p1 - p0
    v02 = p2 - p0
    v12 = p2 - p1

    a = np.linalg.norm(v12)
    b = np.linalg.norm(v02)
    c = np.linalg.norm(v01)

    # Signed double area via 2D cross product (scalar)
    area2 = v01[0] * v02[1] - v01[1] * v02[0]  # = 2*A_signed

    denom = a * b * c
    if denom < 1e-12:
        return 0.0
    return 2.0 * area2 / denom  # since kappa = 4*A/(abc) and area2 = 2*A


def discrete_curvature_polyline(P: np.ndarray) -> np.ndarray:
    """Compute curvature along a polyline P (N x 2) using 3-point formula at interior points.
    Endpoints are assigned curvature 0.
    """
    P = np.asarray(P, dtype=float)
    N = P.shape[0]
    kappa = np.zeros(N)
    for i in range(1, N - 1):
        kappa[i] = curvature_three_points(P[i - 1], P[i], P[i + 1])
    return kappa


def bicycle_kappa_from_delta(delta: np.ndarray, L: float) -> np.ndarray:
    """kappa = tan(delta)/L"""
    return np.tan(delta) / L


def bicycle_delta_from_kappa(kappa: np.ndarray, L: float) -> np.ndarray:
    """delta = arctan(L*kappa)"""
    return np.arctan(L * kappa)


def bicycle_kappa_max(lim: BicycleLimits) -> float:
    """Max achievable curvature magnitude from steering angle bound."""
    return np.tan(lim.delta_max) / lim.L


def v_max_from_lateral_accel(kappa: np.ndarray, lat: LateralAccelLimits) -> np.ndarray:
    """From |a_lat| = v^2 * |kappa| <= a_lat_max => v <= sqrt(a_lat_max/|kappa|).
    For kappa ~ 0, returns +inf.
    """
    kabs = np.abs(kappa)
    vmax = np.full_like(kabs, np.inf, dtype=float)
    mask = kabs > 1e-12
    vmax[mask] = np.sqrt(lat.a_lat_max / kabs[mask])
    return vmax


def kappa_dot_max_from_delta_dot(lim: BicycleLimits, delta: float) -> float:
    """Given delta_dot bound, bound kappa_dot:
       kappa = tan(delta)/L => kappa_dot = (sec^2(delta)/L) * delta_dot
       so |kappa_dot| <= (sec^2(delta)/L) * delta_dot_max.
    """
    sec2 = 1.0 / (np.cos(delta) ** 2)
    return (sec2 / lim.L) * lim.delta_dot_max


def check_bicycle_feasibility(
    kappa: np.ndarray,
    v: np.ndarray,
    lim: BicycleLimits,
    lat: Optional[LateralAccelLimits] = None,
) -> Tuple[np.ndarray, dict]:
    """Check basic feasibility along a path for a bicycle model.

    Constraints checked:
      1) Steering angle: |delta| <= delta_max
      2) Lateral acceleration (optional): v^2 * |kappa| <= a_lat_max
      3) Steering-rate induced curvature-rate bound (coarse):
         approximate kappa_dot via finite difference in time and check |kappa_dot| <= kappa_dot_max(delta)

    Returns:
      feasible_mask: boolean array (N,) for each sample.
      diagnostics: dict of arrays/values for analysis.
    """
    kappa = np.asarray(kappa, dtype=float)
    v = np.asarray(v, dtype=float)
    if kappa.shape != v.shape:
        raise ValueError("kappa and v must have the same shape")

    delta = bicycle_delta_from_kappa(kappa, lim.L)
    steer_ok = np.abs(delta) <= lim.delta_max + 1e-12

    lat_ok = np.ones_like(steer_ok, dtype=bool)
    a_lat = None
    if lat is not None:
        a_lat = (v ** 2) * np.abs(kappa)
        lat_ok = a_lat <= lat.a_lat_max + 1e-12

    # curvature-rate check in time (assume uniform dt from normalized time)
    N = len(kappa)
    dt = 1.0 / max(N - 1, 1)
    kappa_dot = np.gradient(kappa, dt)

    # Per-sample bound
    kappa_dot_max = np.array([kappa_dot_max_from_delta_dot(lim, float(d)) for d in delta])
    rate_ok = np.abs(kappa_dot) <= kappa_dot_max + 1e-12

    feasible = steer_ok & lat_ok & rate_ok
    diagnostics = {
        "delta": delta,
        "steer_ok": steer_ok,
        "a_lat": a_lat,
        "lat_ok": lat_ok,
        "kappa_dot": kappa_dot,
        "kappa_dot_max": kappa_dot_max,
        "rate_ok": rate_ok,
        "kappa_max_steer": bicycle_kappa_max(lim),
    }
    return feasible, diagnostics


def diff_drive_wheel_speeds(v: float, kappa: float, r_w: float, b: float) -> Tuple[float, float]:
    """Map (v, kappa) to differential-drive wheel angular speeds (wr, wl).

    Kinematics:
      v = (r/2) (wr + wl)
      omega = (r/b) (wr - wl)
      kappa = omega / v

    Solve:
      wr = (v/r) (1 + (b/2) kappa)
      wl = (v/r) (1 - (b/2) kappa)
    """
    wr = (v / r_w) * (1.0 + 0.5 * b * kappa)
    wl = (v / r_w) * (1.0 - 0.5 * b * kappa)
    return wr, wl


def example_demo() -> None:
    """Demonstrate curvature computation and feasibility checks on a sample path."""
    # Sample path: a smooth "S" curve in x-y
    t = np.linspace(0.0, 10.0, 400)
    x = t
    y = 2.0 * np.sin(0.6 * t)

    kappa = curvature_parametric(x, y, t)

    # Speed profile (constant here for simplicity)
    v = np.full_like(kappa, 1.5)  # m/s

    lim = BicycleLimits(L=0.35, delta_max=np.deg2rad(30.0), delta_dot_max=np.deg2rad(60.0))
    lat = LateralAccelLimits(a_lat_max=2.5)

    feasible, diag = check_bicycle_feasibility(kappa, v, lim, lat)

    print("Steering-limited |kappa|max =", diag["kappa_max_steer"])
    print("Feasible fraction =", feasible.mean())

    # Compute speed limit from lateral acceleration alone
    vmax_lat = v_max_from_lateral_accel(kappa, lat)
    finite = vmax_lat[np.isfinite(vmax_lat)]
    if finite.size > 0:
        print("Median lateral-accel vmax =", float(np.median(finite)))

    # Differential-drive mapping example at one point
    i = len(kappa) // 2
    r_w, b = 0.08, 0.32
    wr, wl = diff_drive_wheel_speeds(v=float(v[i]), kappa=float(kappa[i]), r_w=r_w, b=b)
    print(f"Diff-drive wheel speeds at mid path: wr={wr:.3f} rad/s, wl={wl:.3f} rad/s")

    # Optional plotting
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(x, y)
        plt.axis("equal")
        plt.title("Path (x,y)")
        plt.figure()
        plt.plot(t, kappa)
        plt.title("Curvature kappa(t)")
        plt.figure()
        plt.plot(t, feasible.astype(float))
        plt.title("Feasibility (1=ok, 0=violates)")
        plt.show()
    except Exception as e:
        print("Plotting skipped:", e)


if __name__ == "__main__":
    example_demo()
