#!/usr/bin/env python3
"""Chapter 3 - Lesson 2 (Exercise 1): Find the maximum constant speed that satisfies
both steering curvature limits and lateral-acceleration limits on a given path.

Task:
  Given a sampled path (x_i, y_i), estimate curvature and compute
  v_max = min_i sqrt(a_lat_max / |kappa_i|) subject to |kappa_i| <= kappa_max_steer.
"""

from __future__ import annotations
import numpy as np
from Chapter3_Lesson2 import (
    discrete_curvature_polyline,
    BicycleLimits,
    LateralAccelLimits,
    bicycle_kappa_max,
    v_max_from_lateral_accel,
)


def main() -> None:
    # Example polyline path (students may replace with their own)
    s = np.linspace(0.0, 12.0, 401)
    x = s
    y = 1.2 * np.sin(0.7 * s) + 0.2 * np.sin(2.2 * s)
    P = np.vstack([x, y]).T

    kappa = discrete_curvature_polyline(P)

    lim = BicycleLimits(L=0.35, delta_max=np.deg2rad(28.0), delta_dot_max=np.deg2rad(60.0))
    lat = LateralAccelLimits(a_lat_max=2.0)

    kappa_max = bicycle_kappa_max(lim)
    if np.any(np.abs(kappa) > kappa_max + 1e-12):
        print("Path violates steering curvature limit at some samples.")
        print("Consider smoothing path or increasing turning radius.")
    else:
        print("Path satisfies steering curvature bound.")

    vmax_lat = v_max_from_lateral_accel(kappa, lat)
    finite = vmax_lat[np.isfinite(vmax_lat)]
    vmax = float(np.min(finite)) if finite.size > 0 else float("inf")
    print(f"Maximum constant speed from lateral acceleration limit: v_max = {vmax:.3f} m/s")


if __name__ == "__main__":
    main()
