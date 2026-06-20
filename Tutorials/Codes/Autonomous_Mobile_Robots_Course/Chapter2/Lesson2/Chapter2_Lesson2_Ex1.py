"""Chapter 2 - Lesson 2 (Exercise 1): Estimate turning radius and curvature from wheel rates.

Task:
  Given wheel rates (phi_dot_l, phi_dot_r), compute:
    - v, w
    - ICC radius R = v/w (if w != 0)
    - curvature kappa = w/v (if v != 0)

Then sweep a set of wheel-rate pairs and print a small table.
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class DiffDriveParams:
    r: float
    L: float


def twist(phi_dot_l: float, phi_dot_r: float, p: DiffDriveParams) -> tuple[float, float]:
    v_l = p.r * phi_dot_l
    v_r = p.r * phi_dot_r
    v = 0.5 * (v_r + v_l)
    w = (v_r - v_l) / p.L
    return float(v), float(w)


def icc_radius(v: float, w: float) -> float:
    if abs(w) < 1e-12:
        return float("inf")
    return v / w


def curvature(v: float, w: float) -> float:
    if abs(v) < 1e-12:
        return float("inf")
    return w / v


if __name__ == "__main__":
    p = DiffDriveParams(r=0.05, L=0.30)
    pairs = [(5, 5), (5, 8), (8, 5), (-5, 5), (0, 6)]

    print("phi_dot_l  phi_dot_r   v[m/s]    w[rad/s]   R[m]      kappa[1/m]")
    for pl, pr in pairs:
        v, w = twist(pl, pr, p)
        R = icc_radius(v, w)
        k = curvature(v, w)
        print(f"{pl:8.2f}  {pr:8.2f}  {v:8.4f}  {w:10.4f}  {R:8.4f}  {k:12.4f}")
