#!/usr/bin/env python3
# Chapter2_Lesson1.py
"""
Autonomous Mobile Robots (Control Engineering)
Chapter 2, Lesson 1: Rolling Constraints and Instantaneous Motion

This script:
1) Defines a planar rigid-body twist in the BODY frame: xi = [v_x, v_y, omega]
2) Encodes standard-wheel rolling constraints:
   - no lateral slip: n_i^T (v + omega * k x r_i) = 0
   - rolling rate:    phi_dot_i = (1/R_i) t_i^T (v + omega * k x r_i)
3) Computes the ICR in the BODY frame: p_ICR = [ -v_y/omega, v_x/omega ]  (omega != 0)
4) Demonstrates consistency of ICR with axle-line intersection.

Dependencies: numpy (required), sympy (optional, for symbolic check).
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np


@dataclass(frozen=True)
class Wheel:
    # Wheel i expressed in BODY frame:
    # r_i = [l_x, l_y]^T is the wheel contact (or wheel center) position in body coordinates.
    lx: float
    ly: float
    # alpha is wheel rolling direction angle (rad) measured from body x-axis to wheel plane direction.
    alpha: float
    # radius
    R: float


def t_dir(alpha: float) -> np.ndarray:
    """Unit rolling direction t = [cos alpha, sin alpha]."""
    return np.array([math.cos(alpha), math.sin(alpha)], dtype=float)


def n_dir(alpha: float) -> np.ndarray:
    """Unit lateral direction n = [-sin alpha, cos alpha] (perpendicular to rolling direction)."""
    return np.array([-math.sin(alpha), math.cos(alpha)], dtype=float)


def v_point_body(vx: float, vy: float, omega: float, r: np.ndarray) -> np.ndarray:
    """
    Planar rigid-body velocity at point r (BODY frame):
      v(r) = v + omega * k x r
    with k x [x, y] = [-y, x].
    """
    return np.array([vx, vy], dtype=float) + omega * np.array([-r[1], r[0]], dtype=float)


def lateral_constraint_row(w: Wheel) -> np.ndarray:
    """
    Returns a row a_i such that a_i @ [vx, vy, omega] = 0 for no lateral slip:
        n_i^T (v + omega k x r_i) = 0.
    """
    r = np.array([w.lx, w.ly], dtype=float)
    n = n_dir(w.alpha)
    # n^T [vx, vy] + omega * n^T [-ry, rx] = 0
    a = np.array([n[0], n[1], (-r[1] * n[0] + r[0] * n[1])], dtype=float)
    return a


def build_A(wheels: List[Wheel]) -> np.ndarray:
    """Stacks lateral no-slip constraints into A such that A @ [vx, vy, omega] = 0."""
    return np.vstack([lateral_constraint_row(w) for w in wheels])


def wheel_spin_rates(wheels: List[Wheel], vx: float, vy: float, omega: float) -> np.ndarray:
    """Computes phi_dot for each wheel from rolling constraint: phi_dot = (1/R) t^T v_point."""
    rates = []
    for w in wheels:
        r = np.array([w.lx, w.ly], dtype=float)
        t = t_dir(w.alpha)
        vpt = v_point_body(vx, vy, omega, r)
        rates.append((t @ vpt) / w.R)
    return np.array(rates, dtype=float)


def icr_body(vx: float, vy: float, omega: float, eps: float = 1e-12) -> Optional[np.ndarray]:
    """
    Instantaneous Center of Rotation (BODY frame).
    For omega != 0: p = [ -v_y/omega, v_x/omega ].
    If omega ~ 0, return None (ICR at infinity; pure translation).
    """
    if abs(omega) < eps:
        return None
    return np.array([-vy / omega, vx / omega], dtype=float)


def axle_line_residual(w: Wheel, p: np.ndarray) -> float:
    """
    For standard wheel with no lateral slip, ICR should lie on the axle line:
      p = r_i + s * n_i
    Equivalent 2D cross product (p - r_i) x n_i = 0.
    Returns scalar residual (should be ~0 if consistent).
    """
    r = np.array([w.lx, w.ly], dtype=float)
    n = n_dir(w.alpha)
    d = p - r
    return float(d[0] * n[1] - d[1] * n[0])


def demo_two_wheel_parallel_track() -> None:
    """
    Demonstration with two fixed, parallel wheels (like the simplest differential base geometry),
    but *without* using differential-drive kinematic equations yet.
    We only check constraints/ICR consistency.

    Wheels are located at y = +/- b/2, rolling along body x-axis (alpha=0).
    """
    b = 0.6
    wheels = [
        Wheel(lx=0.0, ly=+b/2, alpha=0.0, R=0.1),
        Wheel(lx=0.0, ly=-b/2, alpha=0.0, R=0.1),
    ]

    # Choose a twist that respects no-lateral-slip:
    # For alpha=0, n=[0,1], constraint is v_y + omega * x_i = 0 (here x_i=0), so v_y=0.
    vx, vy, omega = 0.5, 0.0, 0.8

    A = build_A(wheels)
    xi = np.array([vx, vy, omega])
    print("A =\n", A)
    print("A @ xi =", A @ xi)

    p = icr_body(vx, vy, omega)
    print("ICR (body) =", p)

    for i, w in enumerate(wheels):
        res = axle_line_residual(w, p)
        print(f"Axle-line residual wheel {i+1}: {res:+.3e}")

    rates = wheel_spin_rates(wheels, vx, vy, omega)
    print("Wheel spin rates [rad/s] =", rates)


def solve_feasible_twist_least_norm(wheels: List[Wheel], v_des: np.ndarray) -> np.ndarray:
    """
    Given desired twist v_des = [vx, vy, omega], compute the closest feasible twist (least squares)
    satisfying A xi = 0:
        min ||xi - v_des||_2  s.t. A xi = 0.

    Solution via projection: xi = (I - A^T (A A^T)^(-1) A) v_des  if A full row rank.
    """
    A = build_A(wheels)
    I = np.eye(3)
    if A.size == 0:
        return v_des.copy()

    AA = A @ A.T
    # Regularize for numerical stability
    lam = 1e-12
    AA_reg = AA + lam * np.eye(AA.shape[0])
    P = I - A.T @ np.linalg.solve(AA_reg, A)
    return P @ v_des


def main() -> None:
    print("=== Demo: two parallel wheels (constraint + ICR check) ===")
    demo_two_wheel_parallel_track()

    print("\n=== Demo: 3 wheels with inconsistent geometry (detect slip) ===")
    wheels = [
        Wheel(lx=0.2, ly=0.2, alpha=0.0, R=0.08),
        Wheel(lx=-0.2, ly=0.2, alpha=math.pi/2, R=0.08),
        Wheel(lx=0.0, ly=-0.25, alpha=0.0, R=0.08),
    ]
    v_des = np.array([0.5, 0.2, 0.6])
    xi = solve_feasible_twist_least_norm(wheels, v_des)
    print("Desired twist:", v_des)
    print("Projected feasible twist:", xi)
    A = build_A(wheels)
    print("Constraint residual A@xi:", A @ xi)

    p = icr_body(float(xi[0]), float(xi[1]), float(xi[2]))
    print("ICR (body) =", p)
    if p is not None:
        for i, w in enumerate(wheels):
            res = axle_line_residual(w, p)
            print(f"Axle-line residual wheel {i+1}: {res:+.3e}")


if __name__ == "__main__":
    main()
