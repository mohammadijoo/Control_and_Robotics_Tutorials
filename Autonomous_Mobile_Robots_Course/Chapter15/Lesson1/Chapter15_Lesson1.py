"""
Chapter15_Lesson1.py
Pure Pursuit and Geometric Path Tracking — minimal, from-scratch reference implementation.

Dependencies:
  numpy
  matplotlib (optional for plotting)

This file focuses on geometry + kinematic simulation (bicycle model).
For a ROS2 integration sketch, see Chapter15_Lesson1.cpp (USE_ROS2 section).
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float  # heading [rad]


def wrap_angle(a: float) -> float:
    """Map angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def world_to_body(pose: Pose2D, p_world: np.ndarray) -> np.ndarray:
    """
    Transform a world point to robot body frame:
      x forward, y left.
    """
    R = rot2(pose.theta).T
    t = np.array([pose.x, pose.y], dtype=float)
    return R @ (p_world - t)


def polyline_arclength(path: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute cumulative arclength s[k] and segment lengths seg[k] for polyline points.
    path: (N,2)
    """
    dif = np.diff(path, axis=0)
    seg = np.linalg.norm(dif, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return s, seg


def closest_point_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float]:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return a.copy(), 0.0
    t = float(np.dot(p - a, ab) / denom)
    t_clamped = max(0.0, min(1.0, t))
    q = a + t_clamped * ab
    return q, t_clamped


def closest_point_on_polyline(p: np.ndarray, path: np.ndarray) -> Tuple[np.ndarray, int, float, float]:
    """
    Return:
      q: closest point in R^2
      i: segment index (between i and i+1)
      t: segment parameter in [0,1]
      d: distance ||p-q||
    """
    best_d = float("inf")
    best_q, best_i, best_t = path[0].copy(), 0, 0.0
    for i in range(len(path) - 1):
        q, t = closest_point_on_segment(p, path[i], path[i + 1])
        d = float(np.linalg.norm(p - q))
        if d < best_d:
            best_d = d
            best_q, best_i, best_t = q, i, t
    return best_q, best_i, best_t, best_d


def point_at_arclength(path: np.ndarray, s: np.ndarray, seg: np.ndarray, s_query: float) -> np.ndarray:
    """Return point at arclength s_query along polyline (clamped)."""
    if s_query <= 0.0:
        return path[0].copy()
    if s_query >= s[-1]:
        return path[-1].copy()

    j = int(np.searchsorted(s, s_query, side="right") - 1)  # j in [0, N-2]
    ds = s_query - s[j]
    t = ds / max(seg[j], 1e-12)
    return (1.0 - t) * path[j] + t * path[j + 1]


def pure_pursuit_curvature(pose: Pose2D, path: np.ndarray, Ld: float) -> Tuple[float, np.ndarray, np.ndarray]:
    """
    Classic pure pursuit curvature:
      - Find closest point on path (for progress)
      - Choose lookahead point Ld ahead in arclength
      - Transform lookahead to body frame (x_r, y_r)
      - Command curvature kappa = 2*y_r / (x_r^2 + y_r^2)

    Returns:
      kappa: curvature command [1/m]
      p_closest: closest point (world)
      p_look: lookahead point (world)
    """
    Ld = max(float(Ld), 1e-3)

    p = np.array([pose.x, pose.y], dtype=float)
    s, seg = polyline_arclength(path)
    q, i, t, _ = closest_point_on_polyline(p, path)

    s_closest = float(s[i] + t * seg[i])
    p_look = point_at_arclength(path, s, seg, s_closest + Ld)

    look_b = world_to_body(pose, p_look)
    x_r, y_r = float(look_b[0]), float(look_b[1])

    L = math.hypot(x_r, y_r)
    if L <= 1e-9:
        return 0.0, q, p_look

    kappa = 2.0 * y_r / (L * L)
    return float(kappa), q, p_look


def bicycle_step(pose: Pose2D, v: float, delta: float, wheelbase: float, dt: float) -> Pose2D:
    """Kinematic bicycle (no slip)."""
    x = pose.x + v * math.cos(pose.theta) * dt
    y = pose.y + v * math.sin(pose.theta) * dt
    theta = wrap_angle(pose.theta + (v / wheelbase) * math.tan(delta) * dt)
    return Pose2D(x, y, theta)


def speed_dependent_lookahead(v: float, Ld_min: float, Ld_max: float, k_v: float) -> float:
    """
    Simple heuristic: Ld = clamp(Ld_min + k_v * |v|, Ld_min, Ld_max).
    """
    return float(min(Ld_max, max(Ld_min, Ld_min + k_v * abs(v))))


def demo():
    # Path: gentle sinusoid
    xs = np.linspace(0.0, 25.0, 500)
    ys = 1.8 * np.sin(0.22 * xs)
    path = np.column_stack([xs, ys])

    pose = Pose2D(x=-2.0, y=-2.0, theta=0.2)
    dt = 0.02
    wheelbase = 0.33  # [m] small AGV-like

    v = 1.5  # [m/s]
    Ld_min, Ld_max, k_v = 0.8, 3.5, 0.8
    kappa_max = 1.6  # [1/m]
    delta_max = math.radians(32.0)

    traj = []
    look_pts = []

    for _ in range(int(25.0 / dt)):
        Ld = speed_dependent_lookahead(v, Ld_min, Ld_max, k_v)
        kappa, _, pL = pure_pursuit_curvature(pose, path, Ld)
        kappa = float(np.clip(kappa, -kappa_max, kappa_max))

        delta = math.atan(wheelbase * kappa)
        delta = float(np.clip(delta, -delta_max, delta_max))

        pose = bicycle_step(pose, v, delta, wheelbase, dt)
        traj.append((pose.x, pose.y))
        look_pts.append((float(pL[0]), float(pL[1])))

    traj = np.array(traj)
    look_pts = np.array(look_pts)

    try:
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(path[:, 0], path[:, 1], label="path")
        plt.plot(traj[:, 0], traj[:, 1], label="robot")
        plt.plot(look_pts[::60, 0], look_pts[::60, 1], ".", label="lookahead (every 60)")
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Pure Pursuit (kinematic bicycle)")
        plt.legend()
        plt.show()
    except Exception as e:
        print("Plot skipped:", e)


if __name__ == "__main__":
    demo()
