# Chapter3_Lesson5.py
"""
Autonomous Mobile Robots (Control Engineering major)
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 5 — Feasibility Checks for Candidate Paths

This module implements a practical feasibility checker for candidate paths of a car-like robot.
It verifies:
  1) curvature (steering) limits
  2) steering-rate limits (via curvature derivative + speed profile)
  3) speed and acceleration limits (forward-backward time scaling)
  4) collision clearance against an occupancy grid (disk footprint approximation)

Dependencies:
  - numpy
  - scipy (optional, for distance_transform_edt)
Robotics ecosystem references (not required to run this file):
  - ROS2 nav_msgs/Path, geometry_msgs/PoseStamped, nav2_costmap_2d (C++)
  - OMPL, FCL (collision), SBPL (grid planning)

Author: course content generator
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Dict, Any, Optional
import math
import numpy as np

try:
    from scipy.ndimage import distance_transform_edt  # type: ignore
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


@dataclass
class Limits:
    wheelbase_m: float
    delta_max_rad: float          # |steering angle| <= delta_max
    delta_dot_max_rad_s: float    # |steering rate| <= delta_dot_max
    v_max_m_s: float              # |v| <= v_max (actuator/command limit)
    a_long_max_m_s2: float        # |v_dot| <= a_long_max
    a_lat_max_m_s2: float         # |v^2*kappa| <= a_lat_max
    robot_radius_m: float         # disk footprint radius
    clearance_margin_m: float     # extra safety margin


@dataclass
class FeasibilityReport:
    ok: bool
    reason: str
    stats: Dict[str, Any]


def _arc_length(xy: np.ndarray) -> np.ndarray:
    """Cumulative arc length s[i] for points xy[i] = [x,y]."""
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.zeros(xy.shape[0], dtype=float)
    s[1:] = np.cumsum(d)
    return s


def _curvature_discrete(xy: np.ndarray, s: np.ndarray) -> np.ndarray:
    """
    Discrete signed curvature estimate using three-point formula.
    For i in 1..N-2, fit circumcircle through (i-1,i,i+1).
    """
    n = xy.shape[0]
    kappa = np.zeros(n, dtype=float)
    eps = 1e-12
    for i in range(1, n - 1):
        p0 = xy[i - 1]
        p1 = xy[i]
        p2 = xy[i + 1]
        a = p1 - p0
        b = p2 - p1
        c = p2 - p0

        la = np.linalg.norm(a)
        lb = np.linalg.norm(b)
        lc = np.linalg.norm(c)

        # Twice signed area of triangle (p0,p1,p2) = cross(a, c)
        area2 = a[0] * c[1] - a[1] * c[0]
        denom = (la * lb * lc) + eps

        # curvature magnitude = 2*area / (la*lb*lc), signed by area
        kappa[i] = 2.0 * area2 / denom

    # endpoints: copy nearest interior
    if n >= 3:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def _dkappa_ds(kappa: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Central difference derivative dkappa/ds with endpoint one-sided diffs."""
    dk = np.zeros_like(kappa)
    n = kappa.size
    if n < 3:
        return dk
    ds = np.diff(s)
    eps = 1e-12
    for i in range(1, n - 1):
        denom = (s[i + 1] - s[i - 1]) + eps
        dk[i] = (kappa[i + 1] - kappa[i - 1]) / denom
    dk[0] = (kappa[1] - kappa[0]) / (ds[0] + eps)
    dk[-1] = (kappa[-1] - kappa[-2]) / (ds[-1] + eps)
    return dk


def _speed_profile_time_scaling(
    s: np.ndarray,
    kappa: np.ndarray,
    limits: Limits
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute a feasible speed profile v(s) satisfying:
      - v <= v_max
      - |v^2 * kappa| <= a_lat_max  (lateral accel bound)
      - |dv/dt| <= a_long_max      (longitudinal accel bound)

    Uses a standard forward-backward pass on v^2 with respect to arc length:
      v_{i+1}^2 <= v_i^2 + 2*a_long_max*ds
      v_i^2     <= v_{i+1}^2 + 2*a_long_max*ds
    Returns:
      v[i] (m/s), and dt[i] for each segment i->i+1.
    """
    n = s.size
    ds = np.diff(s)
    eps = 1e-12

    # Speed upper bound per point from lateral accel + actuator cap
    v_cap = np.full(n, limits.v_max_m_s, dtype=float)
    for i in range(n):
        if abs(kappa[i]) > 1e-9:
            v_lat = math.sqrt(max(limits.a_lat_max_m_s2 / abs(kappa[i]), 0.0))
            v_cap[i] = min(v_cap[i], v_lat)

    # Work with w = v^2 for convex-ish propagation under accel bounds
    w = np.minimum(v_cap, limits.v_max_m_s) ** 2

    # Forward pass (accelerating)
    for i in range(n - 1):
        w[i + 1] = min(w[i + 1], w[i] + 2.0 * limits.a_long_max_m_s2 * max(ds[i], 0.0))

    # Backward pass (decelerating)
    for i in range(n - 2, -1, -1):
        w[i] = min(w[i], w[i + 1] + 2.0 * limits.a_long_max_m_s2 * max(ds[i], 0.0))

    v = np.sqrt(np.maximum(w, 0.0))

    # Segment time using trapezoidal integration: dt = ds / v_avg
    dt = np.zeros(n - 1, dtype=float)
    for i in range(n - 1):
        v_avg = 0.5 * (v[i] + v[i + 1])
        dt[i] = ds[i] / (v_avg + eps)
    return v, dt


def _steering_from_curvature(kappa: np.ndarray, wheelbase_m: float) -> np.ndarray:
    """Ackermann relation: kappa = tan(delta)/L  => delta = atan(L*kappa)."""
    return np.arctan(wheelbase_m * kappa)


def _check_collision_disk(
    xy: np.ndarray,
    occ: np.ndarray,
    resolution_m: float,
    origin_xy_m: Tuple[float, float],
    limits: Limits
) -> Tuple[bool, Dict[str, Any]]:
    """
    Disk-footprint collision check against occupancy grid.
    occ: 2D array (H,W) with 1 = obstacle, 0 = free.
    resolution_m: meters per cell
    origin_xy_m: world coordinate of grid cell (0,0) corner (x0,y0)

    Condition: dist(center, obstacles) >= robot_radius + clearance_margin
    Uses distance transform in grid coordinates.
    """
    H, W = occ.shape
    x0, y0 = origin_xy_m
    needed = (limits.robot_radius_m + limits.clearance_margin_m)

    if _HAS_SCIPY:
        # distance_transform_edt returns distance in cells to nearest zero (False).
        # We want distance to obstacles (occ==1), so invert: free==1 => distance to obstacle
        free = (occ == 0)
        dist_cells = distance_transform_edt(free)
        dist_m = dist_cells * resolution_m
    else:
        # Fallback: crude 8-neighborhood multi-source BFS with chamfer costs
        dist_m = _fallback_distance_transform(occ, resolution_m)

    min_clear = float("inf")
    worst_idx = -1

    for i, (x, y) in enumerate(xy):
        gx = int(math.floor((x - x0) / resolution_m))
        gy = int(math.floor((y - y0) / resolution_m))
        if gx < 0 or gx >= W or gy < 0 or gy >= H:
            return False, {"reason": "path_outside_grid", "index": i}
        c = float(dist_m[gy, gx])
        if c < min_clear:
            min_clear = c
            worst_idx = i
        if c < needed:
            return False, {"reason": "insufficient_clearance", "index": i, "clearance_m": c, "needed_m": needed}

    return True, {"min_clearance_m": min_clear, "worst_index": worst_idx, "needed_m": needed}


def _fallback_distance_transform(occ: np.ndarray, resolution_m: float) -> np.ndarray:
    """
    Simple Dijkstra distance-to-obstacle on a grid (8-connected).
    Returns distances in meters. Obstacles have distance 0.
    """
    import heapq
    H, W = occ.shape
    INF = 1e18
    dist = np.full((H, W), INF, dtype=float)

    pq = []
    for y in range(H):
        for x in range(W):
            if occ[y, x] == 1:
                dist[y, x] = 0.0
                heapq.heappush(pq, (0.0, y, x))

    # 8-connected moves with costs (1 or sqrt(2)) * resolution
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
             (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]

    while pq:
        d, y, x = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
        for dy, dx, w in moves:
            ny, nx = y + dy, x + dx
            if 0 <= ny < H and 0 <= nx < W:
                nd = d + w * resolution_m
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, ny, nx))
    return dist


def check_path_feasibility(
    xy: np.ndarray,
    occ: Optional[np.ndarray],
    resolution_m: float = 0.05,
    origin_xy_m: Tuple[float, float] = (0.0, 0.0),
    limits: Optional[Limits] = None
) -> FeasibilityReport:
    """
    Main feasibility check.
    Inputs:
      xy: N x 2 array of (x,y) world coordinates along candidate path.
      occ: occupancy grid (H x W) with 1 obstacle, 0 free; may be None to skip collision.
    """
    if limits is None:
        limits = Limits(
            wheelbase_m=0.33,
            delta_max_rad=0.45,
            delta_dot_max_rad_s=0.75,
            v_max_m_s=1.2,
            a_long_max_m_s2=0.8,
            a_lat_max_m_s2=1.5,
            robot_radius_m=0.25,
            clearance_margin_m=0.05,
        )

    if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] < 3:
        return FeasibilityReport(False, "invalid_path_shape", {})

    s = _arc_length(xy)
    kappa = _curvature_discrete(xy, s)
    dk_ds = _dkappa_ds(kappa, s)

    # Steering feasibility
    kappa_max = math.tan(limits.delta_max_rad) / limits.wheelbase_m
    if np.max(np.abs(kappa)) > (kappa_max + 1e-9):
        return FeasibilityReport(False, "curvature_limit_violation", {
            "kappa_max": kappa_max,
            "kappa_abs_max": float(np.max(np.abs(kappa))),
        })

    # Time scaling for v(s), dt
    v, dt = _speed_profile_time_scaling(s, kappa, limits)

    # Steering rate feasibility: delta_dot = (d delta/dt)
    delta = _steering_from_curvature(kappa, limits.wheelbase_m)
    # kappa_dot = v * dkappa/ds, delta_dot = (L * cos^2(delta)) * kappa_dot
    kappa_dot = v * dk_ds
    delta_dot = limits.wheelbase_m * (np.cos(delta) ** 2) * kappa_dot
    if np.max(np.abs(delta_dot)) > (limits.delta_dot_max_rad_s + 1e-9):
        return FeasibilityReport(False, "steering_rate_violation", {
            "delta_dot_max": limits.delta_dot_max_rad_s,
            "delta_dot_abs_max": float(np.max(np.abs(delta_dot))),
        })

    # Collision feasibility (disk approximation)
    if occ is not None:
        ok_col, col_stats = _check_collision_disk(xy, occ, resolution_m, origin_xy_m, limits)
        if not ok_col:
            return FeasibilityReport(False, "collision_violation", col_stats)
    else:
        col_stats = {"skipped": True}

    return FeasibilityReport(True, "ok", {
        "path_length_m": float(s[-1]),
        "kappa_abs_max": float(np.max(np.abs(kappa))),
        "kappa_max": float(kappa_max),
        "v_min_m_s": float(np.min(v)),
        "v_max_m_s": float(np.max(v)),
        "delta_abs_max_rad": float(np.max(np.abs(delta))),
        "delta_dot_abs_max_rad_s": float(np.max(np.abs(delta_dot))),
        "total_time_s": float(np.sum(dt)),
        "collision": col_stats,
    })


def _demo() -> None:
    # Simple demo: an arc that is feasible, and a grid with a single obstacle.
    t = np.linspace(0.0, 1.0, 200)
    R = 4.0
    ang = 0.6 * t
    xy = np.column_stack([R * np.sin(ang), R * (1.0 - np.cos(ang))])

    # Occupancy grid 10m x 10m with 0.05m resolution
    res = 0.05
    H = W = int(10.0 / res)
    occ = np.zeros((H, W), dtype=np.uint8)
    # Place a small obstacle block near (2.0, 1.0)
    x0, y0 = 0.0, 0.0
    ox, oy = 2.0, 1.0
    gx = int((ox - x0) / res)
    gy = int((oy - y0) / res)
    occ[max(0, gy - 3):min(H, gy + 3), max(0, gx - 3):min(W, gx + 3)] = 1

    rep = check_path_feasibility(xy, occ, resolution_m=res, origin_xy_m=(x0, y0))
    print(rep.ok, rep.reason)
    print(rep.stats)


if __name__ == "__main__":
    _demo()
