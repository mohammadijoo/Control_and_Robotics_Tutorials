"""
Chapter9_Lesson2.py
Autonomous Mobile Robots — Chapter 9, Lesson 2
Log-Odds Updates and Bayesian Cells (Occupancy Grid Mapping)

This script demonstrates a numerically stable log-odds occupancy grid update
using an inverse sensor model for a 2D range sensor (e.g., LiDAR).
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import numpy as np
import matplotlib.pyplot as plt


def logit(p: float) -> float:
    """Log-odds: log(p/(1-p))."""
    p = float(np.clip(p, 1e-9, 1.0 - 1e-9))
    return math.log(p / (1.0 - p))


def logistic(l: float) -> float:
    """Inverse logit: 1/(1+exp(-l))."""
    # Stable sigmoid
    if l >= 0:
        z = math.exp(-l)
        return 1.0 / (1.0 + z)
    else:
        z = math.exp(l)
        return z / (1.0 + z)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """
    Integer grid traversal between two cells.
    Returns list of (x,y) including both endpoints.
    """
    points: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1

    if dy <= dx:
        err = dx // 2
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        points.append((x1, y1))
    else:
        err = dy // 2
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        points.append((x1, y1))
    return points


@dataclass
class OccupancyGridLogOdds:
    """
    Log-odds occupancy grid.

    Coordinate conventions:
      - Map indices (ix, iy) refer to cell centers.
      - World coordinates (x, y) in meters.
      - origin is world coordinate of map cell (0,0) center.
    """
    width: int
    height: int
    resolution: float  # meters/cell
    origin_x: float = 0.0
    origin_y: float = 0.0

    def __post_init__(self) -> None:
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(round((x - self.origin_x) / self.resolution))
        iy = int(round((y - self.origin_y) / self.resolution))
        return ix, iy

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def update_cell(self, ix: int, iy: int, delta_l: float, l_min: float, l_max: float) -> None:
        if not self.in_bounds(ix, iy):
            return
        self.log_odds[iy, ix] = float(np.clip(self.log_odds[iy, ix] + delta_l, l_min, l_max))

    def update_scan(
        self,
        pose_xytheta: Tuple[float, float, float],
        angles: np.ndarray,
        ranges: np.ndarray,
        range_max: float,
        p0: float = 0.5,
        p_occ: float = 0.7,
        p_free: float = 0.3,
        l_min: float = -10.0,
        l_max: float = 10.0,
    ) -> None:
        """
        Update log-odds with a single scan using a classic inverse sensor model:
          - cells along the beam up to (range - 1 cell) are "free"
          - the endpoint cell at the measured range is "occupied" if range < range_max
          - if range == range_max (no return), mark along the ray as free but no occupied endpoint

        Additive log-odds update:
          l_t = l_{t-1} + logit(p(m|z,x)) - logit(p0)
        """
        x, y, th = pose_xytheta
        l0 = logit(p0)
        l_occ = logit(p_occ) - l0
        l_free = logit(p_free) - l0

        start_ix, start_iy = self.world_to_map(x, y)

        for a, r in zip(angles, ranges):
            # Clamp invalid
            if not np.isfinite(r) or r <= 0.0:
                continue
            r_eff = min(float(r), float(range_max))
            # Beam endpoint in world
            bx = x + r_eff * math.cos(th + float(a))
            by = y + r_eff * math.sin(th + float(a))
            end_ix, end_iy = self.world_to_map(bx, by)

            ray = bresenham(start_ix, start_iy, end_ix, end_iy)
            if len(ray) == 0:
                continue

            # Free cells along the ray excluding the last cell
            for (ix, iy) in ray[:-1]:
                self.update_cell(ix, iy, l_free, l_min, l_max)

            # Occupied endpoint if we actually hit something
            if float(r) < float(range_max) - 1e-9:
                ix, iy = ray[-1]
                self.update_cell(ix, iy, l_occ, l_min, l_max)

    def as_probability(self) -> np.ndarray:
        # vectorize stable sigmoid
        v = np.vectorize(logistic)
        return v(self.log_odds)


def demo() -> None:
    # Map
    grid = OccupancyGridLogOdds(width=220, height=220, resolution=0.05, origin_x=-5.5, origin_y=-5.5)

    # Simulated robot poses and scans around a simple obstacle arrangement
    poses = [
        (-2.0, -2.0, 0.2),
        (0.0, -2.5, 0.6),
        (2.0, -1.5, 1.0),
        (2.0, 1.0, 1.6),
        (0.0, 2.5, 2.5),
        (-2.0, 2.0, -2.7),
    ]

    # Angles for a coarse scan
    angles = np.linspace(-math.pi / 2, math.pi / 2, 121)
    range_max = 6.0

    # Synthetic environment: two circular obstacles
    obstacles = [
        (1.5, 0.5, 0.6),   # (cx, cy, radius)
        (-1.2, 1.2, 0.5),
    ]

    def raycast(pose, a) -> float:
        x, y, th = pose
        dx = math.cos(th + a)
        dy = math.sin(th + a)
        # step along ray
        step = 0.02
        t = 0.0
        while t <= range_max:
            px = x + t * dx
            py = y + t * dy
            # collision check
            hit = False
            for (cx, cy, rr) in obstacles:
                if (px - cx) ** 2 + (py - cy) ** 2 <= rr ** 2:
                    hit = True
                    break
            if hit:
                # add a touch of noise
                return max(0.05, t + np.random.normal(0, 0.03))
            t += step
        return range_max  # no return

    np.random.seed(7)
    for pose in poses:
        ranges = np.array([raycast(pose, float(a)) for a in angles], dtype=np.float32)
        grid.update_scan(pose, angles, ranges, range_max=range_max)

    P = grid.as_probability()

    plt.figure(figsize=(6.2, 6.0))
    plt.imshow(P, origin="lower", extent=[grid.origin_x, grid.origin_x + grid.width * grid.resolution,
                                         grid.origin_y, grid.origin_y + grid.height * grid.resolution])
    plt.title("Occupancy probability (log-odds fused)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.colorbar()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    demo()
