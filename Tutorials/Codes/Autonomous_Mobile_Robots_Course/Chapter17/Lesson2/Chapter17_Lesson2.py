# -*- coding: utf-8 -*-
"""
Chapter 17 - Lesson 2: Information Gain and Entropy Reduction
Autonomous Mobile Robots (Control Engineering)

This file provides a minimal, dependency-light implementation of:
- Bernoulli (occupancy) entropy
- Per-cell mutual information (information gain) for a binary measurement model
- A simple expected information gain estimator for candidate viewpoints using ray casting

Optional integration notes for ROS/ROS2 occupancy grids are included at the end.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Set
import math
import random


def clip(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def bernoulli_entropy(p: float, eps: float = 1e-12) -> float:
    """
    Shannon entropy of Bernoulli(p) in nats:
        H(p) = -p ln p - (1-p) ln(1-p)
    """
    p = clip(p, eps, 1.0 - eps)
    return -p * math.log(p) - (1.0 - p) * math.log(1.0 - p)


def info_gain_cell(p: float, p_hit: float = 0.85, p_false: float = 0.15) -> float:
    """
    Mutual information I(M;Z) for a single Bernoulli occupancy variable M ~ Bernoulli(p)
    observed through a binary sensor Z in {occ, free} with:
        P(Z=occ | M=occ)  = p_hit
        P(Z=occ | M=free) = p_false

    IG = H(M) - E_Z[ H(M | Z) ].

    Returns IG in nats.
    """
    H_prior = bernoulli_entropy(p)

    # Marginal measurement probabilities
    P_z_occ = p_hit * p + p_false * (1.0 - p)
    P_z_free = 1.0 - P_z_occ

    eps = 1e-15
    P_z_occ = clip(P_z_occ, eps, 1.0 - eps)
    P_z_free = clip(P_z_free, eps, 1.0 - eps)

    # Posteriors
    p_post_given_z_occ = (p_hit * p) / P_z_occ
    p_post_given_z_free = ((1.0 - p_hit) * p) / P_z_free

    # Expected posterior entropy
    H_post = (
        P_z_occ * bernoulli_entropy(p_post_given_z_occ)
        + P_z_free * bernoulli_entropy(p_post_given_z_free)
    )

    return H_prior - H_post


@dataclass
class OccupancyGrid:
    """
    A simple occupancy grid storing probabilities p in [0,1].

    width, height: number of cells
    resolution: meters per cell
    origin_x, origin_y: world coordinate of cell (0,0) lower-left corner
    """
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0

    def __post_init__(self) -> None:
        self.p = [[0.5 for _ in range(self.width)] for _ in range(self.height)]

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))
        return ix, iy

    def map_entropy(self) -> float:
        return sum(bernoulli_entropy(prob) for row in self.p for prob in row)


def ray_cast_cells(
    grid: OccupancyGrid,
    x0: float,
    y0: float,
    theta: float,
    max_range: float,
    step: float | None = None,
) -> List[Tuple[int, int]]:
    """
    Simple ray casting by stepping along the ray and collecting visited grid cells.
    For expected IG, we mainly need which cells are visible (approx).

    step default: half a cell.
    """
    if step is None:
        step = 0.5 * grid.resolution

    cells: List[Tuple[int, int]] = []
    seen: Set[Tuple[int, int]] = set()

    t = 0.0
    while t <= max_range:
        x = x0 + t * math.cos(theta)
        y = y0 + t * math.sin(theta)
        ix, iy = grid.world_to_grid(x, y)
        if not grid.in_bounds(ix, iy):
            break
        key = (ix, iy)
        if key not in seen:
            seen.add(key)
            cells.append(key)
        t += step

    return cells


def expected_info_gain_view(
    grid: OccupancyGrid,
    pose: Tuple[float, float, float],
    fov_deg: float = 180.0,
    n_rays: int = 181,
    max_range: float = 8.0,
    p_hit: float = 0.85,
    p_false: float = 0.15,
    unknown_band: Tuple[float, float] = (0.4, 0.6),
    occ_stop_threshold: float = 0.75,
) -> float:
    """
    Approximate expected information gain from a viewpoint pose=(x,y,yaw).

    We:
      1) Cast rays in a fan (FOV).
      2) Traverse cells along each ray until we hit a likely-occupied cell (occlusion).
      3) Accumulate per-cell IG for "uncertain" cells (near 0.5), under an independent-cell model.

    This is a common active-mapping approximation for occupancy grids.
    """
    x, y, yaw = pose
    fov = math.radians(fov_deg)

    if n_rays < 2:
        angles = [0.0]
    else:
        angles = [(-0.5 * fov) + i * (fov / (n_rays - 1)) for i in range(n_rays)]

    lo_u, hi_u = unknown_band
    visited: Set[Tuple[int, int]] = set()
    ig_total = 0.0

    for a in angles:
        theta = yaw + a
        cells = ray_cast_cells(grid, x, y, theta, max_range=max_range)

        for (ix, iy) in cells:
            if (ix, iy) in visited:
                continue
            visited.add((ix, iy))

            p = grid.p[iy][ix]

            # occlusion / stopping
            if p >= occ_stop_threshold:
                break

            # score only uncertain cells
            if lo_u <= p <= hi_u:
                ig_total += info_gain_cell(p, p_hit=p_hit, p_false=p_false)

    return ig_total


def demo() -> None:
    g = OccupancyGrid(width=120, height=120, resolution=0.1, origin_x=-6.0, origin_y=-6.0)

    random.seed(0)
    for _ in range(4000):
        ix = random.randint(0, g.width - 1)
        iy = random.randint(0, g.height - 1)
        g.p[iy][ix] = clip(random.gauss(0.15, 0.10), 0.01, 0.99)  # mostly free-ish
    for _ in range(1200):
        ix = random.randint(0, g.width - 1)
        iy = random.randint(0, g.height - 1)
        g.p[iy][ix] = clip(random.gauss(0.85, 0.08), 0.01, 0.99)  # mostly occupied-ish

    print("Initial map entropy (nats):", round(g.map_entropy(), 3))
    pose_candidates = [
        (0.0, 0.0, 0.0),
        (-2.0, 1.5, 0.7),
        (2.0, -1.0, -1.2),
    ]
    for pose in pose_candidates:
        ig = expected_info_gain_view(g, pose, fov_deg=180, n_rays=181, max_range=6.0)
        print(f"Pose={pose}  Expected IG≈ {ig:.3f} nats")


if __name__ == "__main__":
    demo()

"""
ROS/ROS2 NOTE (optional):
- nav_msgs/OccupancyGrid stores data as int8 in [-1,100]:
    -1 unknown, 0 free, 100 occupied.
  Typical conversion:
    p = 0.5 if v==-1 else v/100.0
- Use LaserScan angles and max range to define rays.
- The same IG scoring can be used to rank frontier candidates or next-best-view poses.
"""
