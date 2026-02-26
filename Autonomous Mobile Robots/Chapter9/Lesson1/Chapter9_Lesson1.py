"""
Chapter 9 - Mapping Representations for Mobile Robots
Lesson 1 - Occupancy Grid Mapping

This script implements a simple 2D occupancy grid mapper using a log-odds grid and
a very basic inverse sensor model for a planar range sensor (e.g., 2D LiDAR).
It also includes a small synthetic simulation to generate range readings in a toy world.

Dependencies:
  - numpy
  - matplotlib

Run:
  python Chapter9_Lesson1.py
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import numpy as np
import matplotlib.pyplot as plt


def logit(p: float) -> float:
    p = float(np.clip(p, 1e-9, 1.0 - 1e-9))
    return math.log(p / (1.0 - p))


def inv_logit(l: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-l))


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """
    Integer grid traversal for a line segment (x0,y0)->(x1,y1).
    Returns list of (x,y) cells including both endpoints.
    """
    pts: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        pts.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return pts


@dataclass
class OccupancyGrid:
    width: int
    height: int
    resolution: float  # meters per cell
    origin: Tuple[float, float] = (0.0, 0.0)  # world coords of grid (0,0) cell corner
    prior: float = 0.5
    p_occ: float = 0.7
    p_free: float = 0.3
    l_min: float = -10.0
    l_max: float = 10.0

    def __post_init__(self) -> None:
        self.L = np.full((self.height, self.width), logit(self.prior), dtype=np.float64)
        # Log-odds increments (relative to prior odds)
        self.l_occ = logit(self.p_occ) - logit(self.prior)
        self.l_free = logit(self.p_free) - logit(self.prior)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_ray(
        self,
        pose: Tuple[float, float, float],
        rel_angle: float,
        rng: float,
        max_range: float,
    ) -> None:
        """
        Update the grid with a single range measurement.
        pose = (x, y, theta) in world coordinates [m, m, rad]
        rel_angle is sensor ray angle relative to robot heading.
        rng is measured range in meters (clipped to max_range).
        """
        x, y, theta = pose
        rng = float(np.clip(rng, 0.0, max_range))
        a = theta + rel_angle

        # Ray end point in world coordinates
        xe = x + rng * math.cos(a)
        ye = y + rng * math.sin(a)

        x0, y0 = self.world_to_grid(x, y)
        x1, y1 = self.world_to_grid(xe, ye)

        if not self.in_bounds(x0, y0):
            return  # robot outside the map (ignore)
        # Traverse cells; all but last are "free". Last is "occupied" if hit before max_range.
        cells = bresenham(x0, y0, x1, y1)
        if len(cells) == 0:
            return

        hit = (rng < max_range - 1e-6)
        free_cells = cells[:-1] if hit else cells  # if no hit, entire ray is free (to max range)

        for cx, cy in free_cells:
            if self.in_bounds(cx, cy):
                self.L[cy, cx] = np.clip(self.L[cy, cx] + self.l_free, self.l_min, self.l_max)

        if hit:
            cx, cy = cells[-1]
            if self.in_bounds(cx, cy):
                self.L[cy, cx] = np.clip(self.L[cy, cx] + self.l_occ, self.l_min, self.l_max)

    def prob(self) -> np.ndarray:
        return inv_logit(self.L)


def build_toy_world(width: int, height: int) -> np.ndarray:
    """
    Ground truth occupancy for synthetic ray casting.
    0 = free, 1 = occupied
    """
    world = np.zeros((height, width), dtype=np.uint8)
    # Border walls
    world[0, :] = 1
    world[-1, :] = 1
    world[:, 0] = 1
    world[:, -1] = 1
    # A rectangle obstacle
    world[30:50, 55:75] = 1
    # A smaller obstacle
    world[70:80, 25:35] = 1
    return world


def cast_ray_grid(
    world_occ: np.ndarray,
    grid: OccupancyGrid,
    pose: Tuple[float, float, float],
    rel_angle: float,
    max_range: float,
    step: float | None = None,
) -> float:
    """
    Simple ray casting by stepping along the ray in world coords until an occupied cell is hit.
    Returns the measured range.
    """
    if step is None:
        step = grid.resolution * 0.5

    x, y, theta = pose
    a = theta + rel_angle
    dist = 0.0
    while dist < max_range:
        xt = x + dist * math.cos(a)
        yt = y + dist * math.sin(a)
        gx, gy = grid.world_to_grid(xt, yt)
        if 0 <= gx < grid.width and 0 <= gy < grid.height:
            if world_occ[gy, gx] == 1:
                return dist
        else:
            # out of bounds acts like a wall
            return dist
        dist += step
    return max_range


def main() -> None:
    # Map parameters
    W, H = 120, 100
    res = 0.05  # 5 cm per cell
    og = OccupancyGrid(W, H, res, origin=(0.0, 0.0), prior=0.5, p_occ=0.75, p_free=0.35)

    # Ground truth (for generating measurements only)
    gt = build_toy_world(W, H)

    # Robot trajectory (world meters)
    traj = []
    for k in range(25):
        x = 0.8 + 0.06 * k
        y = 0.9 + 0.01 * k
        theta = 0.15  # rad
        traj.append((x, y, theta))

    # Sensor model
    fov = math.radians(180.0)
    n_rays = 121
    angles = np.linspace(-fov / 2.0, fov / 2.0, n_rays)
    z_max = 3.0  # meters

    # Mapping loop
    for pose in traj:
        for ang in angles:
            z = cast_ray_grid(gt, og, pose, float(ang), z_max)
            og.update_ray(pose, float(ang), z, z_max)

    # Visualize: probability of occupancy
    P = og.prob()
    plt.figure(figsize=(7, 5))
    plt.title("Occupancy probability (after mapping)")
    plt.imshow(P, origin="lower", vmin=0.0, vmax=1.0)
    plt.xlabel("grid x")
    plt.ylabel("grid y")
    plt.colorbar(label="p(occupied)")
    plt.tight_layout()
    plt.show()

    # Also visualize ground truth for comparison (optional)
    plt.figure(figsize=(7, 5))
    plt.title("Ground truth occupancy (used only for simulation)")
    plt.imshow(gt, origin="lower", vmin=0, vmax=1)
    plt.xlabel("grid x")
    plt.ylabel("grid y")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
