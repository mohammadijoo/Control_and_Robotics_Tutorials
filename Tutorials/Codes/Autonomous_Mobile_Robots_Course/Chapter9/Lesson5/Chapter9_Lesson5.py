"""
Chapter 9 — Mapping Representations for Mobile Robots
Lesson 5 (Lab): Build a 2D Occupancy Grid from LiDAR

This script is self-contained:
1) It defines a simple 2D world with circular obstacles and boundary walls.
2) It simulates a robot trajectory and a 2D LiDAR (range-only beams).
3) It builds an occupancy grid using an inverse sensor model + log-odds updates.
4) It visualizes the resulting occupancy probabilities.

Dependencies: numpy, matplotlib
Install: pip install numpy matplotlib
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt


def logit(p: float) -> float:
    p = min(max(p, 1e-9), 1.0 - 1e-9)
    return math.log(p / (1.0 - p))


def logistic(l: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-l))


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float  # radians


@dataclass
class CircleObs:
    cx: float
    cy: float
    r: float


@dataclass
class World:
    # axis-aligned bounding box and a list of circular obstacles
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    circles: List[CircleObs]


def ray_circle_intersection(px: float, py: float, dx: float, dy: float, c: CircleObs) -> float | None:
    """
    Ray: p + t d, with t >= 0, ||d|| = 1.
    Returns smallest t (distance) to circle boundary if intersection exists.
    """
    ox, oy = px - c.cx, py - c.cy
    b = 2.0 * (ox * dx + oy * dy)
    cterm = ox * ox + oy * oy - c.r * c.r
    disc = b * b - 4.0 * cterm
    if disc < 0:
        return None
    s = math.sqrt(disc)
    t1 = (-b - s) / 2.0
    t2 = (-b + s) / 2.0
    ts = [t for t in (t1, t2) if t >= 0.0]
    return min(ts) if ts else None


def ray_aabb_intersection(px: float, py: float, dx: float, dy: float, xmin: float, xmax: float, ymin: float, ymax: float) -> float | None:
    """
    Ray-AABB intersection using slab method.
    Returns smallest positive t where ray hits boundary.
    """
    tmin, tmax = -math.inf, math.inf

    if abs(dx) < 1e-12:
        if px < xmin or px > xmax:
            return None
    else:
        tx1 = (xmin - px) / dx
        tx2 = (xmax - px) / dx
        tmin = max(tmin, min(tx1, tx2))
        tmax = min(tmax, max(tx1, tx2))

    if abs(dy) < 1e-12:
        if py < ymin or py > ymax:
            return None
    else:
        ty1 = (ymin - py) / dy
        ty2 = (ymax - py) / dy
        tmin = max(tmin, min(ty1, ty2))
        tmax = min(tmax, max(ty1, ty2))

    if tmax < 0.0 or tmin > tmax:
        return None
    # We want first intersection in front of ray origin
    t = tmin if tmin >= 0.0 else tmax
    return t if t >= 0.0 else None


def simulate_lidar(world: World, pose: Pose2D, angles_body: np.ndarray, z_max: float, sigma_r: float = 0.01) -> np.ndarray:
    """
    Returns ranges for each beam angle (in robot/body frame).
    """
    ranges = np.full_like(angles_body, z_max, dtype=float)
    px, py, th = pose.x, pose.y, pose.theta

    for k, a in enumerate(angles_body):
        ang = th + a
        dx, dy = math.cos(ang), math.sin(ang)

        hits = []
        # boundary walls
        tb = ray_aabb_intersection(px, py, dx, dy, world.xmin, world.xmax, world.ymin, world.ymax)
        if tb is not None:
            hits.append(tb)
        # circles
        for c in world.circles:
            t = ray_circle_intersection(px, py, dx, dy, c)
            if t is not None:
                hits.append(t)

        if hits:
            r = min(hits)
            if r <= z_max:
                r = r + np.random.normal(0.0, sigma_r)
                ranges[k] = float(np.clip(r, 0.0, z_max))

    return ranges


class OccupancyGrid:
    def __init__(self, width_m: float, height_m: float, res: float, origin_x: float, origin_y: float,
                 p0: float = 0.5, l_min: float = -10.0, l_max: float = 10.0):
        self.res = float(res)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.w = int(math.ceil(width_m / res))
        self.h = int(math.ceil(height_m / res))
        self.l0 = logit(p0)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.log_odds = np.full((self.h, self.w), self.l0, dtype=float)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int] | None:
        i = int(math.floor((x - self.origin_x) / self.res))
        j = int(math.floor((y - self.origin_y) / self.res))
        if 0 <= i < self.w and 0 <= j < self.h:
            return i, j
        return None

    def grid_to_world_center(self, i: int, j: int) -> Tuple[float, float]:
        x = self.origin_x + (i + 0.5) * self.res
        y = self.origin_y + (j + 0.5) * self.res
        return x, y

    @staticmethod
    def bresenham(i0: int, j0: int, i1: int, j1: int) -> List[Tuple[int, int]]:
        """
        2D Bresenham line traversal (grid indices).
        Returns list of (i,j) cells on the line including both endpoints.
        """
        cells = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0
        while True:
            cells.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return cells

    def update_ray(self, pose: Pose2D, angle_body: float, r: float, z_max: float,
                   l_occ: float, l_free: float, alpha: float = 0.2):
        """
        Inverse sensor model update along one LiDAR ray.
        - Cells before endpoint: free
        - Endpoint cell (if r < z_max - alpha/2): occupied
        - If no hit (r close to z_max): free up to max range, no occupied endpoint
        """
        start = self.world_to_grid(pose.x, pose.y)
        if start is None:
            return
        i0, j0 = start

        ang = pose.theta + angle_body
        ex = pose.x + r * math.cos(ang)
        ey = pose.y + r * math.sin(ang)
        end = self.world_to_grid(ex, ey)
        if end is None:
            # ray endpoint outside map; clip by stepping until outside (simple approach)
            return
        i1, j1 = end

        cells = self.bresenham(i0, j0, i1, j1)
        if len(cells) <= 1:
            return

        hit = (r < (z_max - 0.5 * alpha))
        # free cells: exclude the endpoint; for no-hit, all cells on line are free (excluding robot cell is optional)
        free_cells = cells[1:-1] if hit else cells[1:]
        for (i, j) in free_cells:
            self.log_odds[j, i] = np.clip(self.log_odds[j, i] + (l_free - self.l0), self.l_min, self.l_max)

        if hit:
            (ie, je) = cells[-1]
            self.log_odds[je, ie] = np.clip(self.log_odds[je, ie] + (l_occ - self.l0), self.l_min, self.l_max)

    def probs(self) -> np.ndarray:
        return logistic(self.log_odds)


def main():
    np.random.seed(7)

    # World definition (meters)
    world = World(
        xmin=-10.0, xmax=10.0, ymin=-10.0, ymax=10.0,
        circles=[
            CircleObs(-3.0, 2.0, 1.2),
            CircleObs(2.5, -1.0, 1.0),
            CircleObs(4.0, 4.0, 1.5),
            CircleObs(-4.5, -4.0, 1.0),
        ]
    )

    # LiDAR parameters
    n_beams = 360
    z_max = 8.0
    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)

    # Occupancy grid parameters
    res = 0.1  # meters/cell
    grid = OccupancyGrid(width_m=20.0, height_m=20.0, res=res, origin_x=-10.0, origin_y=-10.0,
                         p0=0.5, l_min=-8.0, l_max=8.0)

    # Inverse sensor model probabilities
    p_occ = 0.70
    p_free = 0.30
    l_occ = logit(p_occ)
    l_free = logit(p_free)
    alpha = 0.2  # obstacle thickness (m), affects "hit" decision band

    # Robot trajectory (known poses)
    T = 220
    poses = []
    for t in range(T):
        # a smooth loop
        ang = 2.0 * math.pi * t / T
        x = 6.0 * math.cos(ang)
        y = 6.0 * math.sin(ang)
        theta = ang + math.pi / 2.0  # tangential heading
        poses.append(Pose2D(x, y, theta))

    # Mapping loop
    for pose in poses:
        z = simulate_lidar(world, pose, angles, z_max=z_max, sigma_r=0.02)
        for a, r in zip(angles, z):
            grid.update_ray(pose, a, r, z_max=z_max, l_occ=l_occ, l_free=l_free, alpha=alpha)

    # Visualization
    P = grid.probs()
    # Convert to occupancy map: 1=occupied, 0=free
    fig = plt.figure(figsize=(7, 6))
    plt.imshow(P, origin='lower', extent=[world.xmin, world.xmax, world.ymin, world.ymax])
    plt.colorbar(label="p(occupied)")
    plt.title("2D Occupancy Grid from Simulated LiDAR")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    # Plot true obstacles
    th = np.linspace(0, 2 * math.pi, 200)
    for c in world.circles:
        plt.plot(c.cx + c.r * np.cos(th), c.cy + c.r * np.sin(th), linewidth=1.5)

    # Plot trajectory
    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    plt.plot(xs, ys, linewidth=1.0)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
