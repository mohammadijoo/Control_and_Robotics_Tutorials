
#!/usr/bin/env python3
"""
Chapter8_Lesson3.py

Autonomous Mobile Robots (Control Engineering) - Chapter 8, Lesson 3
Sensor Likelihoods for LiDAR and Vision

This file provides:
1) A simple 2D occupancy grid and an approximate Euclidean distance transform
   (multi-source Dijkstra / brushfire).
2) A LiDAR likelihood-field sensor model (endpoint distance to nearest obstacle).
3) A bearing-only vision landmark likelihood with an explicit outlier mixture.
4) Numerically stable log-weight normalization (log-sum-exp).

Dependencies:
- Standard library only.
Optional accelerations (not required):
- numpy, scipy for faster distance transforms and vectorization.

Typical robotics integration:
- ROS/ROS2: subscribe to sensor_msgs/LaserScan, run the likelihood on each particle.
- Map source: nav_msgs/OccupancyGrid (2D) or a costmap.
"""

from __future__ import annotations
from dataclasses import dataclass
import heapq
import math
from typing import List, Tuple, Dict


# -----------------------------
# Geometry helpers
# -----------------------------
def wrap_to_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi)
    return a - math.pi


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float  # radians


# -----------------------------
# Occupancy grid and distance field
# -----------------------------
@dataclass
class OccupancyGrid2D:
    """
    Binary occupancy grid.
    occ[y][x] = 1 for obstacle, 0 for free.

    Coordinate convention:
    - World: meters
    - Grid: integer indices (gx, gy)
    - origin: world coordinate of grid cell (0,0) corner
    """
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occ: List[List[int]]  # shape [height][width], 0/1

    @staticmethod
    def empty(width: int, height: int, resolution: float, origin_x: float, origin_y: float) -> "OccupancyGrid2D":
        occ = [[0 for _ in range(width)] for _ in range(height)]
        return OccupancyGrid2D(width, height, resolution, origin_x, origin_y, occ)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def set_obstacle(self, gx: int, gy: int) -> None:
        if self.in_bounds(gx, gy):
            self.occ[gy][gx] = 1

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy


def distance_transform_brushfire(grid: OccupancyGrid2D) -> List[List[float]]:
    """
    Approximate Euclidean distance transform using multi-source Dijkstra on an 8-connected grid.

    Returns:
        dist[y][x] in meters, distance to nearest obstacle cell center (approx).
    """
    INF = 1e18
    dist = [[INF for _ in range(grid.width)] for _ in range(grid.height)]
    pq: List[Tuple[float, int, int]] = []

    # Initialize with all obstacle cells as sources (distance 0)
    for gy in range(grid.height):
        for gx in range(grid.width):
            if grid.occ[gy][gx] == 1:
                dist[gy][gx] = 0.0
                heapq.heappush(pq, (0.0, gx, gy))

    # 8-neighborhood: (dx, dy, step_cost)
    nbh = [
        (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))
    ]

    while pq:
        d, gx, gy = heapq.heappop(pq)
        if d > dist[gy][gx]:
            continue
        for dx, dy, c in nbh:
            nx, ny = gx + dx, gy + dy
            if not grid.in_bounds(nx, ny):
                continue
            nd = d + c
            if nd < dist[ny][nx]:
                dist[ny][nx] = nd
                heapq.heappush(pq, (nd, nx, ny))

    # Convert from cell-steps to meters
    for gy in range(grid.height):
        for gx in range(grid.width):
            dist[gy][gx] *= grid.resolution
    return dist


# -----------------------------
# LiDAR likelihood-field model
# -----------------------------
@dataclass
class LikelihoodFieldModel:
    grid: OccupancyGrid2D
    dist_field_m: List[List[float]]  # meters
    z_max: float
    sigma_hit: float
    w_hit: float
    w_rand: float
    max_dist: float = 2.0  # clamp distance for numerical robustness

    def endpoint_distance(self, wx: float, wy: float) -> float:
        gx, gy = self.grid.world_to_grid(wx, wy)
        if not self.grid.in_bounds(gx, gy):
            return self.max_dist
        return min(self.max_dist, self.dist_field_m[gy][gx])

    def log_likelihood(self, pose: Pose2D, ranges: List[float], rel_angles: List[float]) -> float:
        """
        Likelihood-field:
            p(z^i | x, m) ≈ w_hit * exp(-d^2/(2*sigma^2)) + w_rand*(1/z_max)
        """
        sig2 = self.sigma_hit * self.sigma_hit
        inv_z = 1.0 / self.z_max
        logp = 0.0

        for z, a_rel in zip(ranges, rel_angles):
            zc = min(max(z, 0.0), self.z_max)
            a = pose.theta + a_rel
            wx = pose.x + zc * math.cos(a)
            wy = pose.y + zc * math.sin(a)

            d = self.endpoint_distance(wx, wy)
            p_hit = math.exp(-(d * d) / (2.0 * sig2))
            p = self.w_hit * p_hit + self.w_rand * inv_z
            logp += math.log(max(p, 1e-12))
        return logp


# -----------------------------
# Vision bearing-only landmark likelihood
# -----------------------------
@dataclass
class VisionBearingModel:
    landmarks: Dict[int, Tuple[float, float]]  # id -> (x, y)
    sigma_bearing: float
    eps_outlier: float

    def log_likelihood(self, pose: Pose2D, obs: List[Tuple[int, float]]) -> float:
        """
        Each observation: (landmark_id, measured_bearing_in_robot_frame)
        Robust mixture:
            p = (1-eps)*N_wrap(e;0,sigma^2) + eps*(1/2pi)
        """
        sig2 = self.sigma_bearing * self.sigma_bearing
        norm = 1.0 / math.sqrt(2.0 * math.pi * sig2)
        uni = 1.0 / (2.0 * math.pi)

        logp = 0.0
        for lid, beta in obs:
            if lid not in self.landmarks:
                logp += math.log(uni)
                continue
            lx, ly = self.landmarks[lid]
            pred = wrap_to_pi(math.atan2(ly - pose.y, lx - pose.x) - pose.theta)
            innov = wrap_to_pi(beta - pred)

            p_in = norm * math.exp(-(innov * innov) / (2.0 * sig2))
            p = (1.0 - self.eps_outlier) * p_in + self.eps_outlier * uni
            logp += math.log(max(p, 1e-15))
        return logp


# -----------------------------
# Numerically stable normalization
# -----------------------------
def logsumexp(logw: List[float]) -> float:
    m = max(logw)
    s = sum(math.exp(w - m) for w in logw)
    return m + math.log(s)


def normalize_log_weights(logw: List[float]) -> List[float]:
    lse = logsumexp(logw)
    w = [math.exp(v - lse) for v in logw]
    s = sum(w)
    return [wi / s for wi in w]


# -----------------------------
# Minimal demo
# -----------------------------
def main() -> None:
    # Synthetic map: a wall and a pillar
    grid = OccupancyGrid2D.empty(width=120, height=120, resolution=0.05, origin_x=-3.0, origin_y=-3.0)

    # Wall: y = 0 (approx)
    for gx in range(10, 110):
        grid.set_obstacle(gx, 60)

    # Pillar block
    for gy in range(20, 30):
        for gx in range(85, 95):
            grid.set_obstacle(gx, gy)

    dist = distance_transform_brushfire(grid)

    lidar = LikelihoodFieldModel(
        grid=grid,
        dist_field_m=dist,
        z_max=8.0,
        sigma_hit=0.15,
        w_hit=0.95,
        w_rand=0.05,
        max_dist=2.0
    )

    vision = VisionBearingModel(
        landmarks={
            1: (-1.0,  0.0),
            2: ( 1.0,  0.5),
            3: ( 0.0, -1.0),
        },
        sigma_bearing=math.radians(3.0),
        eps_outlier=0.10
    )

    # Three particles
    particles = [
        Pose2D(-0.5, -0.8, math.radians(90.0)),
        Pose2D(-0.2, -0.6, math.radians(88.0)),
        Pose2D( 0.8,  0.1, math.radians(30.0)),
    ]

    # Synthetic scan: 9 beams centered around forward direction
    rel_angles = [math.radians(-40.0 + 80.0 * i / 8.0) for i in range(9)]
    ranges = [2.8, 3.0, 3.2, 3.1, 3.0, 3.2, 2.9, 2.7, 2.6]

    # Vision observations: (id, bearing)
    obs = [(1, math.radians(70.0)), (2, math.radians(20.0)), (3, math.radians(-95.0))]

    logw = []
    for p in particles:
        ll_lidar = lidar.log_likelihood(p, ranges, rel_angles)
        ll_vis = vision.log_likelihood(p, obs)
        logw.append(ll_lidar + ll_vis)

    w = normalize_log_weights(logw)

    print("log-weights:", [round(v, 3) for v in logw])
    print("weights:", [round(v, 6) for v in w])


if __name__ == "__main__":
    main()
      