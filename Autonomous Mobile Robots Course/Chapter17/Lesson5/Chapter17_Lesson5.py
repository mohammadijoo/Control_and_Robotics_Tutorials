# Chapter17_Lesson5.py
"""
Autonomous Mobile Robots — Chapter 17 (Exploration and Active Mapping)
Lesson 5 Lab: Autonomous Exploration in Unknown Map (single robot)

This is a self-contained grid-world simulator for autonomous exploration:
- Occupancy grid belief (log-odds)
- Frontier detection (unknown cells adjacent to known-free)
- Goal selection by a score: w_ig * IG - w_cost * path_cost - w_risk * risk
- Simple motion: follow shortest path in known-free + unknown (penalized)

Notes
-----
1) This file is designed to run without ROS.
2) For ROS2 integration, treat:
   - belief update as your mapping node (slam_toolbox / cartographer)
   - frontier detection + goal scoring as an "exploration manager" node
   - send chosen goal to Nav2 via an action client.

Author: (course material)
"""
from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import heapq

import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def logit(p: float) -> float:
    p = clamp(p, 1e-6, 1 - 1e-6)
    return math.log(p / (1 - p))


def sigmoid(l: float) -> float:
    # stable sigmoid
    if l >= 0:
        z = math.exp(-l)
        return 1 / (1 + z)
    z = math.exp(l)
    return z / (1 + z)


def bernoulli_entropy(p: float) -> float:
    # H(p) = -p log p - (1-p) log (1-p) (nats)
    p = clamp(p, 1e-12, 1 - 1e-12)
    return -p * math.log(p) - (1 - p) * math.log(1 - p)


@dataclass(frozen=True)
class Pose2D:
    x: int
    y: int


class OccupancyGridBelief:
    """
    Log-odds occupancy grid belief.
    - l=0  => p=0.5 unknown
    - l>0  => occupied
    - l<0  => free
    """
    def __init__(self, w: int, h: int, p_occ: float = 0.7, p_free: float = 0.3):
        self.w = w
        self.h = h
        self.l = np.zeros((h, w), dtype=np.float64)  # log-odds
        self.l_occ = logit(p_occ)
        self.l_free = logit(p_free)
        self.l_min = logit(0.02)
        self.l_max = logit(0.98)

    def prob(self) -> np.ndarray:
        # vectorized sigmoid
        return 1.0 / (1.0 + np.exp(-self.l))

    def classify(self, p_occ_th: float = 0.65, p_free_th: float = 0.35) -> np.ndarray:
        """
        Returns int map:
        -1 unknown, 0 free, 1 occupied
        """
        p = self.prob()
        out = -np.ones_like(p, dtype=np.int8)
        out[p >= p_occ_th] = 1
        out[p <= p_free_th] = 0
        return out

    def update_ray(self, cells_free: List[Tuple[int, int]], cell_occ: Optional[Tuple[int, int]]):
        for (x, y) in cells_free:
            self.l[y, x] = clamp(self.l[y, x] + self.l_free, self.l_min, self.l_max)
        if cell_occ is not None:
            x, y = cell_occ
            self.l[y, x] = clamp(self.l[y, x] + self.l_occ, self.l_min, self.l_max)

    def total_entropy(self) -> float:
        p = self.prob()
        # compute per-cell entropy
        # H = -p log p -(1-p) log(1-p)
        return float(np.sum(-p * np.log(np.clip(p, 1e-12, 1.0)) - (1 - p) * np.log(np.clip(1 - p, 1e-12, 1.0))))


class GridWorld:
    """
    Ground-truth map (binary obstacle grid) + range sensor via ray casting.
    """
    def __init__(self, w: int, h: int, obstacle_prob: float = 0.22, seed: int = 1):
        self.w = w
        self.h = h
        rng = np.random.default_rng(seed)
        self.occ = (rng.random((h, w)) < obstacle_prob).astype(np.uint8)

        # Carve borders as occupied to keep robot inside
        self.occ[0, :] = 1
        self.occ[-1, :] = 1
        self.occ[:, 0] = 1
        self.occ[:, -1] = 1

        # Ensure connectivity-ish by carving a few random corridors
        for _ in range(8):
            x = int(rng.integers(2, w - 2))
            y0 = int(rng.integers(2, h - 2))
            y1 = int(rng.integers(2, h - 2))
            ylo, yhi = min(y0, y1), max(y0, y1)
            self.occ[ylo:yhi + 1, x] = 0

        for _ in range(8):
            y = int(rng.integers(2, h - 2))
            x0 = int(rng.integers(2, w - 2))
            x1 = int(rng.integers(2, w - 2))
            xlo, xhi = min(x0, x1), max(x0, x1)
            self.occ[y, xlo:xhi + 1] = 0

    def is_occ(self, x: int, y: int) -> bool:
        return bool(self.occ[y, x])

    @staticmethod
    def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Bresenham line algorithm: returns list of grid cells from (x0,y0) to (x1,y1), inclusive.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    def cast_ray(self, x: int, y: int, theta: float, r_max: int) -> Tuple[List[Tuple[int, int]], Optional[Tuple[int, int]]]:
        """
        Ray cast from (x,y) at angle theta up to range r_max (cells).
        Returns (free_cells, occ_cell_or_None).
        """
        x1 = int(round(x + r_max * math.cos(theta)))
        y1 = int(round(y + r_max * math.sin(theta)))
        x1 = clamp(x1, 0, self.w - 1)
        y1 = clamp(y1, 0, self.h - 1)
        line = self.bresenham(x, y, int(x1), int(y1))
        free = []
        occ = None
        for (cx, cy) in line[1:]:  # skip origin
            if self.is_occ(cx, cy):
                occ = (cx, cy)
                break
            free.append((cx, cy))
        return free, occ

    def sense(self, pose: Pose2D, fov: float = math.pi * 2, n_rays: int = 36, r_max: int = 10) -> List[Tuple[List[Tuple[int, int]], Optional[Tuple[int, int]]]]:
        """
        2D lidar-like sensing: cast n_rays across FOV.
        """
        out = []
        start = -fov / 2.0
        for k in range(n_rays):
            theta = start + (k / max(1, n_rays - 1)) * fov
            free, occ = self.cast_ray(pose.x, pose.y, theta, r_max)
            out.append((free, occ))
        return out


def neighbors4(x: int, y: int) -> List[Tuple[int, int]]:
    return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]


def frontier_cells(class_map: np.ndarray) -> List[Tuple[int, int]]:
    """
    Frontier: unknown cell adjacent to known-free cell.
    class_map: -1 unknown, 0 free, 1 occupied
    """
    h, w = class_map.shape
    fronts = []
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if class_map[y, x] != -1:
                continue
            for nx, ny in neighbors4(x, y):
                if class_map[ny, nx] == 0:
                    fronts.append((x, y))
                    break
    return fronts


def cluster_frontiers(fronts: List[Tuple[int, int]], max_dist: int = 2) -> List[List[Tuple[int, int]]]:
    """
    Simple BFS clustering of frontier cells by Manhattan distance threshold.
    """
    fronts_set = set(fronts)
    clusters = []
    visited = set()
    for cell in fronts:
        if cell in visited:
            continue
        q = [cell]
        visited.add(cell)
        cluster = []
        while q:
            x, y = q.pop()
            cluster.append((x, y))
            # explore nearby cells (4-neighbors)
            for nx, ny in neighbors4(x, y):
                if (nx, ny) in fronts_set and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    q.append((nx, ny))
        clusters.append(cluster)
    # filter tiny clusters
    clusters = [c for c in clusters if len(c) >= 5]
    return clusters


def cluster_centroid(cluster: List[Tuple[int, int]]) -> Tuple[int, int]:
    xs = [c[0] for c in cluster]
    ys = [c[1] for c in cluster]
    return int(round(sum(xs) / len(xs))), int(round(sum(ys) / len(ys)))


def dijkstra_cost(class_map: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[float, Dict[Tuple[int, int], Tuple[int, int]]]:
    """
    Dijkstra on grid with costs:
    - occupied: forbidden
    - free: cost 1
    - unknown: cost 3 (penalize)
    Returns (cost, parent dict) for path recovery.
    """
    h, w = class_map.shape
    sx, sy = start
    gx, gy = goal
    INF = 1e18
    dist = {start: 0.0}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
    pq = [(0.0, start)]
    visited = set()
    while pq:
        d, (x, y) = heapq.heappop(pq)
        if (x, y) in visited:
            continue
        visited.add((x, y))
        if (x, y) == (gx, gy):
            return d, parent
        for nx, ny in neighbors4(x, y):
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            if class_map[ny, nx] == 1:
                continue
            step = 1.0 if class_map[ny, nx] == 0 else 3.0
            nd = d + step
            if nd < dist.get((nx, ny), INF):
                dist[(nx, ny)] = nd
                parent[(nx, ny)] = (x, y)
                heapq.heappush(pq, (nd, (nx, ny)))
    return float("inf"), parent


def recover_path(parent: Dict[Tuple[int, int], Tuple[int, int]], start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    if goal not in parent and goal != start:
        return []
    path = [goal]
    cur = goal
    while cur != start:
        cur = parent.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path


def approx_information_gain(belief: OccupancyGridBelief, pose: Pose2D, r_max: int = 10) -> float:
    """
    Approximate IG for a candidate pose without simulating future measurements.
    Heuristic: count currently-unknown cells within a disk of radius r_max
    and multiply by ln(2), since unknown p=0.5 has entropy ln(2).
    """
    class_map = belief.classify()
    h, w = class_map.shape
    x0, y0 = pose.x, pose.y
    unknown = 0
    r2 = r_max * r_max
    for y in range(max(0, y0 - r_max), min(h, y0 + r_max + 1)):
        for x in range(max(0, x0 - r_max), min(w, x0 + r_max + 1)):
            if (x - x0) * (x - x0) + (y - y0) * (y - y0) <= r2:
                if class_map[y, x] == -1:
                    unknown += 1
    return unknown * math.log(2.0)


def path_risk(belief: OccupancyGridBelief, path: List[Tuple[int, int]]) -> float:
    """
    Risk proxy: sum of occupancy probabilities along the path.
    """
    if not path:
        return float("inf")
    p = belief.prob()
    r = 0.0
    for (x, y) in path:
        r += float(p[y, x])
    return r / len(path)


class Explorer:
    def __init__(self, world: GridWorld, belief: OccupancyGridBelief):
        self.world = world
        self.belief = belief
        self.pose = Pose2D(2, 2)
        self.steps = 0
        self.travel = 0.0
        self.history_entropy = []
        self.history_known = []

        # scoring weights (tunable)
        self.w_ig = 1.0
        self.w_cost = 0.35
        self.w_risk = 1.2

        # sensor params
        self.n_rays = 48
        self.r_max = 10

    def initialize_pose_free(self):
        # find a free cell in ground truth
        for y in range(1, self.world.h - 1):
            for x in range(1, self.world.w - 1):
                if not self.world.is_occ(x, y):
                    self.pose = Pose2D(x, y)
                    return

    def mapping_update(self):
        scans = self.world.sense(self.pose, n_rays=self.n_rays, r_max=self.r_max)
        for free, occ in scans:
            self.belief.update_ray(free, occ)

    def choose_goal(self) -> Optional[Tuple[int, int]]:
        class_map = self.belief.classify()
        fronts = frontier_cells(class_map)
        if len(fronts) == 0:
            return None
        clusters = cluster_frontiers(fronts)
        if len(clusters) == 0:
            return None
        candidates = []
        start = (self.pose.x, self.pose.y)
        for cluster in clusters:
            gx, gy = cluster_centroid(cluster)

            # if centroid is occupied in belief, skip
            if class_map[gy, gx] == 1:
                continue

            # path cost
            cost, parent = dijkstra_cost(class_map, start, (gx, gy))
            if not math.isfinite(cost):
                continue
            path = recover_path(parent, start, (gx, gy))
            if len(path) < 2:
                continue

            # IG + risk
            ig = approx_information_gain(self.belief, Pose2D(gx, gy), r_max=self.r_max)
            risk = path_risk(self.belief, path)

            score = self.w_ig * ig - self.w_cost * cost - self.w_risk * risk
            candidates.append((score, (gx, gy), cost, ig, risk, len(cluster)))

        if not candidates:
            return None
        candidates.sort(key=lambda t: t[0], reverse=True)
        # greedy pick best
        best = candidates[0]
        return best[1]

    def step_toward(self, goal: Tuple[int, int]) -> bool:
        class_map = self.belief.classify()
        start = (self.pose.x, self.pose.y)
        cost, parent = dijkstra_cost(class_map, start, goal)
        path = recover_path(parent, start, goal)
        if len(path) < 2:
            return False
        # move one step
        nx, ny = path[1]
        # collision check against ground truth
        if self.world.is_occ(nx, ny):
            # collision: mark it as occupied strongly and stop this step
            self.belief.l[ny, nx] = self.belief.l_max
            return False
        self.pose = Pose2D(nx, ny)
        self.travel += 1.0
        return True

    def log_metrics(self):
        self.history_entropy.append(self.belief.total_entropy())
        class_map = self.belief.classify()
        known = float(np.mean(class_map != -1))
        self.history_known.append(known)

    def run(self, max_steps: int = 2000) -> Dict[str, float]:
        self.initialize_pose_free()
        # initial sensing
        self.mapping_update()
        self.log_metrics()

        for _ in range(max_steps):
            self.steps += 1
            goal = self.choose_goal()
            if goal is None:
                break
            moved = self.step_toward(goal)
            # always sense after (even if not moved, to mimic rescan + update)
            self.mapping_update()
            self.log_metrics()
            if not moved:
                # try again next loop
                continue
        # summary metrics
        return {
            "steps": float(self.steps),
            "travel": float(self.travel),
            "known_fraction": float(self.history_known[-1] if self.history_known else 0.0),
            "entropy_final": float(self.history_entropy[-1] if self.history_entropy else 0.0),
            "entropy_initial": float(self.history_entropy[0] if self.history_entropy else 0.0),
        }


def main():
    # Lab parameters
    w, h = 60, 45
    world = GridWorld(w, h, obstacle_prob=0.22, seed=7)
    belief = OccupancyGridBelief(w, h, p_occ=0.72, p_free=0.28)

    explorer = Explorer(world, belief)
    # weight tuning (typical starting point)
    explorer.w_ig = 1.0
    explorer.w_cost = 0.35
    explorer.w_risk = 1.2

    metrics = explorer.run(max_steps=2500)

    print("=== Exploration Summary ===")
    for k, v in metrics.items():
        print(f"{k:>16s}: {v:.4f}" if isinstance(v, float) else f"{k:>16s}: {v}")

    # Optional: visualize using matplotlib if installed
    try:
        import matplotlib.pyplot as plt
        cls = belief.classify()
        plt.figure()
        plt.title("Belief map: -1 unknown, 0 free, 1 occupied")
        plt.imshow(cls, origin="lower")
        plt.figure()
        plt.title("Known fraction over time")
        plt.plot(explorer.history_known)
        plt.figure()
        plt.title("Total entropy over time")
        plt.plot(explorer.history_entropy)
        plt.show()
    except Exception as e:
        print("Matplotlib visualization skipped:", e)


if __name__ == "__main__":
    main()
