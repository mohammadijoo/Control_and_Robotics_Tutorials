"""
Autonomous Mobile Robots — Chapter 17, Lesson 1: Frontier-Based Exploration
File: Chapter17_Lesson1.py

This script demonstrates a self-contained frontier-based exploration loop on a 2D occupancy grid.
Grid encoding (common in ROS nav_msgs/OccupancyGrid):
  -1 = unknown, 0 = free, 100 = occupied

Core steps:
  1) detect frontier cells
  2) cluster frontiers
  3) score clusters by a distance-vs-size utility
  4) choose a reachable goal near the best cluster
  5) (toy) update map to simulate sensing at the goal

NOTE: This is a teaching reference implementation; integrate with ROS2/Nav2 by replacing
the toy "simulate_sensor_update" with real mapping updates and path planning calls.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Set

import numpy as np


Coord = Tuple[int, int]  # (row, col)


@dataclass
class FrontierCluster:
    cells: List[Coord]

    @property
    def size(self) -> int:
        return len(self.cells)

    def centroid(self) -> Tuple[float, float]:
        rs = [c[0] for c in self.cells]
        cs = [c[1] for c in self.cells]
        return (float(np.mean(rs)), float(np.mean(cs)))


def in_bounds(grid: np.ndarray, rc: Coord) -> bool:
    r, c = rc
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]


def neighbors8(grid: np.ndarray, rc: Coord) -> List[Coord]:
    r, c = rc
    nbrs = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < grid.shape[0] and 0 <= cc < grid.shape[1]:
                nbrs.append((rr, cc))
    return nbrs


def neighbors4(grid: np.ndarray, rc: Coord) -> List[Coord]:
    r, c = rc
    nbrs = []
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        rr, cc = r + dr, c + dc
        if 0 <= rr < grid.shape[0] and 0 <= cc < grid.shape[1]:
            nbrs.append((rr, cc))
    return nbrs


def is_free(v: int) -> bool:
    return v == 0


def is_unknown(v: int) -> bool:
    return v == -1


def is_occupied(v: int) -> bool:
    return v >= 50  # common threshold


def detect_frontiers(grid: np.ndarray) -> List[Coord]:
    """
    Frontier cell definition:
      cell is FREE and has at least one UNKNOWN neighbor (4-neighborhood is typical).
    """
    frontiers: List[Coord] = []
    H, W = grid.shape
    for r in range(H):
        for c in range(W):
            if not is_free(int(grid[r, c])):
                continue
            for rr, cc in neighbors4(grid, (r, c)):
                if is_unknown(int(grid[rr, cc])):
                    frontiers.append((r, c))
                    break
    return frontiers


def cluster_frontiers(grid: np.ndarray, frontier_cells: List[Coord]) -> List[FrontierCluster]:
    """
    Cluster frontier cells using 8-connectivity (common choice).
    """
    frontier_set: Set[Coord] = set(frontier_cells)
    visited: Set[Coord] = set()
    clusters: List[FrontierCluster] = []

    for cell in frontier_cells:
        if cell in visited:
            continue
        if cell not in frontier_set:
            continue
        q = deque([cell])
        visited.add(cell)
        comp: List[Coord] = []

        while q:
            u = q.popleft()
            comp.append(u)
            for v in neighbors8(grid, u):
                if v in frontier_set and v not in visited:
                    visited.add(v)
                    q.append(v)

        clusters.append(FrontierCluster(comp))

    # Optional: sort big-to-small for stable behavior
    clusters.sort(key=lambda cl: cl.size, reverse=True)
    return clusters


def bfs_shortest_path_length(grid: np.ndarray, start: Coord) -> np.ndarray:
    """
    Compute shortest path length from start to all reachable FREE cells using BFS (4-neighborhood),
    assuming uniform cost. Obstacles and unknown cells are treated as blocked.
    Returns an array dist with +inf for unreachable.
    """
    H, W = grid.shape
    dist = np.full((H, W), np.inf, dtype=float)
    if not is_free(int(grid[start[0], start[1]])):
        return dist

    q = deque([start])
    dist[start[0], start[1]] = 0.0

    while q:
        u = q.popleft()
        du = dist[u[0], u[1]]
        for v in neighbors4(grid, u):
            if not is_free(int(grid[v[0], v[1]])):
                continue
            if dist[v[0], v[1]] == np.inf:
                dist[v[0], v[1]] = du + 1.0
                q.append(v)
    return dist


def choose_goal_near_cluster(
    grid: np.ndarray,
    dist_from_robot: np.ndarray,
    cluster: FrontierCluster,
    max_search_radius: int = 6,
) -> Optional[Coord]:
    """
    Choose a reachable FREE goal near the cluster centroid.
    Strategy:
      - compute centroid (float)
      - search a growing square window around rounded centroid
      - among candidates that are FREE and reachable, pick minimum dist_from_robot
    """
    cr, cc = cluster.centroid()
    r0, c0 = int(round(cr)), int(round(cc))

    best: Optional[Coord] = None
    best_d = np.inf

    for rad in range(0, max_search_radius + 1):
        for r in range(r0 - rad, r0 + rad + 1):
            for c in range(c0 - rad, c0 + rad + 1):
                if not in_bounds(grid, (r, c)):
                    continue
                if not is_free(int(grid[r, c])):
                    continue
                d = dist_from_robot[r, c]
                if not np.isfinite(d):
                    continue
                if d < best_d:
                    best_d = d
                    best = (r, c)
        if best is not None:
            return best
    return None


def score_cluster(distance: float, size: int, alpha: float = 1.0, beta: float = 3.0) -> float:
    """
    Utility-inspired cost:
      J = alpha * distance - beta * size
    Lower is better.
    """
    return alpha * float(distance) - beta * float(size)


def select_best_frontier_goal(
    grid: np.ndarray,
    robot: Coord,
    alpha: float = 1.0,
    beta: float = 3.0,
    min_cluster_size: int = 5,
) -> Tuple[Optional[Coord], List[FrontierCluster]]:
    """
    Returns:
      goal coordinate (row, col) or None if no suitable frontier
      list of clusters (for debugging/inspection)
    """
    frontier_cells = detect_frontiers(grid)
    if not frontier_cells:
        return None, []

    clusters = cluster_frontiers(grid, frontier_cells)
    clusters = [cl for cl in clusters if cl.size >= min_cluster_size]
    if not clusters:
        return None, []

    dist = bfs_shortest_path_length(grid, robot)

    best_goal: Optional[Coord] = None
    best_cost = np.inf

    for cl in clusters:
        goal = choose_goal_near_cluster(grid, dist, cl)
        if goal is None:
            continue
        d = dist[goal[0], goal[1]]
        J = score_cluster(d, cl.size, alpha=alpha, beta=beta)
        if J < best_cost:
            best_cost = J
            best_goal = goal

    return best_goal, clusters


def simulate_sensor_update(grid: np.ndarray, center: Coord, radius: int = 4) -> None:
    """
    Toy sensing: "reveal" unknown cells inside a square radius around center.
    In a real system, the mapper updates cells based on sensor rays / inverse sensor model.
    """
    r0, c0 = center
    H, W = grid.shape
    for r in range(max(0, r0 - radius), min(H, r0 + radius + 1)):
        for c in range(max(0, c0 - radius), min(W, c0 + radius + 1)):
            if is_unknown(int(grid[r, c])):
                grid[r, c] = 0  # reveal as free (toy)


def demo() -> None:
    # Build a toy map: unknown everywhere, with a known free "starting room" and obstacles
    H, W = 30, 40
    grid = np.full((H, W), -1, dtype=int)

    # Known free region
    grid[5:15, 5:18] = 0
    # Obstacles (walls)
    grid[5:15, 12] = 100
    grid[10, 12:30] = 100
    # Another known pocket
    grid[18:25, 25:35] = 0

    robot = (8, 8)

    for step in range(1, 9):
        goal, clusters = select_best_frontier_goal(grid, robot, alpha=1.0, beta=2.5, min_cluster_size=6)
        print(f"[step {step}] robot={robot} frontier_clusters={len(clusters)} goal={goal}")
        if goal is None:
            print("No frontier goal found. Exploration may be complete (reachable area).")
            break
        # Move robot to goal (toy: teleport)
        robot = goal
        simulate_sensor_update(grid, robot, radius=4)

    # Show a coarse final count
    unknown_count = int(np.sum(grid == -1))
    print("Remaining unknown cells:", unknown_count)


if __name__ == "__main__":
    demo()
