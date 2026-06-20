"""
Autonomous Mobile Robots — Chapter 17, Lesson 1 (Exercise 1)
File: Chapter17_Lesson1_Ex1.py

Exercise: Implement frontier clustering.

You are given a list of frontier cells (row, col) and must return clusters
as lists of cells. Use 8-connectivity. Then compute the size of each cluster.

Start from the skeleton below.
"""
from __future__ import annotations

from collections import deque
from typing import List, Tuple, Set

import numpy as np

Coord = Tuple[int, int]


def neighbors8(grid: np.ndarray, rc: Coord) -> List[Coord]:
    r, c = rc
    out = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < grid.shape[0] and 0 <= cc < grid.shape[1]:
                out.append((rr, cc))
    return out


def cluster_frontiers_exercise(grid: np.ndarray, frontier_cells: List[Coord]) -> List[List[Coord]]:
    """
    TODO:
      - Use BFS/DFS to group frontier_cells into connected components under 8-connectivity.
      - Return a list of clusters, each a list of cells.

    Hints:
      - Convert frontier_cells to a set for O(1) membership.
      - Keep a visited set.
    """
    frontier_set: Set[Coord] = set(frontier_cells)
    visited: Set[Coord] = set()
    clusters: List[List[Coord]] = []

    # TODO: implement
    raise NotImplementedError("Implement me!")


if __name__ == "__main__":
    # Simple test
    grid = np.zeros((6, 6), dtype=int)
    frontiers = [(1, 1), (1, 2), (2, 2), (4, 4)]
    print(cluster_frontiers_exercise(grid, frontiers))
