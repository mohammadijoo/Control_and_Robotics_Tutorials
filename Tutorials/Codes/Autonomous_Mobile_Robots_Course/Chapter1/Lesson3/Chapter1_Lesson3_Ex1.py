"""
Chapter 1 — Lesson 3 (Exercise 1)
File: Chapter1_Lesson3_Ex1.py

Exercise: Implement a multi-source BFS distance transform for a binary occupancy grid.
  - Input: occ (H,W) uint8 where 1=occupied, 0=free
  - Output: dist (H,W) int32 where dist=0 at occupied cells, and each free cell
            stores the Manhattan distance (in grid steps) to the nearest obstacle.

Instructions:
  1) Fill in the function bfs_distance_transform.
  2) Verify correctness on the provided test case.
"""

import numpy as np


def bfs_distance_transform(occ: np.ndarray) -> np.ndarray:
    H, W = occ.shape
    INF = 10**9
    dist = np.full((H, W), INF, dtype=np.int32)

    # TODO: initialize queue with all occupied cells (dist=0)
    # TODO: pop cells and relax 4-neighbors
    # TODO: return dist

    return dist


def _toy_test():
    occ = np.zeros((5, 7), dtype=np.uint8)
    occ[2, 3] = 1  # single obstacle
    dist = bfs_distance_transform(occ)
    print("occ:\n", occ)
    print("dist:\n", dist)


if __name__ == "__main__":
    _toy_test()
