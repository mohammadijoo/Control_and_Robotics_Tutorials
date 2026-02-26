"""
Chapter14_Lesson2.py
Costmaps and Inflation Concepts — reference implementation (grid + inflation).
This script builds a toy occupancy grid, computes an approximate Euclidean distance
field via an 8-connected brushfire, and produces an inflated costmap.
"""

from collections import deque
import math
from typing import List

Grid = List[List[int]]  # 0 free, 1 obstacle

def make_toy_map(w: int = 60, h: int = 40) -> Grid:
    g = [[0 for _ in range(w)] for _ in range(h)]
    # border
    for x in range(w):
        g[0][x] = 1
        g[h-1][x] = 1
    for y in range(h):
        g[y][0] = 1
        g[y][w-1] = 1
    # internal obstacles
    for x in range(10, 50):
        g[12][x] = 1
    for y in range(18, 33):
        g[y][28] = 1
    for x in range(35, 55):
        g[28][x] = 1
    return g

def brushfire_distance(obstacles: Grid, resolution: float) -> List[List[float]]:
    """
    Approximate Euclidean distance transform using multi-source BFS.
    8-neighborhood with step costs {1, sqrt(2)} * resolution.
    """
    h = len(obstacles)
    w = len(obstacles[0])
    INF = 1e9
    dist = [[INF for _ in range(w)] for _ in range(h)]
    q = deque()

    # initialize all obstacle cells
    for y in range(h):
        for x in range(w):
            if obstacles[y][x] == 1:
                dist[y][x] = 0.0
                q.append((x, y))

    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    while q:
        x, y = q.popleft()
        dxy = dist[y][x]
        for dx, dy, step in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                nd = dxy + step * resolution
                if nd < dist[ny][nx]:
                    dist[ny][nx] = nd
                    q.append((nx, ny))
    return dist

def inflation_cost_from_distance(d: float, r_inscribed: float, r_inflation: float,
                                cost_scaling: float,
                                lethal_cost: int = 254,
                                inscribed_cost: int = 253) -> int:
    """
    Common inflation cost model:
      - obstacle cell: lethal_cost
      - 0 < d <= r_inscribed: inscribed_cost
      - r_inscribed < d <= r_inflation: exponential decay
      - d > r_inflation: 0
    """
    if d <= 0.0:
        return lethal_cost
    if d <= r_inscribed:
        return inscribed_cost
    if d > r_inflation:
        return 0

    # decay from (inscribed_cost-1) down to 1, then clamp
    c = (inscribed_cost - 1) * math.exp(-cost_scaling * (d - r_inscribed)) + 1.0
    c_int = int(max(1, min(inscribed_cost - 1, round(c))))
    return c_int

def build_inflated_costmap(obstacles: Grid, resolution: float,
                           r_inscribed: float, r_inflation: float,
                           cost_scaling: float) -> List[List[int]]:
    dist = brushfire_distance(obstacles, resolution)
    h = len(obstacles)
    w = len(obstacles[0])
    cm = [[0 for _ in range(w)] for _ in range(h)]
    for y in range(h):
        for x in range(w):
            cm[y][x] = inflation_cost_from_distance(
                dist[y][x], r_inscribed, r_inflation, cost_scaling
            )
    return cm

def ascii_render_costmap(costmap: List[List[int]]) -> str:
    """
    Render costmap with ASCII characters:
      '#' lethal, '+' high, '.' low, ' ' free
    """
    lines = []
    for row in costmap:
        s = []
        for c in row:
            if c >= 254:
                s.append('#')
            elif c >= 200:
                s.append('X')
            elif c >= 120:
                s.append('+')
            elif c >= 40:
                s.append('.')
            else:
                s.append(' ')
        lines.append(''.join(s))
    return "\n".join(lines)

def main():
    resolution = 0.05  # m/cell
    r_inscribed = 0.25 # m
    r_inflation = 0.60 # m
    cost_scaling = 6.0 # 1/m

    obs = make_toy_map()
    costmap = build_inflated_costmap(obs, resolution, r_inscribed, r_inflation, cost_scaling)
    print("Inflated costmap (ASCII preview):")
    print(ascii_render_costmap(costmap))

if __name__ == "__main__":
    main()
