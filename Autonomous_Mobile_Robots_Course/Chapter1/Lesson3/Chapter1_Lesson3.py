"""
Chapter 1 — Lesson 3: Environment Representations for AMR
File: Chapter1_Lesson3.py

This script builds a simple 2D environment, rasterizes polygon obstacles into an
occupancy grid, computes a signed distance field (SDF), and performs
configuration-space inflation using a disc robot approximation.

Dependencies:
  - numpy
  - scipy (optional; used for Euclidean distance transform)
"""

import math
import numpy as np

try:
    from scipy.ndimage import distance_transform_edt as edt
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def point_in_polygon(px: float, py: float, poly_xy):
    """
    Ray casting test for point inside a simple polygon.
    poly_xy: list of (x,y) vertices (closed or open).
    Returns True if inside, False if outside (boundary treated as inside).
    """
    inside = False
    n = len(poly_xy)
    for i in range(n):
        x1, y1 = poly_xy[i]
        x2, y2 = poly_xy[(i + 1) % n]
        # Check if edge intersects ray to +infty in x direction
        cond = ((y1 > py) != (y2 > py))
        if cond:
            x_int = x1 + (py - y1) * (x2 - x1) / (y2 - y1 + 1e-15)
            if px <= x_int:
                inside = not inside
    return inside


def rasterize_polygons(polys, x_min, x_max, y_min, y_max, resolution):
    """
    Rasterize polygons into an occupancy grid.
    Returns:
      occ: (H,W) uint8 with 1=occupied, 0=free
      xs, ys: coordinate arrays for cell centers
    """
    xs = np.arange(x_min + 0.5 * resolution, x_max, resolution)
    ys = np.arange(y_min + 0.5 * resolution, y_max, resolution)
    W = xs.size
    H = ys.size
    occ = np.zeros((H, W), dtype=np.uint8)

    for r, y in enumerate(ys):
        for c, x in enumerate(xs):
            for poly in polys:
                if point_in_polygon(float(x), float(y), poly):
                    occ[r, c] = 1
                    break
    return occ, xs, ys


def brushfire_manhattan(binary_occ):
    """
    Multi-source BFS distance to obstacles (Manhattan distance in grid cells).
    binary_occ: 1=occupied, 0=free.
    Returns dist: int32 array, dist=0 on occupied cells.
    """
    H, W = binary_occ.shape
    INF = 10**9
    dist = np.full((H, W), INF, dtype=np.int32)

    # Queue arrays for speed and reproducibility (no deque dependency)
    qx = np.empty(H * W, dtype=np.int32)
    qy = np.empty(H * W, dtype=np.int32)
    head = 0
    tail = 0

    for r in range(H):
        for c in range(W):
            if binary_occ[r, c] == 1:
                dist[r, c] = 0
                qx[tail] = r
                qy[tail] = c
                tail += 1

    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while head < tail:
        r = int(qx[head])
        c = int(qy[head])
        head += 1
        d = int(dist[r, c])
        for dr, dc in nbrs:
            rr = r + dr
            cc = c + dc
            if 0 <= rr < H and 0 <= cc < W:
                if dist[rr, cc] > d + 1:
                    dist[rr, cc] = d + 1
                    qx[tail] = rr
                    qy[tail] = cc
                    tail += 1

    return dist


def signed_distance_field(occ, resolution):
    """
    Compute signed distance field (SDF) in meters:
      SDF > 0 in free space, SDF < 0 inside obstacles.
    If SciPy is available: Euclidean transform; else Manhattan transform.
    """
    occ_bool = (occ.astype(np.uint8) == 1)
    free_bool = ~occ_bool

    if _HAS_SCIPY:
        # Distance from free cells to nearest obstacle
        d_free_to_obs = edt(free_bool) * float(resolution)
        # Distance from obstacle cells to nearest free
        d_obs_to_free = edt(occ_bool) * float(resolution)
    else:
        d_free_to_obs = brushfire_manhattan(occ) * float(resolution)
        d_obs_to_free = brushfire_manhattan(1 - occ) * float(resolution)

    sdf = d_free_to_obs.copy().astype(np.float64)
    sdf[occ_bool] = -d_obs_to_free[occ_bool].astype(np.float64)
    return sdf


def inflate_obstacles_via_sdf(sdf, robot_radius_m):
    """
    Inflate obstacles for a disc robot (point-robot reduction):
      A cell is safe iff SDF >= robot_radius_m.
    Returns inflated occupancy (1=occupied after inflation).
    """
    inflated_occ = (sdf < float(robot_radius_m)).astype(np.uint8)
    return inflated_occ


def main():
    # Environment bounds and resolution
    x_min, x_max = 0.0, 10.0
    y_min, y_max = 0.0, 10.0
    resolution = 0.05  # 5 cm grid

    # Polygon obstacles (counterclockwise vertices)
    polys = [
        [(2.0, 2.0), (4.5, 2.0), (4.5, 3.5), (2.0, 3.5)],
        [(6.0, 6.0), (8.5, 6.2), (8.2, 8.0), (6.2, 8.3)],
        [(1.0, 7.0), (2.0, 6.0), (3.0, 7.0), (2.0, 8.0)],
    ]

    occ, xs, ys = rasterize_polygons(polys, x_min, x_max, y_min, y_max, resolution)
    sdf = signed_distance_field(occ, resolution)

    robot_radius = 0.30  # 30 cm disc approximation
    occ_infl = inflate_obstacles_via_sdf(sdf, robot_radius)

    # Save artifacts for downstream lessons (e.g., planning, localization)
    np.savetxt("occupancy_grid.csv", occ, fmt="%d", delimiter=",")
    np.savetxt("inflated_occupancy_grid.csv", occ_infl, fmt="%d", delimiter=",")
    np.savetxt("signed_distance_field.csv", sdf, fmt="%.6f", delimiter=",")

    # Minimal textual report
    free_ratio = float(np.mean(occ == 0))
    infl_free_ratio = float(np.mean(occ_infl == 0))
    print("SciPy available:", _HAS_SCIPY)
    print("Grid size (H,W):", occ.shape)
    print("Free ratio (original):", free_ratio)
    print("Free ratio (inflated):", infl_free_ratio)
    print("Files written: occupancy_grid.csv, inflated_occupancy_grid.csv, signed_distance_field.csv")


if __name__ == "__main__":
    main()
