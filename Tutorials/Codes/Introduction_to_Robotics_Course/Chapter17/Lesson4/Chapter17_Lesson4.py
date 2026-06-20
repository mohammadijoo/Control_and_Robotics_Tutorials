import numpy as np

UNKNOWN = -1
FREE = 0
OBSTACLE = 1

def is_frontier(occ_grid, i, j):
    if occ_grid[i, j] != FREE:
        return False
    # 4-neighborhood
    neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
    for ni, nj in neighbors:
        if 0 <= ni < occ_grid.shape[0] and 0 <= nj < occ_grid.shape[1]:
            if occ_grid[ni, nj] == UNKNOWN:
                return True
    return False

def select_closest_frontier(occ_grid, robot_idx):
    ri, rj = robot_idx
    frontiers = []
    for i in range(occ_grid.shape[0]):
        for j in range(occ_grid.shape[1]):
            if is_frontier(occ_grid, i, j):
                frontiers.append((i, j))
    if not frontiers:
        return None  # exploration complete
    # choose frontier with minimum Euclidean distance
    dists = [np.linalg.norm(np.array([i - ri, j - rj])) for (i, j) in frontiers]
    idx = int(np.argmin(dists))
    return frontiers[idx]
      
