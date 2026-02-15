import heapq
import math
import numpy as np

# Discretization parameters
DX = 1.0
DY = 1.0
N_THETA = 16  # number of discrete headings
DT = 1.0      # primitive duration
V = 1.0       # forward speed
L = 2.0       # wheelbase

# Occupancy grid: 0 = free, 1 = obstacle
# Here we just create a small empty grid as an example.
NX, NY = 50, 30
occ_grid = np.zeros((NX, NY), dtype=np.uint8)

def in_bounds(ix, iy):
    return 0 <= ix < NX and 0 <= iy < NY

def is_free(ix, iy):
    return in_bounds(ix, iy) and occ_grid[ix, iy] == 0

def angle_index(theta):
    """Map continuous angle to discrete index."""
    theta = (theta + 2.0 * math.pi) % (2.0 * math.pi)
    k = int(round(theta / (2.0 * math.pi) * N_THETA)) % N_THETA
    return k

def angle_from_index(k):
    return 2.0 * math.pi * k / N_THETA

# Precompute reference motion primitives at pose (0,0,0)
STEER_ANGLES = [-0.4, 0.0, 0.4]  # radians, simple left/straight/right
ref_primitives = []  # each entry: (dx, dy, dtheta, length)

for phi in STEER_ANGLES:
    kappa = math.tan(phi) / L  # curvature
    if abs(kappa) < 1e-6:
        # straight-line primitive
        dx = V * DT
        dy = 0.0
        dtheta = 0.0
        length = V * DT
    else:
        # circular-arc primitive
        dtheta = V * kappa * DT
        th_end = dtheta  # from 0 to dtheta
        dx = (1.0 / kappa) * math.sin(th_end)
        dy = -(1.0 / kappa) * (math.cos(th_end) - 1.0)
        length = abs(dtheta / kappa)
    ref_primitives.append((dx, dy, dtheta, length))

def successors(state):
    """
    state = (ix, iy, kth)
    Returns list of (next_state, cost).
    """
    ix, iy, kth = state
    theta = angle_from_index(kth)
    succs = []
    for dx_ref, dy_ref, dth_ref, length in ref_primitives:
        # transform reference primitive to global frame
        gx = ix * DX
        gy = iy * DY
        dx = math.cos(theta) * dx_ref - math.sin(theta) * dy_ref
        dy = math.sin(theta) * dx_ref + math.cos(theta) * dy_ref
        gx2 = gx + dx
        gy2 = gy + dy
        kth2 = angle_index(theta + dth_ref)
        ix2 = int(round(gx2 / DX))
        iy2 = int(round(gy2 / DY))

        if not is_free(ix2, iy2):
            continue

        cost = length  # use arc length as cost
        succs.append(((ix2, iy2, kth2), cost))
    return succs

def heuristic(state, goal_state):
    ix, iy, _ = state
    gx, gy, _ = goal_state
    x = ix * DX
    y = iy * DY
    xg = gx * DX
    yg = gy * DY
    return math.hypot(x - xg, y - yg)

def astar(start_state, goal_state):
    """
    A* search on the lattice.
    state = (ix, iy, kth)
    """
    open_heap = []
    g = {start_state: 0.0}
    parent = {start_state: None}
    f_start = heuristic(start_state, goal_state)
    heapq.heappush(open_heap, (f_start, start_state))

    closed = set()

    while open_heap:
        f_curr, curr = heapq.heappop(open_heap)
        if curr in closed:
            continue
        closed.add(curr)

        if (curr[0], curr[1]) == (goal_state[0], goal_state[1]):
            # reached goal cell (ignoring exact orientation)
            path = []
            s = curr
            while s is not None:
                path.append(s)
                s = parent[s]
            path.reverse()
            return path, g[curr]

        for nbr, cost in successors(curr):
            if nbr in closed:
                continue
            new_g = g[curr] + cost
            if new_g < g.get(nbr, float("inf")):
                g[nbr] = new_g
                parent[nbr] = curr
                f_nbr = new_g + heuristic(nbr, goal_state)
                heapq.heappush(open_heap, (f_nbr, nbr))

    return None, float("inf")

# Example usage
start = (2, 2, angle_index(0.0))
goal = (40, 20, angle_index(0.0))
path, cost = astar(start, goal)
print("Path length:", len(path), "Total cost:", cost)
      
