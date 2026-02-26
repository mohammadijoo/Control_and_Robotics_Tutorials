import math
from heapq import heappush, heappop

# ------------- State and lattice utilities -------------

LATTICE_DX = 1.0
LATTICE_DY = 1.0
N_THETA    = 16
DELTA_TH   = 2.0 * math.pi / N_THETA

def wrap_theta(th):
    """Wrap heading to [0, 2*pi)."""
    th = th % (2.0 * math.pi)
    return th

def quantize_state(x, y, th):
    """Map continuous state to lattice indices (i,j,k)."""
    i = int(round(x / LATTICE_DX))
    j = int(round(y / LATTICE_DY))
    k = int(round(wrap_theta(th) / DELTA_TH)) % N_THETA
    return (i, j, k)

def dequantize_state(i, j, k):
    """Map indices back to continuous state representative."""
    x = i * LATTICE_DX
    y = j * LATTICE_DY
    th = k * DELTA_TH
    return (x, y, th)

# ------------- Motion primitive generation -------------

class Primitive:
    def __init__(self, v, omega, T, dt=0.1):
        self.v = v
        self.omega = omega
        self.T = T
        self.dt = dt
        # Precompute displacement from canonical state (0,0,0)
        self.disp, self.cost = self._simulate_from(0.0, 0.0, 0.0)

    def _simulate_from(self, x0, y0, th0):
        x, y, th = x0, y0, th0
        t = 0.0
        cost = 0.0
        while t < self.T:
            # simple Euler integration of unicycle model
            x += self.v * math.cos(th) * self.dt
            y += self.v * math.sin(th) * self.dt
            th += self.omega * self.dt
            t += self.dt
            cost += self.dt  # time cost
        th = wrap_theta(th)
        return (x - x0, y - y0, th - th0), cost

# Define a small symmetric set of primitives in canonical frame
PRIMITIVES = [
    Primitive(v=1.0,  omega=0.0,  T=1.0),   # straight
    Primitive(v=1.0,  omega=0.8,  T=1.0),   # left arc
    Primitive(v=1.0,  omega=-0.8, T=1.0),   # right arc
]

def apply_primitive(i, j, k, prim):
    """Apply primitive starting from lattice node (i,j,k)."""
    x0, y0, th0 = dequantize_state(i, j, k)
    dx, dy, dth = prim.disp
    # rotate displacement by th0
    x1 = x0 + math.cos(th0) * dx - math.sin(th0) * dy
    y1 = y0 + math.sin(th0) * dx + math.cos(th0) * dy
    th1 = wrap_theta(th0 + dth)
    return quantize_state(x1, y1, th1), prim.cost

# ------------- Collision checking -------------

def is_free(i, j, occupancy):
    """Check grid occupancy: occupancy[j][i] == 0 means free."""
    if j < 0 or j >= len(occupancy):
        return False
    if i < 0 or i >= len(occupancy[0]):
        return False
    return occupancy[j][i] == 0

# ------------- A* search on lattice -------------

def heuristic(node, goal):
    (ix, iy, _) = node
    (gx, gy, _) = goal
    dx = (ix - gx) * LATTICE_DX
    dy = (iy - gy) * LATTICE_DY
    dist = math.hypot(dx, dy)
    v_max = 1.0
    return dist / v_max

def astar_lattice(start_state, goal_state, occupancy):
    start = quantize_state(*start_state)
    goal = quantize_state(*goal_state)

    open_heap = []
    heappush(open_heap, (0.0, start))
    g_score = {start: 0.0}
    parent = {start: None}

    while open_heap:
        _, current = heappop(open_heap)
        if current == goal:
            # reconstruct path of lattice nodes
            path = []
            n = current
            while n is not None:
                path.append(dequantize_state(*n))
                n = parent[n]
            path.reverse()
            return path

        (i, j, k) = current
        if not is_free(i, j, occupancy):
            continue

        for prim in PRIMITIVES:
            nxt, cost = apply_primitive(i, j, k, prim)
            (ni, nj, nk) = nxt
            if not is_free(ni, nj, occupancy):
                continue

            tentative_g = g_score[current] + cost
            old_g = g_score.get(nxt, float("inf"))
            if tentative_g < old_g:
                g_score[nxt] = tentative_g
                parent[nxt] = current
                f = tentative_g + heuristic(nxt, goal)
                heappush(open_heap, (f, nxt))

    return None  # no path found

# Example usage (simple empty grid)
if __name__ == "__main__":
    width, height = 20, 20
    occupancy = [[0 for _ in range(width)] for _ in range(height)]

    start_state = (1.0, 1.0, 0.0)
    goal_state  = (10.0, 10.0, 0.0)
    path = astar_lattice(start_state, goal_state, occupancy)
    print("Path length:", len(path) if path is not None else 0)
      
