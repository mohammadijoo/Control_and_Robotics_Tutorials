import numpy as np
from heapq import heappush, heappop

# ---------- Geometry and collision model (2D example) ----------

def sample_free(n_samples, bounds, obstacles):
    """
    Uniformly sample configurations in C_free in 2D.
    bounds = ((x_min, x_max), (y_min, y_max))
    obstacles is a list of axis-aligned boxes (x_min, y_min, x_max, y_max).
    """
    xs = np.random.uniform(bounds[0][0], bounds[0][1], size=n_samples * 2)
    ys = np.random.uniform(bounds[1][0], bounds[1][1], size=n_samples * 2)
    pts = []
    for x, y in zip(xs, ys):
        if not in_collision((x, y), obstacles):
            pts.append((x, y))
        if len(pts) == n_samples:
            break
    return np.array(pts)


def in_collision(q, obstacles):
    x, y = q
    for (xmin, ymin, xmax, ymax) in obstacles:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False


def interpolate(q1, q2, n_steps):
    """Straight line interpolation in configuration space."""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    return [q1 + float(t) / float(n_steps) * (q2 - q1) for t in range(n_steps + 1)]


def edge_collision_free(q1, q2, obstacles, n_steps=20):
    """Check discrete collision along segment q1-q2."""
    for q in interpolate(q1, q2, n_steps):
        if in_collision(q, obstacles):
            return False
    return True


def euclidean(q1, q2):
    return float(np.linalg.norm(np.asarray(q1) - np.asarray(q2)))


# ---------- PRM Roadmap construction ----------

def build_prm(samples, r, obstacles):
    """
    Build PRM roadmap.
    samples: (N,2) array of points in C_free.
    r: connection radius.
    Returns adjacency list: graph[i] = list of (j, cost).
    """
    n = samples.shape[0]
    graph = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d = euclidean(samples[i], samples[j])
            if d <= r:
                if edge_collision_free(samples[i], samples[j], obstacles):
                    graph[i].append((j, d))
                    graph[j].append((i, d))
    return graph


def add_node_to_graph(q, samples, graph, r, obstacles):
    """
    Insert a new node q into an existing roadmap.
    Returns new index and updated samples and graph.
    """
    samples = np.vstack([samples, np.asarray(q)])
    idx = samples.shape[0] - 1
    graph.append([])
    for j in range(idx):
        d = euclidean(samples[idx], samples[j])
        if d <= r:
            if edge_collision_free(samples[idx], samples[j], obstacles):
                graph[idx].append((j, d))
                graph[j].append((idx, d))
    return idx, samples, graph


# ---------- Dijkstra shortest path ----------

def dijkstra(graph, start, goal):
    """
    graph: adjacency list with (neighbor, cost).
    start, goal: integer indices.
    """
    n = len(graph)
    dist = [np.inf] * n
    prev = [-1] * n
    dist[start] = 0.0
    pq = [(0.0, start)]
    while pq:
        d, u = heappop(pq)
        if u == goal:
            break
        if d > dist[u]:
            continue
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heappush(pq, (nd, v))
    if dist[goal] == np.inf:
        return None
    # reconstruct path
    path = []
    u = goal
    while u != -1:
        path.append(u)
        u = prev[u]
    path.reverse()
    return path


# ---------- Lazy-PRM modifications ----------

def build_lazy_prm(samples, r):
    """
    Build a 'hypothetical' roadmap: edges added based on distance only,
    with no collision checking yet.
    """
    n = samples.shape[0]
    graph = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d = euclidean(samples[i], samples[j])
            if d <= r:
                graph[i].append((j, d))
                graph[j].append((i, d))
    return graph


def validate_path_lazy(path, samples, graph, obstacles, n_steps=20):
    """
    Validate edges along a path. If an edge is invalid, remove it from graph.
    Returns (valid, first_invalid_edge) where first_invalid_edge is (u, v) or None.
    """
    for u, v in zip(path[:-1], path[1:]):
        if not edge_collision_free(samples[u], samples[v], obstacles, n_steps=n_steps):
            # remove edge symmetrically
            graph[u] = [(nbr, w) for (nbr, w) in graph[u] if nbr != v]
            graph[v] = [(nbr, w) for (nbr, w) in graph[v] if nbr != u]
            return False, (u, v)
    return True, None


def lazy_prm_query(samples, graph, q_start, q_goal, r, obstacles):
    """
    Lazy-PRM query on pre-sampled roadmap nodes.
    """
    # Add start and goal nodes without collision checking on edges.
    samples = np.vstack([samples, np.asarray(q_start), np.asarray(q_goal)])
    idx_start = samples.shape[0] - 2
    idx_goal = samples.shape[0] - 1
    graph.append([])
    graph.append([])
    # connect start and goal to neighbors by distance only
    for idx in [idx_start, idx_goal]:
        for j in range(idx):
            d = euclidean(samples[idx], samples[j])
            if d <= r:
                graph[idx].append((j, d))
                graph[j].append((idx, d))

    while True:
        path = dijkstra(graph, idx_start, idx_goal)
        if path is None:
            return None  # no path exists
        valid, bad_edge = validate_path_lazy(path, samples, graph, obstacles)
        if valid:
            return [samples[i] for i in path]
        # else loop again with reduced graph


# Example usage (for testing):
if __name__ == "__main__":
    np.random.seed(0)
    bounds = ((0.0, 1.0), (0.0, 1.0))
    obstacles = [(0.3, 0.3, 0.6, 0.7)]
    samples = sample_free(200, bounds, obstacles)
    r = 0.15

    # Standard PRM
    graph = build_prm(samples, r, obstacles)
    q_start = (0.1, 0.1)
    q_goal = (0.9, 0.9)
    idx_start, samples, graph = add_node_to_graph(q_start, samples, graph, r, obstacles)
    idx_goal, samples, graph = add_node_to_graph(q_goal, samples, graph, r, obstacles)
    path_idx = dijkstra(graph, idx_start, idx_goal)
    prm_path = [samples[i] for i in path_idx] if path_idx is not None else None

    # Lazy-PRM
    samples_lazy = sample_free(200, bounds, obstacles)
    graph_lazy = build_lazy_prm(samples_lazy, r)
    lazy_path = lazy_prm_query(samples_lazy, graph_lazy, q_start, q_goal, r, obstacles)
      
