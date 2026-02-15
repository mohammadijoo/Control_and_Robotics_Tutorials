import numpy as np
from dataclasses import dataclass

# Joint limits for 6-DOF arm (example values, radians)
Q_MIN = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
Q_MAX = np.array([ np.pi,  np.pi,  np.pi,  np.pi,  np.pi,  np.pi])

@dataclass
class Node:
    q: np.ndarray
    parent: int
    cost: float  # cost-to-come

def sample_config():
    return Q_MIN + (Q_MAX - Q_MIN) * np.random.rand(6)

def distance(q1, q2, w=None):
    if w is None:
        w = np.ones(6)
    diff = (q1 - q2) * w
    return float(np.linalg.norm(diff))

def forward_kinematics(q):
    """
    Placeholder: return link frames or sample points on the robot.
    Use your existing DH parameters or rigid-body model here.
    """
    raise NotImplementedError

def in_collision(q, obstacles):
    """
    Placeholder: use forward_kinematics(q) and distance checks vs obstacles.
    Return True if the robot at configuration q intersects any obstacle.
    """
    raise NotImplementedError

def edge_collision_free(q1, q2, obstacles, step=0.05):
    d = distance(q1, q2)
    if d == 0.0:
        return not in_collision(q1, obstacles)
    steps = int(np.ceil(d / step))
    for k in range(steps + 1):
        alpha = k / max(steps, 1)
        q = (1.0 - alpha) * q1 + alpha * q2
        if in_collision(q, obstacles):
            return False
    return True

def steer(q_from, q_to, eta):
    d = distance(q_from, q_to)
    if d <= eta:
        return q_to
    alpha = eta / d
    return q_from + alpha * (q_to - q_from)

def nearest(nodes, q_rand):
    d_min = float("inf")
    idx_min = -1
    for i, node in enumerate(nodes):
        d = distance(node.q, q_rand)
        if d < d_min:
            d_min = d
            idx_min = i
    return idx_min

def near_indices(nodes, q_new, gamma, eta):
    n = len(nodes)
    if n == 0:
        return []
    r = min(eta, gamma * (np.log(n + 1) / (n + 1)) ** (1.0 / 6.0))
    idxs = []
    for i, node in enumerate(nodes):
        if distance(node.q, q_new) <= r:
            idxs.append(i)
    return idxs

def extract_path(nodes, idx_goal):
    path = []
    idx = idx_goal
    while idx != -1:
        path.append(nodes[idx].q)
        idx = nodes[idx].parent
    path.reverse()
    return np.array(path)

def rrt_star(q_start, q_goal, obstacles,
             eta=0.3, gamma=2.0,
             max_iter=5000, goal_radius=0.1):
    nodes = [Node(q=np.array(q_start, dtype=float), parent=-1, cost=0.0)]
    goal_index = None

    for it in range(max_iter):
        q_rand = sample_config()
        i_near = nearest(nodes, q_rand)
        q_near = nodes[i_near].q
        q_new = steer(q_near, q_rand, eta)

        if in_collision(q_new, obstacles):
            continue
        if not edge_collision_free(q_near, q_new, obstacles):
            continue

        # Choose parent with minimal cost-to-come
        idxs_near = near_indices(nodes, q_new, gamma, eta)
        cost_best = nodes[i_near].cost + distance(nodes[i_near].q, q_new)
        parent_best = i_near
        for j in idxs_near:
            if not edge_collision_free(nodes[j].q, q_new, obstacles):
                continue
            c = nodes[j].cost + distance(nodes[j].q, q_new)
            if c < cost_best:
                cost_best = c
                parent_best = j

        nodes.append(Node(q=q_new, parent=parent_best, cost=cost_best))
        i_new = len(nodes) - 1

        # Rewire neighbors through q_new if cheaper
        for j in idxs_near:
            if j == parent_best:
                continue
            if not edge_collision_free(q_new, nodes[j].q, obstacles):
                continue
            c_through_new = nodes[i_new].cost + distance(q_new, nodes[j].q)
            if c_through_new + 1e-9 < nodes[j].cost:
                nodes[j].parent = i_new
                nodes[j].cost = c_through_new

        # Check goal
        if distance(q_new, q_goal) <= goal_radius:
            if edge_collision_free(q_new, q_goal, obstacles):
                nodes.append(
                    Node(q=np.array(q_goal, dtype=float),
                         parent=i_new,
                         cost=nodes[i_new].cost + distance(q_new, q_goal))
                )
                goal_index = len(nodes) - 1
                break

    if goal_index is None:
        return None  # planning failed
    return extract_path(nodes, goal_index)

def prm(q_start, q_goal, obstacles,
        n_samples=1000, k_neighbors=10):
    """
    Simple PRM: uniform sampling, k-nearest neighbors with local collision check.
    Returns a list of configurations forming a path, or None.
    """
    samples = []
    for _ in range(n_samples):
        q = sample_config()
        if not in_collision(q, obstacles):
            samples.append(q)
    samples = [np.array(q, dtype=float) for q in samples]

    # build adjacency list
    n = len(samples)
    adj = [[] for _ in range(n)]

    for i in range(n):
        # find k nearest neighbors
        dists = [(j, distance(samples[i], samples[j])) for j in range(n) if j != i]
        dists.sort(key=lambda p: p[1])
        for j, d in dists[:k_neighbors]:
            if edge_collision_free(samples[i], samples[j], obstacles):
                adj[i].append(j)
                adj[j].append(i)

    # include start and goal as extra nodes
    nodes = [np.array(q_start, dtype=float)] + samples + [np.array(q_goal, dtype=float)]
    start_idx = 0
    goal_idx = len(nodes) - 1
    adj_ext = [[] for _ in range(len(nodes))]
    # copy PRM adjacency
    for i in range(n):
        for j in adj[i]:
            adj_ext[i + 1].append(j + 1)
    # connect start and goal to nearest neighbors
    def connect_special(idx_special, q_special):
        dists = [(i + 1, distance(q_special, samples[i])) for i in range(n)]
        dists.sort(key=lambda p: p[1])
        for j, d in dists[:k_neighbors]:
            if edge_collision_free(q_special, samples[j - 1], obstacles):
                adj_ext[idx_special].append(j)
                adj_ext[j].append(idx_special)

    connect_special(start_idx, nodes[start_idx])
    connect_special(goal_idx, nodes[goal_idx])

    # Dijkstra's algorithm
    import heapq
    INF = float("inf")
    dist_arr = [INF] * len(nodes)
    parent = [-1] * len(nodes)
    dist_arr[start_idx] = 0.0
    heap = [(0.0, start_idx)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist_arr[u]:
            continue
        if u == goal_idx:
            break
        for v in adj_ext[u]:
            w = distance(nodes[u], nodes[v])
            nd = d + w
            if nd < dist_arr[v]:
                dist_arr[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd, v))

    if dist_arr[goal_idx] == INF:
        return None

    # reconstruct path
    path = []
    u = goal_idx
    while u != -1:
        path.append(nodes[u])
        u = parent[u]
    path.reverse()
    return np.array(path)
      
